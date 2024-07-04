#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.time import Time
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import math
from rclpy.duration import Duration
from time import time
from collections import Counter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.msg import LinkStates

'''
This script is responsible for converting messages from as twist messages 
to give commands to position controllers (for steer --front wheels) and velocity controllers (for velocity--rear wheels) 
and also publishes odometry data from gazebo_ros_state to /catvehicle/odom
'''

class Cmd_convertor(Node):

    def __init__(self):
        super().__init__('cmdvel2Cat')

        self.cmd_sub = self.create_subscription(Twist,"/cmd_vel",self.cmd_callback,10)
        self.state_sub = self.create_subscription(LinkStates,"/gazebo/link_states",self.state_callback,10)
        self.pub_steer = self.create_publisher(Float64MultiArray,'/forward_position_controller/commands',10)
        self.pub_vel = self.create_publisher(Float64MultiArray,'/velocity_controller/commands',10)
        self.odom = self.create_publisher(Odometry,'/catvehicle/odom',10)

        self.odom_loop = self.create_timer(0.03,self.timer_loop)
        
        self.odom_msg = Odometry()

        self.x = 0
        self.z = 0

        self.L = 2.62

        self.timout = Duration(seconds=0.2)
        self.lastMsg = Time()
        self.T=1.29
        self.maxsteerInside=0.6
        rMax = self.L/math.tan(self.maxsteerInside)
        rIdeal = rMax+(self.T/2.0)
        self.maxsteer=math.atan2(self.L,rIdeal)

        self.cnt = 0


    def cmd_callback(self,data:Twist):
        self.x = data.linear.x*2.6101
        self.z = max(-self.maxsteer,min(self.maxsteer,data.angular.z))
        self.lastMsg = Time()
        self.publish()
        

    def publish(self):
        self.get_logger().info("linear_velocity: "+str(self.x)+ " angular_velocity: "+str(self.z))
        if Time()-self.lastMsg > self.timout:
            self.x = 0
            return
        
        if self.z!=0:
            T=self.T
            L=self.L
            r = L/math.fabs(math.tan(self.z))

            rL = r-(math.copysign(1,self.z)*(T/2.0))
            rR = r+(math.copysign(1,self.z)*(T/2.0))
            msg_speed = Float64MultiArray()
    
            msgRearR = self.x*rR/r

            msgRearL = self.x*rL/r

            msg_speed.data = [msgRearL,msgRearR]

            self.pub_vel.publish(msg_speed)

            msg_steer = Float64MultiArray()
            msgSteerL = math.atan2(L,rL)*math.copysign(1,self.z)
            msgSteerR = math.atan2(L,rR)*math.copysign(1,self.z)

            msg_steer.data = [msgSteerL,msgSteerR]

            self.pub_steer.publish(msg_steer)

        else:
            msg_speed = Float64MultiArray()
            msg_speed.data = [self.x, self.x]
            self.pub_vel.publish(msg_speed)

            msg_steer = Float64MultiArray()
            msg_steer.data = [self.z, self.z]

            self.pub_steer.publish(msg_steer)
    
    def state_callback(self,data: LinkStates):
        pf = data.pose[1]  #index 1 for catvehicle
        tf = data.twist[1]
        # self.get_logger().info(f"{pf.position.x}")

        self.odom_msg.pose.pose.position.x = pf.position.x
        self.odom_msg.pose.pose.position.y = pf.position.y
        self.odom_msg.pose.pose.position.z = pf.position.z
        self.odom_msg.pose.pose.orientation.x = pf.orientation.x
        self.odom_msg.pose.pose.orientation.y = pf.orientation.y
        self.odom_msg.pose.pose.orientation.z = pf.orientation.z
        self.odom_msg.pose.pose.orientation.w = pf.orientation.w
        self.odom_msg.twist.twist.linear.x = tf.linear.x
        self.odom_msg.twist.twist.linear.y = tf.linear.y
        self.odom_msg.twist.twist.linear.z = tf.linear.z
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = tf.angular.z

        
    def timer_loop(self):
        self.odom.publish(self.odom_msg)


def main(args = None):
    rclpy.init(args=args)

    cmd_node = Cmd_convertor()
    rclpy.spin(cmd_node)
    cmd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()







