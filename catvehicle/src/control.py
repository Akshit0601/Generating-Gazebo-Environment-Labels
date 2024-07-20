#!/usr/bin/env python3

import warnings 
warnings.filterwarnings("ignore",category=DeprecationWarning)

import rclpy
import xacro
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import Float32,Int64MultiArray,String
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import math, statistics
from time import time
from collections import Counter
# import pandas

import pandas

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import SetBool
import torch


class MyVehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        self.time_int = 0
        self.declare_parameter('path', rclpy.Parameter.Type.STRING)
        self.data_path = self.get_parameter('path').value
        self.data = pandas.read_csv(self.data_path)
        # self.data = pandas.read_csv("/home/akshit/catvehicle_ros2/src/catvehicle/src/data.csv")
        # self.data.rename(columns ={'Unnamed: 0': 'id'}, inplace = True)

        



        self.state_vector_map = dict() #storing velocity mapped to id
        self.cmd_dict = dict() #storing topic names mapped to id
        # self.index_counter = dict() #storing index counter mapped to id
        self.msg = Twist()

        self.preprocessing()

        self.callback_grp = MutuallyExclusiveCallbackGroup()
        self.callback_group_pub = ReentrantCallbackGroup()

        for i in self.data.id.unique():
            pub_topic = "/vehicle"+str(i)+"/cmd_vel"
            self.cmd_dict[i] = self.create_publisher(Twist,pub_topic,10,callback_group=self.callback_group_pub)
            

        self.status_sub = self.create_subscription(Int64MultiArray,'/status',self.status_callback,10)
        
        self.rel_client = self.create_client(SetBool,'/init')
        while not self.rel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for initiation service")

        req = SetBool.Request()
        req.data = True
        self.rel_client.call_async(req)
    
    def preprocessing(self):

        for i in self.data.id.unique():
            
            vec_list = list()
            data_map = self.data[self.data['id']==i]

            for iter,row in data_map.iterrows():
                vec = [row.velocity,row.ang_vel,row.timestamp_ms]
                vec_list.append(vec)
            # state_tensor = torch.tensor(vec_list).to("cuda")
            self.state_vector_map[i] = vec_list



    
    def status_callback(self,msg:Int64MultiArray):
        l = list(msg.data)
        self.control(l[:-1])
        self.time_int = l[len(l)-1]

    def control(self,spawned_vehicle):

        for i in spawned_vehicle:
            idx = int((self.time_int*100 - self.state_vector_map[i][0][2])/100)
            if idx<0:
                continue
            else:
                # print(self.state_vector_map[i][idx][3])
                try:
                    self.msg.linear.x = self.state_vector_map[i][idx][0]
                    self.msg.angular.z = self.state_vector_map[i][idx][1]
                    self.cmd_dict[i].publish(self.msg)
                except Exception as e:
                    # print(i," for ",idx)
                    print(e.__class__.__name__," for ",i)

        self.get_logger().info(str(self.time_int))

        
        
        



def main(args=None):
    rclpy.init(args=args)

    speed_publisher = MyVehicleController()
    

    executor = MultiThreadedExecutor()
    executor.add_node(speed_publisher)
    executor.spin()
    # rclpy.spin(speed_publisher)
    speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

