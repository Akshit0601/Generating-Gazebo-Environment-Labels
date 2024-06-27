import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import Float32,Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import math, statistics
from time import time
from collections import Counter
# import pandas
import pandas
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from gazebo_msgs.srv import DeleteEntity
from std_srvs.srv import SetBool

# Define a class for your ROS2 node
class MyVehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        self.callback_group_pub = ReentrantCallbackGroup()
        # self.callback_group_1 = MutuallyExclusiveCallbackGroup()
        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        # Create a publisher for sending velocity commands to the robot
        # self.cmd_1 = self.create_publisher(Twist,'/vehicle1/cmd_vel',10)
        # self.cmd_2 = self.create_publisher(Twist,"/vehicle2/cmd_vel",10)
        # self.cmd_3 = self.create_publisher(Twist,"/vehicle3/cmd_vel",10)
        # self.cmd_4 = self.create_publisher(Twist,"/vehicle4/cmd_vel",10)


        # self.in_x = 2.0
        # self.in_ang = 1.0
        # self.msg_1 = Twist()
        # self.msg_2 = Twist()
        # self.msg_3 = Twist()
        self.cnt = 0

        self.cmd_dict = dict()
        self.msg = Twist()

        self.callback_grp = MutuallyExclusiveCallbackGroup()
        # flags and parameters here
        
        #timer object
        # self.controller_timer = self.create_timer(0.1, self.controller_loop,callback_group=self.callback_group)
        self.control_timer = self.create_timer(0.1,self.control_loop,self.default_callback_group)
        self.data = pandas.read_csv('/home/akshit/generate_labels/src/generate_labels/data/vehicle_tracks_000_mod_1.csv')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
            depth=15
        )
        self.destroyer = self.create_client(DeleteEntity,"/delete_entity")
        while not self.destroyer.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("waiting for deletion service")

        self.get_logger().info("connected to deletion service")
        self.delete_req = DeleteEntity.Request()
        self.delete_req.name = str()
        for i in self.data.track_id.unique():
        # for i in [1,2,3] : 
            pub_topic = "/vehicle"+str(i)+"/cmd_vel"
            temp_pub  = self.create_publisher(Twist,pub_topic,10,callback_group=self.callback_group_pub)
            self.cmd_dict[i] = temp_pub
            # try: 
            #     temp_pub.publish(self.msg)
            # except:
            #     pass  #this is expected to pass
            # else:
            #     self.get_logger().info("publisher "+str(i)+"active")
            

            # self.get_logger().info()

        self.df_dict = dict()
        self.unique_keys = self.data.track_id.unique()
        # for i in range[1,2,3]
        for i in self.data.track_id.unique():
            self.df_dict[i] = self.data[self.data['track_id']==i].reset_index()
        
        self.client = self.create_client(SetBool)

        
            


    def control_loop(self):

        for i in self.unique_keys:
        # for i in [7]:
            df_temp = self.df_dict[i]
            try:
                self.msg.linear.x = df_temp.vel_in_vehicle_frame[self.cnt]
                # rad_vel = df_temp.radial_velocity[self.cnt]
                # if rad_vel>=0.1:
                self.msg.angular.z = df_temp.radial_velocity[self.cnt]
                # else:
                    # self.msg.angular.z = 0.0
                # self.msg.angular.z = 0.0
                self.cmd_dict[i].publish(self.msg)  
                self.get_logger().info("vehicle: "+str(i)+" speed_lin: "+str(self.msg.linear.x)+" angular_speed: "+str(self.msg.angular.z))

            except:
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.cmd_dict[i].publish(self.msg)
                self.delete_req.name = "vehicle"+str(i)+"_robot"
                self.destroyer.call_async(self.delete_req)
                
        self.cnt+=1
        # rclpy.sleep()
        
        # if self.cnt%10==0 :
        #     self.in_x = self.in_x*(-1)
        # self.in_ang = self.in_ang*(-1)


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
        








        
        
