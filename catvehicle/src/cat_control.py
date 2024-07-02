#!/usr/bin/env python3
# import rospy
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
# from tf. import euler_from_quaternion
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Float64
import math
import numpy as np
import time
# from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
# from concurrent.futures import ThreadPoolExecutor

#global pose
#global steer_angle_feedback
#global ranges
#global angles

#pose = [0,0,0]
#steer_angle_feedback = 0
#ranges = []
#angles = []


class CarVehicle(Node):

    def __init__(self,robot_name,goal_x,goal_y):
        self.name = robot_name
        self.velocity = Twist()
        self.odometry = Odometry()
        self.scan = LaserScan()
        self.pose = [0,0,0]
        self.ranges = []
        self.angles = []
        self.steer_angle = 0
        self.L = 2.72 #2.62 initially
        self.T = 1.29
        self.goal_x = goal_x
        self.goal_y = goal_y

        super().__init__('catvehicle_controller')

        self.callback_grp = ReentrantCallbackGroup()
        self.transformedPose = np.array([self.pose[0] + self.L * math.cos(self.pose[2]),
                                    self.pose[1] + self.L * math.sin(self.pose[2]),
                                    self.angle_normalizer(self.pose[2] + self.steer_angle),
                                    self.angle_normalizer(self.pose[2])])

        # rospy.Subscriber(f'/{self.name}/odom',Odometry,self.RobotPose)
        # self.create_subscription('/catvehicle/odom',Odometry,self.RobotPose,10,callback_group=self.callback_grp)
        self.create_subscription(Odometry,'/catvehicle/odom',self.RobotPose,10)

        #rospy.Subscriber(f'/{self.name}/steer_angle_commanded',Float64,self.SteerAngleFeedback)
        # self.velocity_publisher = rospy.Publisher(f'/{self.name}/cmd_vel_safe', Twist, queue_size=10)
        self.velocity_publisher = self.create_publisher(Twist,"/cmd_vel",10)

        self.control_loop = self.create_timer(100,self.controller)


        # rospy.Subscriber(f'/{self.name}/front_laser_points', LaserScan, self.lidar_callback, queue_size=10)
        self.create_subscription(LaserScan,"/catvehicle/scan", self.lidar_callback, 10)

        #velocity_msg = AckermannDriveStamped()
        self.velocity_msg = Twist()
        # self.rate = rospy.Rate(100)


## quaternion to euler
    def quat2euler(self,x,y,z,w):
        quat = [x,y,z,w]
        return euler_from_quaternion(quat)
########################

    def euclidean_distance(self,x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def RobotPose(self,data):
        #global pose
        self.pose = [data.pose.pose.position.x,data.pose.pose.position.y,euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]]
        self.transformedPose = np.array([self.pose[0] + self.L * math.cos(self.pose[2]),
                                    self.pose[1] + self.L * math.sin(self.pose[2]),
                                    self.angle_normalizer(self.pose[2] + self.steer_angle),
                                    self.angle_normalizer(self.pose[2])])
    #def SteerAngleFeedback(self,data):
        #global steer_angle_feedback

        #self.steer_angle_feedback = data.data
    #print(steer_angle_feedback)
    #print("\n")

    def angle_normalizer(self,angle):
        if angle < -math.pi:
            out = angle + 2 * math.pi
        elif angle > math.pi:
            out = angle - 2 * math.pi
        else:
            out = angle
        return out

    def lidar_callback(self,msg):
        #global ranges, angles
        ranges_raw = msg.ranges
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.ranges = np.zeros(len(ranges_raw))
        #self.min_angle = msg.angle_min
        #self.max_angle = msg.angle_max
        #if len(self.ranges) >135:
        #self.ranges = [self.ranges[i] for i in range(len(self.ranges)) if i % 8 == 0]
        #if len(self.angles) > 135:    
        #self.angles = [self.angles[i] for i in range(len(self.angles)) if i % 8 == 0]
        for i in range(len(ranges_raw)):
            #print(len(self.ranges))
            if ranges_raw[i]  > 15:
                self.ranges[i] = 15
            else:
                self.ranges[i] = max(0,ranges_raw[i]-self.T/2)
                #print("range enc")
    
    def R_max_finder(self):
        #global ranges
        #global angles

        delta = int(-(len(self.ranges)/np.pi)*self.steer_angle)
        print(delta) # Measure of steering angle (interms of angle indices) relative to the robot's orientation
        indices_limited_fov = [] #Angle indices of interest considered for icecone computations

        for i in range(int((len(self.ranges)/4)+delta/2),int(len(self.ranges)-(len(self.ranges)/4)+delta/2)):
            indices_limited_fov.append(i)

        mid_element = (len(self.ranges)//2) + delta
        #print(mid_element)

        d = []
        d_min = np.zeros(len(self.ranges))
        d_min[mid_element] = 0.6*self.ranges[mid_element]
        #print(2*(index_of_interest-mid_element))

        for j in indices_limited_fov:
            index_of_interest = j
            d = []
            for i in range(min(mid_element,mid_element+2*(index_of_interest-mid_element)),max(mid_element,mid_element+2*(index_of_interest-mid_element))):
                try:
                    d.append(self.ranges[i]/(abs(math.cos(self.angle_normalizer(self.angles[index_of_interest]-self.angles[i])))+math.sqrt(math.sin(self.angle_normalizer(self.angles[mid_element]-self.angles[index_of_interest]))**2 - math.sin(self.angle_normalizer(self.angles[index_of_interest]-self.angles[i]))**2)))
                except Exception as e:
                    pass
            # rospy.loginfo(d)
            self.get_logger().info(str(d))
            try:
                d_min[j] = 0.6*min(d)
            except Exception as e:
                pass

        return d_min, indices_limited_fov

    def way_point_finder(self, d_min, indices_limited_fov): #Considering a straight line road

        max_x = 0

        # Computed based on horizontal road
        for i in indices_limited_fov:

            x_lim = self.transformedPose[0] + d_min[i]*math.cos(self.angle_normalizer(self.angles[i]+self.transformedPose[3]))
            y_lim = self.transformedPose[1] + d_min[i]*math.sin(self.angle_normalizer(self.angles[i]+self.transformedPose[3]))

            if(x_lim > max_x):
                way_x_opt = x_lim
                way_y_opt = y_lim
                max_x = x_lim
        #print(way_x_opt)
        #print(way_y_opt)
        return way_x_opt, way_y_opt

    def find_closest_element_with_index(self,array, target):
        # Enumerate the array to get both index and element
        closest_index, closest_element = min(enumerate(array), key=lambda x: abs(x[1] - target))
        return closest_index, closest_element

    def way_point_finder_distance_based(self,d_min, indices_limited_fov): #for reaching a target point

        min_dis = 1000

        # Computed based on horizontal road
        for i in indices_limited_fov:

            x_lim = self.transformedPose[0] + d_min[i]*math.cos((self.angles[i]+self.transformedPose[3]))
            y_lim = self.transformedPose[1] + d_min[i]*math.sin((self.angles[i]+self.transformedPose[3]))

            
            distance = self.euclidean_distance(x_lim,y_lim,self.goal_x,self.goal_y)

            rel_angle_to_goal = self.angle_normalizer(self.transformedPose[3] - np.arctan2(self.transformedPose[1] - self.goal_y, self.transformedPose[0] - self.goal_x))
            distance_to_goal = self.euclidean_distance(self.transformedPose[0],self.transformedPose[1],self.goal_x,self.goal_y)

            closest_index, closest_element = self.find_closest_element_with_index(self.angles,rel_angle_to_goal)

            if(d_min[closest_index]>distance_to_goal):
                way_x_opt = self.goal_x
                way_y_opt = self.goal_y
                break

            if(distance<min_dis):
                way_x_opt = x_lim
                way_y_opt = y_lim
                min_dis = distance

        # print(way_x_opt)
        # print(way_y_opt)
        return way_x_opt, way_y_opt
        


    ## Main Node
    def controller(self):
            
        print('Entered control computer')    
        while(len(self.pose)==0):
            print('First pose not obtained yet')
            continue

        while(len(self.ranges) == 0):
            continue

        self.goal_x = self.goal_x
        self.goal_y = self.goal_y

        #while not rospy.is_shutdown():
            
        start_time = time.time()


        d_min, indices_limited_fov = self.R_max_finder()
        #way_x_opt, way_y_opt = self.way_point_finder(d_min,indices_limited_fov)

        way_x_opt, way_y_opt = self.way_point_finder_distance_based(d_min,indices_limited_fov)

        rel_bearing = self.angle_normalizer(self.transformedPose[2] - np.arctan2(self.transformedPose[1] - way_y_opt, self.transformedPose[0] - way_x_opt))

        if rel_bearing >= np.pi/2:
            sigma = rel_bearing - np.pi
        elif rel_bearing < -np.pi/2:
            sigma = rel_bearing + np.pi
        else:
            sigma = rel_bearing

        R = self.euclidean_distance(way_x_opt , way_y_opt, self.transformedPose[0], self.transformedPose[1])   
        K_1 = 1
        K_2 = 2.5
        
        w_1 = -K_1 * np.tanh(R) * np.sign(np.cos(rel_bearing))

        if R<0.05:
            w_2 = -K_2 * np.sign(sigma) * np.sqrt(np.abs(sigma))+ (-K_1 * np.sign(np.cos(rel_bearing))) * np.sin(rel_bearing)
        else:
            w_2 = -K_2 * np.sign(sigma) * np.sqrt(np.abs(sigma))+ (w_1/R)*np.sin(rel_bearing)

        V = np.linalg.solve(np.array([[1, 0],
                                    [np.sin(self.angle_normalizer(self.transformedPose[2] -
                                                                self.transformedPose[3]))/self.L, 1]]),
                            np.array([w_1, w_2]))

        end_time = time.time()
        elapsed_time = end_time - start_time
        self.steer_angle = (self.steer_angle + V[1]*elapsed_time)

        if(self.steer_angle>1.2):
            self.steer_angle = 1.2
        
        if(self.steer_angle<-1.2):
            self.steer_angle = -1.2

        velocity = V[0]/math.cos(self.steer_angle)

        self.velocity_msg.linear.x = velocity
        self.velocity_msg.angular.z = self.steer_angle
        self.velocity_publisher.publish(self.velocity_msg)
        self.get_logger().info("publshing now")

        #rate.sleep()
    ########################


# def control_car(car, param1, param2):
#     rate = rospy.Rate(100)

#     while not rospy.is_shutdown():
#         car.controller(param1, param2)
#         rate.sleep()

if __name__ == "__main__":
    # rospy.init_node("parallel_controller_node")
    rclpy.init()


    # Instantiate your Car objects
    Car_1 = CarVehicle('catvehicle',18,7)

    executor = MultiThreadedExecutor()
    executor.add_node(Car_1)

    try:
        executor.spin()
    except KeyboardInterrupt:
        executor.shutdown()
        # rclpy.shutdown()
    # Car_2 = CarVehicle('catvehicle_obs')
    # Car_3 = CarVehicle('catvehicle_obss')
    #Car_4 = CarVehicle('catvehicle_perp')

    # Create a ThreadPoolExecutor with 2 threads for concurrent execution
    # with ThreadPoolExecutor(max_workers=16) as executor:
    #     # Submit the control functions for execution
    #     future_car_1 = executor.submit(control_car, Car_1, 18, 7)
    #     future_car_2 = executor.submit(control_car, Car_2, -10, -6) 
    #     future_car_3 = executor.submit(control_car, Car_3, -4.5, -3)
        #future_car_4 = executor.submit(control_car, Car_4, 18, 20)

        # Block until both futures are done (not necessary in a ROS node, as the node keeps running)
        #future_car_1.result()
        #future_car_2.result()
        #future_car_3.result()
        #future_car_4.result()


