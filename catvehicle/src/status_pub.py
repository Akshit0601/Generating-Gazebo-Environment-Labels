#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from std_srvs.srv import SetBool
from math import cos,sin
import torch
import pandas
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from rclpy.qos import HistoryPolicy,QoSProfile


class spawn_despawn(Node):

    def __init__(self):
        super().__init__("spawner")
        self.data = pandas.read_csv("/home/akshit/catvehicle_ros2/src/catvehicle/src/data.csv")
        # self.data.rename(columns ={'Unnamed: 0': 'id'}, inplace = True)
        self.vector_map = dict()

        self.result_map = dict()
        max_timestamp = max(self.data.timestamp_ms)

        self.exit_threshold = len(self.data.id.unique())
        self.exit_counter = 0

        self.individual_idx_counter = dict()  #index counter for each vehicle, since they can be spawned at different timestamps

        self.qos_profile = QoSProfile(
            history = HistoryPolicy.KEEP_ALL,
            depth = 1
        )
        
        #most time-consuming, will be performed once initially
        self.preprocessing()

        #paramaters
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0

        #lidar parameters

        x_threshold = 15
        y_threshold = 15

        #time_stamp --  have interval of 100ms starting from 0

        self.time_int = 0
        self.time_threshold = 0

        self.c_grp = ReentrantCallbackGroup()
        self.c_grp2 = MutuallyExclusiveCallbackGroup()



        self.greater_threshold = torch.tensor([x_threshold,y_threshold,self.time_threshold]).to('cuda') ##should be less than this
        self.lesser_threshold = torch.tensor([0,-1*y_threshold,0]).to("cuda") ##should be greater than this
    

        #pub-suba
        self.odom_listener = self.create_subscription(Odometry,"/catvehicle/odom",self.odom_callback,1,callback_group=self.c_grp)
        self.status_publisher = self.create_publisher(Int64MultiArray,"/status",10,callback_group=self.c_grp)
        self.status_timer = self.create_timer(0.1,self.process,callback_group=self.c_grp2)
        self.msg = Int64MultiArray()

        self.server = self.create_service(SetBool,'/init',self.server_callback)



        #linear and angular transform matrix 

        self.linear_transform = torch.zeros([3]).to("cuda")
        self.angular_transform = torch.zeros([3,3]).to("cuda")
        self.angular_transform[2][2] = 1.0
        self.linear_transform[2] = 0.0

        self.control_loop_init = False

        self.list_updated = False
        



        


        # self.vector_map = dict()

    def preprocessing(self):


        '''
        Uncomment to compute angular and velocity in vehicle frame from dataset
        '''
        # vel_list = list()
        # ang_list = list()
        # for i in self.data.id.unique():
        #     temp_data = self.data[self.data['id']==i].reset_index()
        #     for iter,row in temp_data.iterrows():
        #         velocity = (row.vx**2+row.vy**2)**0.5
        #         vel_list.append(velocity)
        #         # print(iter-1)
        #         if iter==0:
        #             ang_list.append(0)
        #         else:
        #             # print(iter-1)
        #             w = (row.psi_rad - temp_data.iloc[iter-1].psi_rad)/0.1
        #             ang_list.append(w)
        # ang = pandas.Series(ang_list)
        # vel = pandas.Series(vel_list)
        # self.data['velocity'] = vel
        # self.data['ang_vel'] = ang

        '''
        this is done to normalise the coordinates of the traffic vehicles
        '''
        # for iter,row in self.data.iterrows():
        #     self.data.loc[iter,'x'] = row.x - 995.5 
        #     self.data.loc[iter,'y'] = row.y - 998
        # self.data.to_csv("data.csv")
        
        self.pub_list = np.zeros(max(self.data.id)+1)

        '''
        Creating and mapping tensors over device(can be cpu or cuda) of each traffic vehicle id
        '''
        for i in self.data.id.unique():
            init_time = torch.tensor([0])
            self.individual_idx_counter[i] = init_time.to("cuda")


            data_map = self.data[self.data['id']==i]
            vec_list = list()
            for iter,row in data_map.iterrows():
                vec = [row.x,row.y,row.timestamp_ms]
                vec_list.append(vec)
            vector_tensor = torch.tensor(vec_list).to("cuda")
            # name = "pt/"+str(i)+".pt"
            # torch.save(vector_tensor,name)
            self.vector_map[i] = vector_tensor
            self.result_map[i] = vector_tensor


        self.get_logger().info("Preprocessing done")
    
    '''
    normal odom callback updating odometry of the ego vehicle
    '''
    def odom_callback(self,odom_data:Odometry):
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y
        x = odom_data.pose.pose.orientation.x
        y = odom_data.pose.pose.orientation.y
        z = odom_data.pose.pose.orientation.z
        w = odom_data.pose.pose.orientation.w

        self.phi = euler_from_quaternion([x,y,z,w])[2]
        self.get_logger().info(f"inside callback {self.x}")

        # self.get_logger().info(str(self.phi))

    def server_callback(self,request:SetBool.Request,response:SetBool.Response):
        self.control_loop_init = request.data
        response.success = True
        response.message = "control loop initiated"
        return response
    
        
    def process(self):

        # linear_transform = torch.tensor([self.x,self.y])
        # angular_tranform = torch.tensor([[cos(self.phi),-1*sin(self.phi)],
        #                                     [sin(self.phi), cos(self.phi)]])

        #time thresholds
        if self.control_loop_init: 
            self.list_updated = False
            self.time_threshold = self.time_int*100
            self.greater_threshold[2] = self.time_threshold
            '''
            check whether ego vehicle has been updated or not, if yes then the transformation matrices are 
            updated and coordinates are transformed over cuda/cpu of each vehicle.
            Done to avoid computing transformation over each iteration even if there is no change in odometry
            '''
            # linear_truth = self.linear_transform[0] != self.x or self.linear_transform[1] != self.y
            # angular_truth = self.angular_transform[0][0] != cos(self.phi) or self.angular_transform[0][1] != -1*sin(self.phi) or self.angular_transform[1][0] != sin(self.phi) or self.angular_transform[1][1] != cos(self.phi) 
            
            linear_truth = round((self.linear_transform[0].item() - self.x),2) != 0.0 or round((self.linear_transform[1].item() - self.y),2) != 0 
            angular_truth = round((self.angular_transform[0][0].item() - cos(self.phi)),2) != 0.0 or round((self.angular_transform[0][1].item() - (-1)*sin(self.phi)),2) != 0.0 or round((self.angular_transform[1][0].item() - sin(self.phi)),2) != 0.0 or round((self.angular_transform[1][1].item() - cos(self.phi)),2) != 0.0
            keys_to_be_deleted = list()

            
            self.get_logger().info(f"{self.x}")

            if (linear_truth):
                self.linear_transform[0] = self.x
                self.linear_transform[1] = self.y
            if (angular_truth):
                self.angular_transform[0][0] = cos(self.phi)
                self.angular_transform[0][1] = -1*sin(self.phi)
                self.angular_transform[1][0] = sin(self.phi)
                self.angular_transform[1][1] = cos(self.phi)

            if (linear_truth or angular_truth):
                # self.get_logger().info("vector state map updated")

                for i in self.vector_map:
                    idx = self.individual_idx_counter[i].item()

                    ## transformation of vector map
                    self.result_map[i] = self.vector_map[i] - self.linear_transform
                    self.result_map[i] = torch.bmm(self.vector_map[i].unsqueeze(0),self.angular_transform.unsqueeze(0)).squeeze() 
                    ## check if the traffic vehicle is within fov and has been spawned yet, based on current timestamp shown by self.timeint
                    
                    try:
                        truth_tensor = self.result_map[i][idx].less_equal(self.greater_threshold)
                        greater_truth = False if False in truth_tensor else True
                        less_truth = False if False in self.result_map[i][idx].greater_equal(self.lesser_threshold) else True
                        
                        
                        # if truth_tensor[2] == True:
                        #     print(self.time_threshold,' for ',i,' with ',self.vector_map[i][idx][2])
                        #     self.individual_idx_counter[i]+=1
                        # if int(greater_truth and less_truth):
                        #     print('idx for ',i," is ",idx)
                        #     print(self.vector_map[i][idx])
                        #     print(self.time_int)
                        if self.result_map[i][idx][2].equal(self.greater_threshold[2]):
                            self.individual_idx_counter[i]+=1   #only increment in index if they have been spawned

                        self.pub_list[i] = int(greater_truth and less_truth)
                     

                    except IndexError:
                        # self.get_logger().info(str(e.__class__.__name__))
                        self.pub_list[i] = 0
                        keys_to_be_deleted.append(i)
                        self.exit_counter+=1
                
                self.list_updated = True
                        
                        # self.get_logger().info("exception raised for:"+str(i))  
                        
                    # self.get_logger().info("first block was executed")                           
            
            # non updated odometry case
            if not self.list_updated:
                for i in self.vector_map:
                    idx = self.individual_idx_counter[i].item()
                    try:
                        truth_tensor = self.result_map[i][idx].less_equal(self.greater_threshold)
                        greater_truth = False if False in truth_tensor else True
                        less_truth = False if False in self.result_map[i][idx].greater_equal(self.lesser_threshold) else True
                        
                        # if truth_tensor[2] == True:
                        #     self.individual_idx_counter[i]+=1
                        # if int(greater_truth and less_truth):
                        #     print()
                        # if int(greater_truth and less_truth):
                        #     print('idx for ',i," is ",idx)
                        #     print(self.vector_map[i][idx])
                        #     print(self.time_int)
                        if self.result_map[i][idx][2].equal(self.greater_threshold[2]):
                            self.individual_idx_counter[i]+=1
                            
                        self.pub_list[i] = int(greater_truth and less_truth)
                        

                    except IndexError:
                        self.pub_list[i] = 0
                        # self.get_logger().info(str(e.__class__.__name__))

                        keys_to_be_deleted.append(i)
                        self.exit_counter+=1
            
                        # self.get_logger().info("exception raised for:"+str(i))  
                        
                # self.get_logger().info("second was executed")

                    # print(self.pub_list[i])

            #delete entries whose data is not available after this iteration
            
            for key in keys_to_be_deleted:
                del self.vector_map[key] 
                del self.result_map[key]
            #when all entries are deleted
            if self.exit_counter==self.exit_threshold:   #kills the node when no more vehicle are there to analyse
                self.get_logger().info("killing node now")
                self.get_logger().info("length: "+str(len(self.vector_map)))
                self.get_logger().info(str(self.time_int))
                self.destroy_node()
                rclpy.shutdown() 

            d = np.nonzero(self.pub_list)[0].tolist()
            d.append(self.time_int) ##like a checksum
            self.msg.data = d
            
            self.status_publisher.publish(self.msg)
            # self.get_logger().info("publishing status list")
            # self.get_logger().info(str(self.exit_counter))

            self.time_int+=1

def main(args = None):
    rclpy.init(args=args)

    
    status_node = spawn_despawn()

    executor = MultiThreadedExecutor()
    executor.add_node(status_node)

    executor.spin()
    # executor.shutdown()
    # rclpy.spin(status_node)

    status_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    







              





        


            
            


        


