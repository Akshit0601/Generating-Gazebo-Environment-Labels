import rclpy
import warnings
warnings.filterwarnings("ignore",category=DeprecationWarning)
import pandas


from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import time
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt 
from math import sin,cos,atan
import torch
from scipy.spatial.distance import cdist
from time import time
from gazebo_msgs.msg import ModelStates

class clust(Node):

    def __init__(self):
        super().__init__('lidar_clusterer')
        # self.lidar_data = np.ones(180)

        self.lenth_threshold = 0.5
        self.m = 0
        self.states_data = pandas.read_csv("data.csv")


        self.create_subscription(LaserScan,"/catvehicle/scan",self.lidar_processing,10)

        self.create_subscription(ModelStates,"/gazebo/model_states",self.tranform_update,10)

        self.create_subscription(Float64MultiArray,"/status",self.index_updater,10)


        self.arr_curr = np.array([])
        self.arr_prev = np.array([])
        self.colors = ["yellow","red",'blue','orange']
        self.vel_states = dict()
    
    def preprocessing(self):
        
        for iter,row in self.states_data.iterrows():
            self.vel_states[int(row.id)] = np.array([row.velocity, row.psi_rad])
            
        
        
        pass


    def lidar_processing(self,data:LaserScan):
        self.lidar_data = np.array(data.ranges)
        
        tensor = torch.tensor(self.lidar_data).to('cuda')
        l = self.batch_slice_tensor(tensor)
        cluster = list()
        self.vel_list = np.zeros(180)
        self.ang_list = np.zeros(180)
        index_dict = dict()


        m_list = list()
        print(len(l))

        for i in l: 
            # print(l)
            temp_cluster = list()
            tensor_set = list()
            for j in i:
                thetha = np.argwhere(self.lidar_data == j.item())[0][0]*0.017 - 1.57
                # z = complex(i*cos((lidar.index(i))*0.017),i*sin(lidar.index(i)*0.017))
                point_tensor = [float(j.item())*cos(thetha),float(j.item())*sin(thetha)]
                index_dict[point_tensor[0]] = j.item()
                tensor_set.append(point_tensor)
        
            tensor_set_ = torch.tensor(tensor_set).to('cuda')
            for k in range(1,len(tensor_set)):
                length = torch.norm(tensor_set_[k]-tensor_set_[k-1])
                # print(length)
                if abs(length.item())>self.lenth_threshold:
                    print("no.of points inside clusters: ",len(temp_cluster))
                    if len(temp_cluster)>=2:
                        cluster.append(temp_cluster)
                    # print('appending')
                    temp_cluster = list()
                    # print("thresh")
                
                temp_cluster.append(tensor_set[k])
            if len(temp_cluster)>=2:
                print("no.of points inside clusters: ",len(temp_cluster))
                cluster.append(temp_cluster)
        
        for i in cluster:
            if i:
                m_list.append(np.mean(i,axis=0).tolist())

        self.arr_curr = np.array(m_list)
        print("no.of clusters: ",len(cluster))
        # self.plot_cluster(cluster=cluster)

        # print(self.arr_curr)
        try:
            cluster.remove([])
        except Exception as e:
            # print(e)
            pass
        '''
        velocity calculation
        '''
        if self.arr_prev.any() and self.arr_curr.any():
            
            dist = cdist(self.arr_curr,self.arr_prev)

            for i in range(self.arr_curr.shape[0]):
                id = np.argmin(dist[i]) #id refers to previous cluster id
                # print(id)
                velocity = dist[i][id]/0.03
                a1 = torch.tensor(np.vstack((self.arr_curr[i],self.arr_prev[id]))).to('cuda')
                # print(a1)
                diff = torch.diff(a1,dim=0)

                if velocity:
                    angle = atan(diff[0][1].item()/(diff[0][0].item()))
                else:
                    angle = 0
                for k in cluster[i]:
                    idx = np.where(self.lidar_data == index_dict[k[0]])[0].item()
                    # print(index_dict[k[0]])
                    self.vel_list[idx] = velocity
                    self.ang_list[idx] = angle
        # except Exception as e:
        #     print(e)
            
        final = np.vstack((self.lidar_data,self.vel_list,self.ang_list))
        self.arr_prev = self.arr_curr
        np.save('/home/akshit/colcon_ws/src/catvehicle/src/results/f'+str(self.m)+".npy",final)
        self.m+=1
                    
    def batch_slice_tensor(self,tensor):
        result = []
        current_subarray = []

        for element in tensor:
            if element < 15:
                current_subarray.append(element)
            else:
                if current_subarray:
                    result.append(torch.tensor(current_subarray).to('cuda'))
                    current_subarray = []
        if current_subarray:
            result.append(torch.tensor(current_subarray).to('cuda'))

        return result
    
    def plot_cluster(self,cluster):
        cnt = 0
        tot = 0
        for i in cluster:
            for j in i:
                fig,ax = plt.subplots()
                ax.plot(j[0],j[1],color = self.colors[cnt],marker = ".")
                fig.savefig("/home/akshit/colcon_ws/src/catvehicle/src/results/"+str(tot)+".png")
                tot+=1
                
            cnt+=1
            if cnt==4:
                cnt=0
                # print(j)
        # plt.close()

        

def main(args = None):
    rclpy.init(args=args)

    clusterer = clust()
    rclpy.spin(clusterer)
    clusterer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()