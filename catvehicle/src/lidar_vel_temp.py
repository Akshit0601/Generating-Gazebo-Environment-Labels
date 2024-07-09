import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
from math import cos, sin
from copy import deepcopy
from scipy.spatial.distance import cdist
class logger(Node):

    def __init__(self):
        super().__init__("logger")
        self.create_subscription(ModelStates,'/gazebo/model_states',self.log,10)
        self.time_stamp = 0
        self.file_name = '/home/akshit/colcon_ws/src/catvehicle/src/log3.txt'
        self.idx = 0
        self.arr_curr = np.array([])  ##something


    def log(self,data:ModelStates):
        self.filehandle = open(self.file_name, 'a+')
        cat_pose  = data.pose[1].position
        cat_orient = data.pose[1].orientation
        a = cat_pose.x
        b = cat_pose.y
        print(a,b)
        yaw  = euler_from_quaternion([cat_orient.x,cat_orient.y,cat_orient.z,cat_orient.w])[2]
        # print(yaw)
        a = a + 2.466*cos(yaw)
        b = b + 2.466*sin(yaw)
        linear_transform = np.array([a,b])
        angular_transform = np.zeros([2,2])
        angular_transform[0][0] = cos(yaw)
        angular_transform[0][1] = -1*sin(yaw)
        angular_transform[1][0] = sin(yaw)
        angular_transform[1][1] = cos(yaw)

        self.get_logger().info('recieved')
        d = str(data.name[2:])
        k = Pose()
        
        # k.position.x
        arr_present = np.zeros(len(data.name)-2)
        arr_global = np.zeros(1000)
        
        for i in range (2,len(data.pose)):
            a = (np.array([data.pose[i].position.x,data.pose[i].position.y]) - linear_transform) @ angular_transform
            arr_global[int(data.name[i][-1])] = a
            arr_present[i] = deepcopy(a)

        dist = cdist(self.arr_curr,arr_present)
        for i in range(self.arr_curr.shape[0]):
            idx = np.argmin(dist[i]) #id refers to previous cluster id
            id = np.where(arr_global == arr_present[idx])[0].item()
            velocity = self.data[id][t].velocity
            angle = self.data[id][t].angle

            for k in cluster[i]:
                    idx = np.where(self.lidar_data == index_dict[k[0]])[0].item()
                    # print(index_dict[k[0]])
                    self.vel_list[idx] = velocity
                    self.ang_list[idx] = angle

        # lt = data.ranges.tolist()
        self.filehandle.write(str(lt)+'\n\n')
        self.filehandle.close()



def main(args = None):
    rclpy.init(args=args)
    
    log_node = logger()
    # for i in range(2):
        # rclpy.spin_once(log_node)
    rclpy.spin(log_node)
    log_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    




