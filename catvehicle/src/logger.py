import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

class logger(Node):

    def __init__(self):
        super().__init__("logger")
        self.create_subscription(LaserScan,'/catvehicle/scan',self.log,10)
        self.time_stamp = 0
        self.file_name = '/home/akshit/colcon_ws/src/catvehicle/src/log.txt'

    def log(self,data:LaserScan):
        self.filehandle = open(self.file_name, 'a+')

        lt = data.ranges.tolist()
        self.filehandle.write(str(lt)+'\n\n')
        self.filehandle.close()



def main(args = None):
    rclpy.init(args=args)
    
    log_node = logger()
    rclpy.spin(log_node)

    log_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
