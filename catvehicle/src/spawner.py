#!/usr/bin/env python3

import warnings

import rclpy.parameter
warnings.filterwarnings("ignore",category=DeprecationWarning)
import rclpy.node
import xacro
import rclpy
from gazebo_msgs.srv import SpawnEntity,DeleteEntity
import pandas
from std_msgs.msg import Int64MultiArray
from random import randint
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
spawned_veh = list()
time_int = 0
xml_map = dict()
vector_map = dict()

def preprocessing():
    global df,xml_map,vector_map,nh,xacro_path
    for i in df.id.unique():
        
        doc = xacro.parse(open(xacro_path))

        namespace = "vehicle"+str(i)
        xacro.process_doc(doc,{'robot_namespace' : namespace, 'bit1': hex(2**randint(1,15))})
        xml_map[i] = doc.toxml()
        temp_df = df[df['id']==i]
        vector = list()
        for iter,row in temp_df.iterrows():
            vec = [row.x,row.y,row.psi_rad,row.timestamp_ms]
            vector.append(vec)
        vector_map[i] = vector
    nh.get_logger().info("Preprocessing done")

def spawn_cb(msg:Int64MultiArray):
    global time_int,spawned_veh
    l = list(msg.data)
    print(l)
    time_int = l[len(l)-1]

    add_vehicles = set(l[:-1]) - set(spawned_veh)
    removed_vehicles = set(spawned_veh) - set(l[:-1])

    print("add_vehicle: ",add_vehicles)

    print("remove vechicles: ", removed_vehicles)

    for i in add_vehicles:
        # print(i,"spawned")
        spawn_vehicle(i)    
    
    for j in removed_vehicles:
        # print(j,"despawned")
        delete_vehicle(j)
    spawned_veh = l[:-1]
    print("loop done")

def spawn_vehicle(id):
    global df,xml_map,vector_map,nh
    nh.get_logger().info("spawning: "+str(id))

    idx = int((time_int*100 - vector_map[id][0][3])/100) 
    print(idx," for ",id)
    spawn_req = SpawnEntity.Request()
    spawn_req.xml = xml_map[id]

    if idx<0:
        print('returning for',id)
        return
    else:
        print('inside')
        spawn_req.initial_pose.position.x = vector_map[id][idx][0]
        spawn_req.initial_pose.position.y = vector_map[id][idx][1]
        spawn_req.initial_pose.orientation.z = vector_map[id][idx][2]
        spawn_req.name = "vehicle" + str(id) + "_robot"

        future = spawn_client.call_async(spawn_req)
        # while rclpy.ok():
        # while rclpy.ok():
            # if future.done():
            #     print("future spawned")
                # break
        #     rclpy.spin_once() 


def delete_vehicle(id):
    global nh
    delete_req = DeleteEntity.Request()

    delete_req.name = "vehicle"+str(id)+"_robot"
    future = despawn_client.call_async(delete_req)
    if future.done():
        nh.get_logger().info("despawned: ",id)

    
    


rclpy.init()
c_group = ReentrantCallbackGroup()
executor = MultiThreadedExecutor()
nh = rclpy.create_node("spawner_despawner")
nh.declare_parameter('path',rclpy.Parameter.Type.STRING)
nh.declare_parameter('xacro_path',rclpy.Parameter.Type.STRING)
df = pandas.read_csv(nh.get_parameter('path').value)
# df = pandas.read_csv("/home/akshit/catvehicle_ros2/src/catvehicle/src/data.csv")
xacro_path = nh.get_parameter('xacro_path').value
nh.get_logger().info(xacro_path)
preprocessing()

spawn_client = nh.create_client(SpawnEntity,"/spawn_entity",callback_group=c_group) 
while not spawn_client.wait_for_service(timeout_sec=1):
    nh.get_logger().info("waiting for spawn service")
despawn_client = nh.create_client(DeleteEntity,"/delete_entity",callback_group=c_group)
while not despawn_client.wait_for_service(timeout_sec=1):
    nh.get_logger().info("waiting for delete service")

nh.create_subscription(Int64MultiArray,"/status",spawn_cb,10)

executor.add_node(nh)
try:
    # rclpy.spin(nh)
    executor.spin()
except KeyboardInterrupt:
    nh.destroy_node()
    executor.shutdown()
    rclpy.shutdown()