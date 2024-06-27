#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from tf_transformations import quaternion_from_euler
# from math import sin,cos
from launch_ros.actions import Node
from random import randint

import pandas

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_am_path = get_package_share_directory('generate_labels')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )
    rviz_config_file = '/home/akshit/generate_labels/src/generate_labels/rviz/default.rviz'
    rviz_node = Node(package    ='rviz2',
                     executable ='rviz2',
                     name       ='rviz2',
                     output     ='log',
                     arguments  =['-d', rviz_config_file]) 

    ld =  LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),

        # DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        gazebo,
        # rviz_node
   
    ])
    

    # Path to the Xacro file
    xacro_path = '/home/akshit/generate_labels/src/generate_labels/urdf/backup.urdf.xacro'
        # position_x = 0.0
        # position_y = 0.0
    data = pandas.read_csv('/home/akshit/generate_labels/src/generate_labels/data/vehicle_tracks_000_mod_1.csv')


    track_id = len(data.track_id.unique())
    for i in data.track_id.unique():
        robot_namespace_1 = "vehicle"+str(i)
        
        # x = str(1+radius*cos(yaw)) ##test for controls 
        # y = str(1+radius*sin(yaw))
        # z = str(yaw)

        df_extract = data[data["track_id"]==i].reset_index()
        x = str(df_extract.x_trans[0])
        y = str(df_extract.y_trans[0])
        z = str(df_extract.psi_rad[0]) 


     
        robot_description = xacro.process_file(
        xacro_path,
        mappings={
        "robot_namespace": robot_namespace_1,
        # "collide_bitmask" : hex(randint(0,255))
                                                                                 
        })

        
        params = {'robot_description': robot_description.toxml(), 'use_sim_time': use_sim_time}
        robot_state_publisher_1 = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_namespace_1,
                output='screen',
                parameters=[params]
            ) 

        xu,yu,zu,wu = quaternion_from_euler(0.0,0.0,df_extract.psi_rad[0])
        spawn_entity_1 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', robot_namespace_1 + "/robot_description",
                '-entity', PythonExpression(['"', robot_namespace_1, '_robot"']), #default enitity name _bcr_bot
                '-z', "0.0",
                '-x', x,
                '-y', y,
                '-Y', z,
            ]
        )
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_namespace_1,
            parameters=[params],
        )        

        node_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0','0.0', str(xu), str(yu), str(zu), str(wu), 
                'world', robot_namespace_1 + '/odom'],

        )
        ld.add_action(robot_state_publisher_1)
        ld.add_action(spawn_entity_1)
        # ld.add_action(joint_state_publisher_node)
        # ld.add_action(node_tf)

        # yaw+=1.57
        # if i%4 == 0:
        #     radius+=5





    # ld.add_action(spawn_entity_1)
    return ld