#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit

from tf_transformations import quaternion_from_euler
# from math import sin,cos

from launch_ros.actions import Node

import pandas

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )
    # rear_control_config = "/home/akshit/catvehicle_ros2/src/catvehicle/config/rear_control.yaml"
    # front_control_config = "/home/akshit/catvehicle_ros2/src/catvehicle/config/front_control.yaml"

    ld =  LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument(
          'world',
          default_value=['/home/akshit/catvehicle_ros2/src/catvehicle/world/label.world'],
          description='SDF world file'),

        # DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        gazebo,
   
    ])
    

    # Path to the Xacro file
    xacro_path = '/home/akshit/catvehicle_ros2/src/catvehicle/urdf/catvehicle.urdf.xacro'
        # position_x = 0.0
        # position_y = 0.0
    # data = pandas.read_csv('/home/akshit/generate_labels/src/generate_labels/data/vehicle_tracks_000_mod_1.csv')




     
    robot_description = xacro.process_file(
    xacro_path,
    mappings={
    "roboname": "catvehicle",                                                                         
    })

    
    params = {'robot_description': robot_description.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_1 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        ) 
    spawn_entity_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', "catvehicle",
            '-z', "0.0",
            '-x', '0.0',
            '-y', '0.0',
            '-Y', '0.0',
        ]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
    )   
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )   

    # node_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0.0', '0.0','0.0', str(xu), str(yu), str(zu), str(wu), 
    #         'world', robot_namespace_1 + '/odom'],

    # )
    event_handler1  =RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_1,
                on_exit=[load_joint_state_broadcaster],
            )
        )
    event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_position_controller],
            )
        )
    event_handler2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_position_controller,
                on_exit=[load_joint_velocity_controller],
            )
        ) 
    ld.add_action(event_handler1)
    ld.add_action(event_handler)
    ld.add_action(event_handler2)
    ld.add_action(robot_state_publisher_1)
    ld.add_action(spawn_entity_1)
    # ld.add_action(joint_state_publisher_node)
    

    return ld