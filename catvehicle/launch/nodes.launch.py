#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration       
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_path = os.path.expanduser('~')
    default_path = os.path.join(base_path,"data.csv")
    data_path = LaunchConfiguration('path', default=default_path )
    package_path = get_package_share_directory('generate_labels')
    xacro_path = os.path.join(package_path,'urdf','car_4w2.urdf.xacro')
    # xacro_path = "/home/akshit/generate_labels/src/generate_labels/urdf/car_4w2.urdf.xacro"
    status_pub = Node(
        name="status_pub",
        package="catvehicle",
        executable="status_pub.py",
        output="screen",
        parameters = [
            {'path' : data_path},
        ]
    )

    spawner = Node(
        name="spawner_despawner",
        package="catvehicle",
        executable="spawner.py",
        parameters = [
            {'path' : data_path},
            {'xacro_path' : xacro_path}
        ],
        output="screen"
    )

    odom_vel = Node(
        name="cmdvel2Cat",
        package="catvehicle",
        executable="cmd2gazebo.py",
        parameters = [
            {'path' : data_path},
        ],
        output="screen" 
    )
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("path", default_value = default_path))

    
    ld.add_action(odom_vel)
    ld.add_action(status_pub)
    ld.add_action(spawner)

    return ld