#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    status_pub = Node(
        name="status_pub",
        package="catvehicle",
        executable="status_pub.py",
        output="screen",
    )

    spawner = Node(
        name="spawner_despawner",
        package="catvehicle",
        executable="spawner.py",
        # output="screen",
    )

    odom_vel = Node(
        name="cmdvel2Cat",
        package="catvehicle",
        executable="cmd2gazebo.py",
        # output="screen", 
    )
    ld = LaunchDescription()
    
    ld.add_action(odom_vel)
    ld.add_action(status_pub)
    ld.add_action(spawner)

    return ld