from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Include navigation2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'map': '/home/phubet2547/map/map_world.yaml'#ใส่ที่อยู่ map
        }.items()
    )
    
    # Include robot integrate launch (XML)
    robot_integrate_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'robot_integrate.launch.xml'
            ])
        ])
    )
    
    return LaunchDescription([
        nav2_launch,
        robot_integrate_launch,
    ])
