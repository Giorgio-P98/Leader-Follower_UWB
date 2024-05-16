import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

from launch_ros.actions import Node
import xacro
import rclpy

rclpy.logging.set_logger_level

THIS_PATH = os.getcwd()

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    # pkg_name = 'running_box'
    # file_subpath = 'description/prism/prism.urdf.xacro'
    
    pkg_name = 'move_target'
    file_subpath = 'description/person_standing/person_standing.urdf.xacro'
    
    


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    
    # Configure the node
    
    px4_autopilot = ExecuteProcess(
        cmd=[[
            'cd ~/PX4-Autopilot/ && make px4_sitl gazebo PX4_SITL_WORLD='+THIS_PATH+'/github-Unmanned_Project/Leader-Follower_Simulation/worlds/empty.world output=log'
        ]],
        shell=True
    )
    
    micro_ros_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 -p 8888'
        ]],
        shell=True
    )
    
    node_robot_state_publisher = TimerAction(
            period=8.0, 
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    output='log',
                    parameters=[{'robot_description': robot_description_raw,
                                 'use_sim_time': True}]
                )
            ])

    joint_state_publisher = TimerAction(
            period=8.0, 
            actions=[
                Node(
                    package='move_target',
                    executable='publisher_good',
                    output='log',
                    parameters=[{'robot_description': robot_description_raw,
                                 'use_sim_time': True}]
                )
            ])

    spawn_entity = TimerAction(
            period=8.0, 
            actions=[
                Node(
                    package='gazebo_ros', 
                    executable='spawn_entity.py',
                    arguments=['-topic', 
                               'robot_description',
                               '-entity', 
                               'person'],
                    output='log'
                )
            ])

    sensor_sim = TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='sensor_sim',
                    executable='sensor_sim',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ])
    
    offboard_control = TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='offboard_control',
                    executable='offboard_control',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ])

    # Run the node
    return LaunchDescription([
        px4_autopilot,
        micro_ros_agent,
        node_robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        sensor_sim,
        offboard_control
    ])

generate_launch_description()
 
