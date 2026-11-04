"""Hospital world with the JetArm on the mecanum chassis.

    ros2 launch robot_gazebo hospital.launch.py

Starts Gazebo (ogre renderer - ogre2 shows a blank view in this VM), the
ROS <-> Gazebo bridge (cmd_vel, odom, camera, joint states), the robot
model from urdf/jetarm_mecanum.gazebo.xacro and the arm/gripper controllers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_gazebo')
    world = os.path.join(pkg, 'worlds', 'hospital.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'jetarm_mecanum.gazebo.xacro')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
        launch_arguments=[('ign_args', [' -r --render-engine ogre ' + world])])

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'ros_ign_bridge.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                     'use_sim_time': True}])

    spawn = Node(
        package='ros_ign_gazebo', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'robot',
                   '-x', '0', '-y', '0', '-z', '0.002'],
        parameters=[{'use_sim_time': True}])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name], output='screen')

    jsb, arm, grip = (spawner('joint_state_broadcaster'), spawner('arm_controller'),
                      spawner('gripper_controller'))

    return LaunchDescription([
        gazebo, bridge, robot_state_publisher, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grip])),
    ])
