"""Hospital world with the JetArm on the mecanum chassis.

    ros2 launch robot_gazebo hospital.launch.py              # with the Gazebo window
    ros2 launch robot_gazebo hospital.launch.py gui:=false   # headless (server only)
    ros2 launch robot_gazebo hospital.launch.py video:=true  # overview camera 2 Hz (videos)
    ros2 launch robot_gazebo hospital.launch.py photo:=true  # + three 1280x800 room cameras

Screenshots without the GUI: python3 ~/sdp_sim/tools/sim_screenshot.py (fixed
overview camera in the world, /overview_cam/image, bridged here).

Starts Gazebo (ogre renderer - ogre2 shows a blank view in this VM), the
ROS <-> Gazebo bridge (camera, joint states), the robot model from
urdf/jetarm_mecanum.gazebo.xacro, the arm/gripper controllers and the planar
base driver ~/sdp_sim/scripts/planar_base.py (/controller/cmd_vel -> base joint
velocities /base/vel_*, and /odom from the base joint positions) and the smart
bedside table controller ~/sdp_sim/scripts/smart_table.py (contact sensor, weld
state and arm pose in; lift joint commands, /smart_table/state and /height out).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_gazebo')
    # video:=true -> hospital_video.sdf (same world, overview camera at 2 Hz instead of 0.5)
    # photo:=true -> hospital_photo.sdf (same world + room cameras /room_cam/{a,b,c}/image)
    world = PythonExpression(["'", os.path.join(pkg, 'worlds', 'hospital'), "' + ('_video' if '",
                              LaunchConfiguration('video'), "'.lower() == 'true' else ('_photo' if '",
                              LaunchConfiguration('photo'), "'.lower() == 'true' else '')) + '.sdf'"])
    xacro_file = os.path.join(pkg, 'urdf', 'jetarm_mecanum.gazebo.xacro')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
        # gui:=false adds -s (server only); sensors still render (camera images work)
        launch_arguments=[('ign_args', PythonExpression([
            "' -r ' + ('' if '", LaunchConfiguration('gui'), "'.lower() == 'true' else '-s ') + ",
            "'--render-engine ogre ' + '", world, "'"]))])

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
                   '-x', '0', '-y', '0', '-z', '0'],
        parameters=[{'use_sim_time': True}])

    # planar base: world-frame joint velocity commands ROS -> Gazebo JointController
    base_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='base_bridge',
        output='screen',
        arguments=['/base/vel_x@std_msgs/msg/Float64]ignition.msgs.Double',
                   '/base/vel_y@std_msgs/msg/Float64]ignition.msgs.Double',
                   '/base/vel_yaw@std_msgs/msg/Float64]ignition.msgs.Double',
                   # fixed overview camera for screenshots (tools/sim_screenshot.py)
                   '/overview_cam/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                   # container weld (DetachableJoint): commands + state, used by
                   # scripts/medicine_weld.py for confirmed attach/detach
                   '/medicine/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
                   '/medicine/detach@std_msgs/msg/Empty]ignition.msgs.Empty',
                   '/medicine/attached@std_msgs/msg/String[ignition.msgs.StringMsg',
                   # smart bedside table (scripts/smart_table.py): platform contact sensor,
                   # lift joint states and the stage position commands (N_STAGES = 3)
                   '/smart_table/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
                   '/smart_table/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                   ] + ['/smart_table/lift%d/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double' % i
                        for i in range(1, 4)] + [
                   # report cameras, only in hospital_photo.sdf (tools/sim_screenshot.py --topic)
                   '/room_cam/%s/image@sensor_msgs/msg/Image[ignition.msgs.Image' % k for k in 'abc'])
    planar_base = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/sdp_sim/scripts/planar_base.py')],
        output='screen')
    smart_table = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/sdp_sim/scripts/smart_table.py')],
        output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name], output='screen')

    jsb, arm, grip = (spawner('joint_state_broadcaster'), spawner('arm_controller'),
                      spawner('gripper_controller'))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = headless Gazebo server, no window'),
        DeclareLaunchArgument('video', default_value='false',
                              description='true = overview camera at 2 Hz (for tools/record_video.py)'),
        DeclareLaunchArgument('photo', default_value='false',
                              description='true = hospital_photo.sdf with three room cameras (report pictures)'),
        gazebo, bridge, base_bridge, planar_base, smart_table, robot_state_publisher, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grip])),
    ])
