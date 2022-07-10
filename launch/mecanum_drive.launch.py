import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("gazebo_ros"), "launch"), "/gazebo.launch.py"]),
    )

    mecanum_drive_xacro = os.path.join(get_package_share_directory("mechai_sims"), "urdf", "mecanum_drive.xacro.urdf")

    robot_doc = xacro.parse(open(mecanum_drive_xacro))
    xacro.process_doc(robot_doc)
    robot_params = {"robot_description": robot_doc.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_params]
    )

    urdf_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        arguments=["-topic", "/robot_description", "-entity", "ros_mecanum_drive"],
        output="screen"
    )

    config_dir = os.path.join(get_package_share_directory("mechai_sims"), "config")

    joy_config = os.path.join(config_dir, "joy_config.yaml")
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[joy_config]
    )

    teleop_config = os.path.join(config_dir, "teleop_config.yaml")
    print(teleop_config)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        output="screen",
        parameters=[teleop_config]
    )


    return LaunchDescription([
        gazebo,
        urdf_spawner,
        robot_state_publisher,
        joy_node,
        teleop_node
    ])