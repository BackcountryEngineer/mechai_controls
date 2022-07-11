from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    sims_pkg_share = get_package_share_directory("mechai_sims")
    controls_pkg_share = get_package_share_directory("mechai_controls")

    default_model_path = PathJoinSubstitution([sims_pkg_share, "urdf/mecanum_drive.xacro.urdf"])
    default_world_path = PathJoinSubstitution([sims_pkg_share, "world/sample_nav2.world"])
    default_rviz_config_path = PathJoinSubstitution([sims_pkg_share, "rviz/urdf_config.rviz"])

    default_controller_config = PathJoinSubstitution([controls_pkg_share, "config/mecanum_driver_controllers.yaml"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([get_package_share_directory("gazebo_ros"), "launch"]), "/gazebo.launch.py"])
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}]
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "mecanum_drive",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    load_joint_state_interface = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"]
    )

    load_joint_control_interface = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "-c", "/controller_manager"]
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration("rvizconfig")],
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot urdf file"
        ),
        DeclareLaunchArgument(
            "world", 
            default_value=default_world_path,
            description="Specify world file name"
        ),
        DeclareLaunchArgument(
            name="rvizconfig", 
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file"
        ),
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time"
        ),
        DeclareLaunchArgument(
            name="gui", 
            default_value="False",
            description="Flag to enable gui"
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_interface],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_interface,
                on_exit=[load_joint_control_interface],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_control_interface,
        #         on_exit=[rviz_node],
        #     )
        # ),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])