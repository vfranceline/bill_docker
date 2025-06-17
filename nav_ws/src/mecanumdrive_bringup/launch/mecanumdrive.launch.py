import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'bill_navigation'
    bringup_package_name = 'mecanumdrive_bringup'

    mecanumdrive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'spawn_robot.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "mecanumdrive",
            "-allow_renaming",
            "true",
        ],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(bringup_package_name),
            "config",
            "mecanum_drive_controller.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller", "--param-file", robot_controllers],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    nodes = [
        gazebo,
        mecanumdrive,        
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        gz_spawn_entity,
        ros_gz_bridge
    ]

    return LaunchDescription(nodes)