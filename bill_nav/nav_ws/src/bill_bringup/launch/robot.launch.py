import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_urdf_path = get_package_share_directory('bill_description')
    pkg_bill_navigation = get_package_share_directory('bill_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_urdf_path)
    #os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='Nome do arquivo Xacro/URDF do robô real'
    )


    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_urdf_path,
        "urdf","robots",
        LaunchConfiguration('model') 
    ])

    # Launch Rviz with diff bot rviz file
    rviz_config_file = os.path.join(pkg_bill_navigation, 'rviz', 'nav.config.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz_launch_arg),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',
                    remappings=[('/map', 'map'),
                                ('/tf', 'tf'),
                                ('/tf_static', 'tf_static'),
                                ('/goal_pose', 'goal_pose'),
                                ('/clicked_point', 'clicked_point'),
                                ('/initialpose', 'initialpose')])]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}] # Garanta que use_sim_time seja False
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': False},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_link'],
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bill_navigation, 'config', 'ekf.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[("/odometry/filtered", "/odom")]
    )

    # Launch Twist Mux
    twist_mux_params = os.path.join(pkg_bill_navigation,'config','twist_mux_params.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(rviz2)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    launchDescriptionObject.add_action(twist_mux)
    launchDescriptionObject.add_action(tf2_node)

    return launchDescriptionObject