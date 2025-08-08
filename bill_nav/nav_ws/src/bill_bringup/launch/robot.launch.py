import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_urdf_path = get_package_share_directory('bill_description')
    pkg_bill_navigation = get_package_share_directory('bill_navigation')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='Nome do arquivo Xacro/URDF do robô real'
    )

    urdf_file_path = PathJoinSubstitution([
        pkg_urdf_path, "urdf", "robots", LaunchConfiguration('model')
    ])

    rviz_config_file = os.path.join(pkg_bill_navigation, 'rviz', 'nav.config.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
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
                        ('/initialpose', 'initialpose')]
        )]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    pkg_urdf_path, 'urdf', 'robots', LaunchConfiguration('model')
                ])
            ]),
            'use_sim_time': False
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
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

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[os.path.join(pkg_bill_navigation, 'config', 'twist_mux_params.yaml'),
                    {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    return LaunchDescription([
        model_arg,
        rviz_launch_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # ekf_node,
        # twist_mux,
        # rviz2
    ])
