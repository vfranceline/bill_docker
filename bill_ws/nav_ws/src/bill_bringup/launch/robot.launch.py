import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_urdf_path, 'rviz', 'rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': False},
        ]
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

    rf2o_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',             # Tópico de entrada do laser
            'odom_topic': '/odom_rf2o',             # Tópico de saída da odometria
            'publish_tf': False,                    # MUITO IMPORTANTE: Não deixe este nó publicar TFs
            'base_frame_id': 'base_link',           # Frame do robô
            'odom_frame_id': 'odom',                # Frame de odometria
            'freq_time': 0.1,                       # Frequência de publicação
            'verbose': False
        }]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    # launchDescriptionObject.add_action(rf2o_odometry_node)

    return launchDescriptionObject