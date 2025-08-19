import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Caminho para a pasta do seu pacote
    pkg_bill_navigation = get_package_share_directory('bill_navigation')

    # Caminhos para os arquivos de configuração
    # ADICIONADO: Caminho para o arquivo de configuração do Lidar
    lslidar_param_file = os.path.join(
        pkg_bill_navigation, 'config', 'lsn10p.yaml'
    )
    slam_toolbox_params_file = os.path.join(
        pkg_bill_navigation, 'config', 'slam_toolbox_mapping.yaml'
    )

    # Declaração dos argumentos de lançamento
    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Flag to enable use_sim_time'
    )

    # --- Definição dos Nós ---

    # ADICIONADO: Nó do Driver do Lidar
    # Este nó é essencial para publicar os dados brutos da nuvem de pontos
    lslidar_driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        parameters=[lslidar_param_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan_convertido',
            'publish_tf': True,
            'publish_odom_tf': True,
            'max_laser_range': 20.0,
            'min_laser_range': 0.1,
            'laser_min_height': -0.1,
            'laser_max_height': 0.1,
            'angle_min': -3.1415,  # -180 graus
            'angle_max': 3.1415,   # +180 graus
            'use_inf': True,
            'transform_publish_period': 0.05,
        }]
    )

    # Nó para converter PointCloud2 para LaserScan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/lslidar_point_cloud'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'laser_link',
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.1415, # -180 graus
            'angle_max': 3.1415,  # +180 graus
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            # MELHORIA: Passando use_sim_time para garantir consistência
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Nó do SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file,
            # Sobrescreve o tópico do scan e garante o uso correto do tempo
            # {'scan_topic': '/scan_convertido'},
            # MELHORIA: Passando use_sim_time para garantir consistência
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Nó do RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bill_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # --- Montagem da Descrição de Lançamento ---

    ld = LaunchDescription()

    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)
    # ld.add_action(lslidar_driver_node)
    # ld.add_action(rf2o_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld