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


    # Nó do SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_params_file,
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
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld