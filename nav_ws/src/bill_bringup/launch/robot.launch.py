import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Caminhos para os pacotes
    bill_bringup_pkg = get_package_share_directory('bill_bringup')
    bill_description_pkg = get_package_share_directory('bill_description')
    bill_navigation_pkg = get_package_share_directory('bill_navigation')
    bill_gazebo_pkg = get_package_share_directory('bill_gazebo')

    # 1. Declarar o argumento de lançamento 'use_sim'
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) or real robot'
    )
    use_sim = LaunchConfiguration('use_sim')

    # 2. Iniciar o Robot State Publisher (comum para sim e real)
    # O valor de 'use_sim_time' será definido pelo argumento 'use_sim'
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bill_description_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    # 3. Grupo de Ações para Simulação (executado se use_sim for 'true')
    simulation_group = GroupAction(
        condition=IfCondition(use_sim),
        actions=[
            # Inicia o mundo Gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bill_gazebo_pkg, 'launch', 'world.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim}.items()
            ),
            # Spawna o robô no Gazebo e inicia a ponte ROS-Gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bill_navigation_pkg, 'launch', 'spawn_robot.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim}.items()
            ),
        ]
    )

    # 4. Grupo de Ações para o Robô Físico (executado se use_sim for 'false')
    # real_robot_group = GroupAction(
    #     condition=UnlessCondition(use_sim),
    #     actions=[
    #         # Inicia os drivers de hardware
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(bill_bringup_pkg, 'launch', 'hardware.launch.py')
    #             )
    #         )
    #     ]
    # )

    # 5. Iniciar a Navegação (comum para sim e real)
    # O valor de 'use_sim_time' e do mapa também são passados
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bill_navigation_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim,
            'map': os.path.join(bill_navigation_pkg, 'maps', 'mapa_save.yaml')
        }.items()
    )

    return LaunchDescription([
        use_sim_arg,
        robot_state_publisher_launch,
        simulation_group,
        navigation_launch
    ])