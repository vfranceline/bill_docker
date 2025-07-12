import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pkg_urdf_path = FindPackageShare('bill_description')
    default_rviz_config_path = PathJoinSubstitution([pkg_urdf_path, 'rviz', 'bill.rviz'])

    # joint state publisher gui para testar as joints
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    
    # rviz config path
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                    description='Absolute path to rviz config file')
    

    # modelo urdf
    model_arg = DeclareLaunchArgument(
        'model', default_value='robot_3d.urdf.xacro',
        description='Name of the URDF description to load'
    )

    # usando o pacote proprio do ros2 urdf com os args do bill
    urdf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'bill_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'robots', LaunchConfiguration('model')]),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(gui_arg)
    launchDescriptionObject.add_action(rviz_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(urdf)

    return launchDescriptionObject
