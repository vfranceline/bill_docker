# esse launch roda o robot state publisher, o joint state publisher e o rviz para visualizar o robô
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
 
# definir os parametros configuraveis do robô
ARGUMENTS = [
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation')
]
 
 
def generate_launch_description():
    # definindo o nome dos arquivos
    urdf_package = 'bill_description'
    urdf_filename = 'robot_3d.urdf.xacro'
    rviz_config_filename = 'bill.rviz'
 
    # setando os paths para os arquivos
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', 'robots', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', rviz_config_filename])
 
    # variaveis de config do launch
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
 
    # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
 
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
 
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
 
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo')
    ]), value_type=str)
 
    # subscreve os joint states do robô e publica a posição 3d dos links
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])
 
    # publica os valores das joint states para as joints não fixas (rodas)
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(jsp_gui))
 
    # mesma coisa do start_joint_state_publisher_cmd porém com gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))
 
    # roda o rviz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])
 

    ld = LaunchDescription(ARGUMENTS)
 
    # declarar as opções de launch
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
 
    # add actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
 
    return ld