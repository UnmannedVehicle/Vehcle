
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():

    port = '/dev/ttyUSB1'
    baud = 230400

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    urdf_file_name = 'husky.urdf'
    urdf_file = os.path.join(
        get_package_share_directory('armbot_description'),
        'urdf',
        urdf_file_name)
    velocity_launch_file_dir = os.path.join(get_package_share_directory('velocity_smoother'), 'launch')  
     
    rviz2_config_flie= os.path.join(
        get_package_share_directory('armbot_description'),
        'config',
        'armbot_rviz2_config.rviz')

    armbot_dsp = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                         'use_sim_time': use_sim_time,
                         'publish_frequency': 10.0
                        }],
                        arguments=[urdf_file],
        )

    armbot_joint = launch_ros.actions.Node(
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        )


    rviz2 = launch_ros.actions.Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz2_config_flie]
        )
               
    minimal = launch_ros.actions.Node(
		package='aimibot',
		executable='aimibot', 
		name='aimibot',  
		#remappings=[ ('/aimibot/commands/velocity', '/aimibot/commands/velocity'),],        
		remappings=[ ('/aimibot/commands/velocity', '/aimibot/cmd_vel'),],
		output='screen',
		#namespace='Vehicle1',
		parameters=[{'port': port,
            		 'baud':baud}],
		)
    navigation = launch_ros.actions.Node(
            package='aimibot',
            executable='navigation',
            name='navigation',
            output='screen',
            
            )
    
    return LaunchDescription([
    	IncludeLaunchDescription(PythonLaunchDescriptionSource([velocity_launch_file_dir, '/velocity_smoother-launch.py']),),
        #armbot_dsp,
        navigation,
        minimal,
        #armbot_joint,
        #rviz2,
    ])

