
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
    
    map_file_name = 'ma.osm'
    origin_latitude = 28.185424
    origin_longitude = 112.945468
    port = '/dev/ttyUSB1'
    baud = 921600

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    publish_frequency = LaunchConfiguration('publish_frequency', default='20.0')
    urdf_file_name = 'aimibot.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('description'),
        'urdf',
        urdf_file_name)
        
    rviz_config_dir = os.path.join(
        get_package_share_directory('description'),
        'rviz',
        'model.rviz')
        
    velocity_launch_file_dir = os.path.join(get_package_share_directory('velocity_smoother'), 'launch')
    osm_launch_file_dir = os.path.join(get_package_share_directory('osm_plan'), 'launch')
      
    state_publisher = launch_ros.actions.Node(
 		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[{'use_sim_time': use_sim_time,
                     'publish_frequency': 10.0 }],
        arguments=[urdf],
        )
        
    minimal = launch_ros.actions.Node(
		package='aimibot',
		executable='aimibot', 
		name='aimibot',          
		remappings=[ ('/aimibot/commands/velocity', '/aimibot/cmd_vel'),],
		output='screen',
		namespace='Vehicle1',
		parameters=[{'port': port,
            		 'baud':baud}],
		)

    
    return LaunchDescription([
    	IncludeLaunchDescription(PythonLaunchDescriptionSource([velocity_launch_file_dir, '/velocity_smoother-launch.py']),),
    	IncludeLaunchDescription(PythonLaunchDescriptionSource([osm_launch_file_dir, '/planner_node_launch.py']),),
        state_publisher,
        minimal,
        #armbot_rviz2,
    ])

