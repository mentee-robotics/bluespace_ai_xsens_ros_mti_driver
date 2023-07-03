from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)
    
    pkg_shared_dir = get_package_share_directory('bluespace_ai_xsens_mti_driver')
    imu_params = DeclareLaunchArgument('imu_params', 
                                       default_value=os.path.join(pkg_shared_dir, 'param', 'xsens_mti_node.yaml')
    )
    
    xsens_mti_node = Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[LaunchConfiguration('imu_params')],
            arguments=[]
            )
    ld.add_action(imu_params)
    ld.add_action(xsens_mti_node)

    return ld
