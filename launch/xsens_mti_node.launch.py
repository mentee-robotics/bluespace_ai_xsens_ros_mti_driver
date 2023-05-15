from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from os import getenv, path

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)

    code_path = getenv("CODE_PATH", None)
    imu_params_path = path.join(code_path, "ros2_workspace/menteebot/configs/sensors/imus/xsens_mti_node_uart.yaml")
    parameters_file_path = Path(imu_params_path)
    xsens_mti_node = Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[parameters_file_path],
            arguments=[]
            )
    ld.add_action(xsens_mti_node)

    return ld
