from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',  
        description='Path to the serial port device, e.g., /dev/ttyACM0'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',  
        description='Baud rate for serial communication, e.g., 921600'
    )

    linktrack_node = Node(
        package='nlink_parser',
        executable='linktrack_node',
        name='linktrack0',
        output='screen',
        parameters=[
            {'port_name': LaunchConfiguration('port_name')},
            {'baud_rate': LaunchConfiguration('baud_rate')}
        ]
    )

    return LaunchDescription([
        port_name_arg,
        baud_rate_arg,
        linktrack_node,
    ])