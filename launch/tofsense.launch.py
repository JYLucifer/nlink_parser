from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',  
        description='TOFSense串口设备路径，例如 /dev/ttyACM0 或 /dev/ttyUSB0'
    )
    
    declare_baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='TOFSense串口通信波特率'
    )
    
    declare_inquire_mode_arg = DeclareLaunchArgument(
        'inquire_mode',
        default_value='false',
        description='是否开启查询模式'
    )

    tofsense_node = Node(
        package='nlink_parser',           
        executable='tofsense_node',   
        name='tofsense0',                 
        output='screen',                  
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'inquire_mode': LaunchConfiguration('inquire_mode'),
        }]
    )

    return LaunchDescription([
        declare_port_name_arg,
        declare_baud_rate_arg,
        declare_inquire_mode_arg,
        tofsense_node,
    ])