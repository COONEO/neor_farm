from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node( 
            name='nano_motor_adaptor_foxy_node',
            package='nano_motor_adaptor_foxy',
            executable='nano_motor_adaptor_foxy_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200, 
                'control_rate_': 10,
                'rear_odom_correct_param_': 0,
                'wheel_distance_': 0.214,
            }],
        ),
    ])
