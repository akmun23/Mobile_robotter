from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy_node to read input from the joystick
        Node(
            package='joy', 
            executable='joy_node',  
            name='joy_node', 
            output='screen', 
            parameters=[{
                'dev': '/dev/input/js0'
            }]
        ),
        Node(
            package='controller_input',
            executable='controller_input', 
            name='controller_input',  
            output='screen'
        )
    ])

