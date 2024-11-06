from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy_node to read input from the joystick
        Node(
            package='joy',  # Package containing joy_node
            executable='joy_node',  # Executable name
            name='joy_node',  # Node name
            output='screen',  # Output to screen
            parameters=[{
                # Optional: Specify joystick device if not default
                'dev': '/dev/input/js0'
            }]
        ),
        # Launch your custom controller_input node
        Node(
            package='controller_input',  # Your package name
            executable='controller_input',  # Your node's executable
            name='controller_input',  # Node name
            output='screen'  # Output to screen
        ),
        Node(package='lidar_data',
        executable='lidar_data',
        name='lidar_data',
        output='screen'
        )
    ])

