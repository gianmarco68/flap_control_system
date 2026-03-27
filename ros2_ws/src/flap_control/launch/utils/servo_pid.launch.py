from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pid_node = Node(
        package='flap_control',
        executable='pid_node',
        name='pid_node',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/reference', '/wand_ref'),
            ('/measurement', '/wand_angle'),
            ('/control_value', '/servo_angle'),
        ]
    )
    ld.add_action(pid_node)

    servo_to_brunilde = Node(
        package='flap_control',
        executable='servo2brunilde',
        name='servo_to_brunilde',
        arguments=['--ros-args', '--log-level', 'WARN'],
        output='screen',
        remappings=[
            ('/servo_angle', '/servo_angle'),
        ]
    )
    ld.add_action(servo_to_brunilde)

    return ld
