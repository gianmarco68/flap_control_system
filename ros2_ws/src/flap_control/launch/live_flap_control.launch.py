# /ros2_ws/src/flap_control/launch/live_flap_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pid_controller_node_config = PathJoinSubstitution([
        FindPackageShare('flap_control'),
        'config',
        'pid_controller_config.yaml'
    ])

    # === NODO 1: PID CONTROLLER (Il Cervello) ===
    pid_controller_node = Node(
        package='flap_control',
        executable='pid_controller_node',
        name='pid_controller_node',       
        output='screen',
        parameters=[pid_controller_node_config],
    )

    
    flap_controller_node = Node(
        package='flap_control',
        executable='flap_controller_node',
        name='flap_controller_node',    
        output='screen',
    )

    return LaunchDescription([
        pid_controller_node,
        flap_controller_node
    ])