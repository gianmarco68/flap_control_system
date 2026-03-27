# ros2_ws/src/state_estimation/launch/live_state_estimation.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Variabile per il tempo (LIVE = orologio di sistema)
    sim_time_param = {'use_sim_time': False}
    
    # 2. Percorso del file YAML dell'EKF
    ekf_config_path = PathJoinSubstitution([
        FindPackageShare('state_estimation'),
        'config',
        'ekf.yaml' # Assicurati di aver salvato il blocco "ekf_node" in questo file!
    ])

    # === NODI DELLA STIMA ALTEZZA ===
    height_estimator = Node(
        package='state_estimation',
        executable='height_estimator_node',
        name='height_estimator_node',
        output='screen',
        parameters=[sim_time_param]
    )

    # === NODI DELLA CATENA GPS & EKF ===
    gps_header_fixer = Node(
        package='state_estimation',
        executable='gps_header_fixer_node',
        name='gps_header_fixer_node',
        output='screen',
        parameters=[sim_time_param]
    )

    gps_input_adapter = Node(
        package='state_estimation',
        executable='gps_input_adapter_node',
        name='gps_input_adapter_node',
        output='screen',
        parameters=[sim_time_param]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config_path, sim_time_param] # Carica il file YAML + il sim_time!
    )

    gps_output_adapter = Node(
        package='state_estimation',
        executable='gps_output_adapter_node',
        name='gps_output_adapter_node',
        output='screen',
        parameters=[sim_time_param]
    )

    # 3. Creiamo e ritorniamo la Launch Description
    return LaunchDescription([
        height_estimator,
        gps_header_fixer,
        gps_input_adapter,
        ekf_node,
        gps_output_adapter
    ])