from launch import LaunchDescription
from launch_ros.actions import Node

from pneumatic_actuation_demos.pressure_trajectory_node import SegmentTrajectoryType

def generate_launch_description():
    num_segments = 1
    num_chambers = 3

    commanded_pressures_topic = "/pneumatic_actuation/commanded_pressures"
    measured_pressures_topic = "/pneumatic_actuation/measured_pressures"

    use_vtem = True
    node_frequency = 100.
    vtem_status_topic = "/vtem_control/vtem_status"
    common_vtem_params = {"num_valves": num_segments*num_chambers, "modbus_node": "192.168.4.3", "modbus_service": "502"}

    node_list = [
        Node(
            package='pneumatic_actuation_demos',
            namespace='pneumatic_actuation',
            executable='pressure_trajectory_node',
            parameters=[
                {
                    "amplitude_derivative_signs": [1],
                    "commanded_pressures_topic": commanded_pressures_topic,
                    "commanded_pressures_array_topic": "commanded_pressures_array",
                    "deflate_time": 5,
                    # the amplitude will once extend to the pressure preak and then come back to zero pressure by the time the experiment ends
                    "experiment_duration": 120,
                    "inflate_time": 5,
                    "node_frequency": node_frequency,
                    "num_chambers": num_chambers,
                    "num_segments": num_segments,
                    "pressure_offsets": [225*100],
                    "pressure_peaks": [225*100],
                    "segment_trajectories": [SegmentTrajectoryType.SPIRAL_2D_CONST_LINEAR_VEL],
                    "trajectory_velocities": [0.125], # Nm / s
                    "vtem_status_topic": vtem_status_topic,
                    "wait_for_vtem": use_vtem,
                }
            ]
        )
    ]

    if use_vtem:
        node_list.extend([
            Node(
                package='vtem_control_cpp',
                namespace='vtem_control',
                executable='input_pressures_sub_node',
                parameters=[
                    common_vtem_params,
                    {"input_pressures_topic": commanded_pressures_topic, "max_pressure": 450*100.0}
                ]
            ),
            Node(
                package='vtem_control_cpp',
                namespace='vtem_control',
                executable='output_pressures_pub_node',
                parameters=[
                    common_vtem_params,
                    {"output_pressures_topic": "output_pressures", "pub_freq": node_frequency, "vtem_status_topic": vtem_status_topic}
                ]
            ),
            Node(
                package="topic_tools",
                namespace='relay_output_pressures',
                executable="relay",
                parameters=[{"input_topic": "/vtem_control/output_pressures", "output_topic": measured_pressures_topic}]
            ),
            Node(
                package="topic_tools",
                namespace='relay_output_pressures_array',
                executable="relay",
                parameters=[{"input_topic": "/vtem_control/output_pressures_array", "output_topic": f"{measured_pressures_topic}_array"}]
            )
        ])

    return LaunchDescription(node_list)