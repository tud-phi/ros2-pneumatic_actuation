from launch import LaunchDescription
from launch_ros.actions import Node

from pneumatic_actuation_demos.pressure_trajectory_node import SegmentTrajectoryType

def generate_launch_description():
    num_segments = 1
    num_chambers = 4

    commanded_pressures_topic = "/pneumatic_actuation/commanded_pressures"
    measured_pressures_topic = "/pneumatic_actuation/measured_pressures"
    vtem_status_topic = "/vtem_control/vtem_status"

    common_vtem_params = {"num_valves": num_segments*num_chambers, "modbus_node": "192.168.4.3", "modbus_service": "502"}

    return LaunchDescription([
        Node(
            package='pneumatic_actuation_demos',
            namespace='pneumatic_actuation',
            executable='pressure_trajectory_node',
            parameters=[
                {
                    "commanded_pressures_topic": commanded_pressures_topic,
                    "commanded_pressures_array_topic": "commanded_pressures_array",
                    "deflate_time": 5,
                    "experiment_duration": 60,
                    "pressure_peaks": [50*100],
                    "inflate_time": 5,
                    "node_frequency": 10,
                    "num_chambers": num_chambers,
                    "num_segments": num_segments,
                    "pressure_offset": 125*100,
                    "radius_CoP": 0.1,
                    "segment_trajectories": [SegmentTrajectoryType.BENDING_1D_X],
                    "trajectory_frequencies": [0.1],
                    "vtem_status_topic": vtem_status_topic,
                }
            ]
        ),
        Node(
            package='vtem_control_cpp',
            namespace='vtem_control',
            executable='input_pressures_sub_node',
            parameters=[
                common_vtem_params,
                {"input_pressures_topic": commanded_pressures_topic, "max_pressure": 200*100.0}
            ]
        ),
        Node(
            package='vtem_control_cpp',
            namespace='vtem_control',
            executable='output_pressures_pub_node',
            parameters=[
                common_vtem_params,
                {"output_pressures_topic": "output_pressures", "pub_freq": 50., "vtem_status_topic": vtem_status_topic}
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