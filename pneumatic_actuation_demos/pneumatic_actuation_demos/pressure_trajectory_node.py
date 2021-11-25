from enum import IntEnum
import numpy as np
import rclpy
from rclpy.node import Node

from pneumatic_actuation_msgs.msg import FluidPressures
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, Float64MultiArray, MultiArrayLayout, MultiArrayDimension


class ExperimentState(IntEnum):
    """
    Enum class for the experiment state.
    """
    BOOT_UP = 0
    INFLATE = 1
    RUNNING = 2
    DEFLATE = 3

class SegmentTrajectoryType(IntEnum):
    """
    Enum class for the trajectory type of an individual segment.
    """
    BENDING_1D_X = 0
    BENDING_1D_Y = 0
    CIRCLE = 10
    HALF_8_SHAPE = 20
    FULL_8_SHAPE = 21


class PressureTrajectoryNode(Node):

    def __init__(self):
        super().__init__('pressure_trajectory_node')

        self.declare_parameter('num_segments', 1)
        self.num_segments = self.get_parameter('num_segments').value
        self.declare_parameter('num_chambers', 4) # air chambers per segment
        self.num_chambers = self.get_parameter('num_chambers').value

        # Publisher of commanded pressures as pneumatic_actuation_msgs.msg.FluidPressures
        self.declare_parameter('commanded_pressures_topic', '/pneumatic_actuation/commanded_pressures')
        commanded_pressures_topic = self.get_parameter('commanded_pressures_topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(FluidPressures, commanded_pressures_topic, 10)

        # Publisher of commanded pressures as std_msgs.msg.Float64MultiArray
        self.declare_parameter('commanded_pressures_array_topic', '/pneumatic_actuation/commanded_pressures_array')
        commanded_pressures_array_topic = self.get_parameter('commanded_pressures_array_topic').get_parameter_value().string_value
        self.publisher_array = self.create_publisher(Float64MultiArray, commanded_pressures_array_topic, 10)

        # Subscriber to VTEM status messages
        self.declare_parameter('vtem_status_topic', '/vtem_control/vtem_status')
        vtem_status_topic = self.get_parameter('vtem_status_topic').get_parameter_value().string_value
        self.sub_vtem_status = self.create_subscription(Bool, vtem_status_topic, self.vtem_status_callback, 10)
        self.vtem_status = False

        self.declare_parameter('pressure_offset', 150*100) # pressure in all chambers in straight configuration [Pa]
        self.pressure_offset = self.get_parameter('pressure_offset').value

        self.declare_parameter('radius_CoP', 0.1) # radius from center-line of segment to the center of pressure of each chamber
        self.radius_CoP = self.get_parameter('radius_CoP').value

        if self.num_chambers == 4:
            self.A_p = 1 / self.radius_CoP * np.array([[1/2, 0], [0, 1/2], [-1/2, 0], [0, -1/2]])
        elif self.num_chambers == 3:
            self.A_p = 1 / self.radius_CoP * np.array([[2/3, 0], [-1/3, 1/np.sqrt(3)], [-1/3, -1/np.sqrt(3)]])
        else:
            raise NotImplementedError

        self.declare_parameter('inflate_time', 5)
        self.inflate_time = self.get_parameter('inflate_time').value
        self.declare_parameter('experiment_duration', 60)
        self.experiment_duration = self.get_parameter('experiment_duration').value
        self.declare_parameter('deflate_time', 5)
        self.deflate_time = self.get_parameter('deflate_time').value

        self.declare_parameter('segment_trajectories', [SegmentTrajectoryType.BENDING_1D_X])
        self.segment_trajectories = self.get_parameter('segment_trajectories').value
        assert len(self.segment_trajectories) == self.num_segments

        self.declare_parameter('trajectory_frequencies', [0.1])
        self.trajectory_frequencies = self.get_parameter('trajectory_frequencies').value
        assert len(self.trajectory_frequencies) == self.num_segments
        self.trajectory_periods = [1/x for x in self.trajectory_frequencies]

        self.declare_parameter('pressure_peaks', [1500]) # [N]
        self.pressure_peaks = self.get_parameter('pressure_peaks').value # [N]
        assert len(self.pressure_peaks) == self.num_segments

        self.declare_parameter('node_frequency', 10)
        self.node_frequency = self.get_parameter('node_frequency').value # [Hz]
        self.timer_period = 1 / self.node_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.counter = 0

        self.state = ExperimentState.INFLATE
        self.state_counter = 0

        self.commanded_pressures = np.zeros(shape=(self.num_segments, self.num_chambers))

        self.msg: FluidPressures = self.prep_fluid_pressures_msg()

    def timer_callback(self):
        if self.state == ExperimentState.BOOT_UP:
            if self.vtem_status == True:
                self.state = ExperimentState.INFLATE
                self.state_counter = 0
            else:
                self.state_counter += 1
        elif self.state == ExperimentState.INFLATE:
            self.commanded_pressures = (self.state_counter+1)*self.timer_period/self.inflate_time*np.ones_like(self.commanded_pressures)*self.pressure_offset

            if self.state_counter*self.timer_period >= self.inflate_time:
                self.state = ExperimentState.RUNNING
                self.state_counter = 0
            else:
                self.state_counter += 1
        elif self.state == ExperimentState.RUNNING:
            
            self.commanded_forces = np.zeros(shape=(self.num_segments, 2))
            for segment_idx in range(self.num_segments):
                trajectory_type = self.segment_trajectories[segment_idx]
                num_completed_periods = self.state_counter * self.timer_period // self.trajectory_periods[segment_idx]
                trajectory_time = self.state_counter * self.timer_period - num_completed_periods*self.trajectory_periods[segment_idx]
                force_peak = self.pressure_peaks[segment_idx] / np.max(self.A_p)

                if trajectory_type == SegmentTrajectoryType.BENDING_1D_X:
                    self.commanded_forces[segment_idx] = self.bending_1d_x_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.BENDING_1D_Y:
                    self.commanded_forces[segment_idx] = self.bending_1d_y_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.CIRCLE:
                    self.commanded_forces[segment_idx] = self.circle_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.HALF_8_SHAPE:
                    self.commanded_forces[segment_idx] = self.half_8_shape_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.FULL_8_SHAPE:
                    self.commanded_forces[segment_idx] = self.full_8_shape_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                else:
                    raise NotImplementedError

            self.commanded_pressures = self.forces_to_pressures(self.commanded_forces)

            if self.state_counter*self.timer_period >= self.experiment_duration:
                self.state = ExperimentState.DEFLATE
                self.state_counter = 0
            else:
                self.state_counter += 1

        elif self.state == ExperimentState.DEFLATE:
            self.commanded_pressures = (1-(self.state_counter+1)*self.timer_period/self.inflate_time)*np.ones_like(self.commanded_pressures)*self.pressure_offset

            if self.state_counter*self.timer_period >= self.deflate_time:
                self.shutdown()
            else:
                self.state_counter += 1

        self.get_logger().info(f'Commanding pressure at t={(self.counter*self.timer_period):.2f} with state {self.state.name}: {self.commanded_pressures.flatten()}')

        msg_multi_array = Float64MultiArray()
        msg_multi_array.layout = MultiArrayLayout()
        msg_multi_array.layout.dim = [MultiArrayDimension(label="segments"), MultiArrayDimension(label="chambers")]
        msg_multi_array.layout.data_offset = 0
        msg_multi_array.data = self.commanded_pressures.flatten().tolist()
        self.publisher_array.publish(msg_multi_array)

        self.msg = self.prep_fluid_pressures_msg()
        self.publisher.publish(self.msg)
        
        # self.get_logger().info(f'Publishing msg {self.counter}: {self.msg.data}')
        self.counter += 1

    def vtem_status_callback(self, msg):
        self.vtem_status = msg.data

    def bending_1d_x_trajectory(self, trajectory_time, trajectory_period, force_peak) -> np.array:
        if trajectory_time < 0.5*trajectory_period:
            f_x = force_peak * trajectory_time / (0.5*trajectory_period)
        else:
            f_x = force_peak * (2-trajectory_time / (0.5*trajectory_period))

        f_y = 0

        return np.array([f_x, f_y])

    def bending_1d_y_trajectory(self, trajectory_time, trajectory_period, force_peak) -> np.array:
        f_x = 0

        if trajectory_time < 0.5*trajectory_period:
            f_y = force_peak * trajectory_time / (0.5*trajectory_period)
        else:
            f_y = force_peak * (2-trajectory_time / (0.5*trajectory_period))

        return np.array([f_x, f_y])

    def half_8_shape_trajectory(self, trajectory_time, trajectory_period, force_peak) -> np.array:
        # for description of trajectory: https://www.overleaf.com/read/dxvsqnksnqgt
        F_x, F_y = 1, 0.5

        f_x = force_peak * np.sin(2*np.pi*F_x*trajectory_time/trajectory_period)
        f_y = force_peak * np.sin(2*np.pi*F_y*trajectory_time/trajectory_period)

        return np.array([f_x, f_y])

    def full_8_shape_trajectory(self, trajectory_time, trajectory_period, force_peak) -> np.array:
        # for description of trajectory: https://www.overleaf.com/read/dxvsqnksnqgt
        F_x, F_y = 2, 1

        f_x = force_peak * np.sin(2*np.pi*F_x*trajectory_time/trajectory_period)
        f_y = force_peak * np.sin(2*np.pi*F_y*trajectory_time/trajectory_period)

        return np.array([f_x, f_y])

    def circle_trajectory(self, trajectory_time, trajectory_period, force_peak) -> np.array:
        f_x = force_peak * np.cos(2*np.pi*trajectory_time/trajectory_period)
        f_y = force_peak * np.sin(2*np.pi*trajectory_time/trajectory_period)

        return np.array([f_x, f_y])

    def forces_to_pressures(self, torques: np.array) -> np.array:
        """
            Convert a torque at tip of segment to a pressure.
            Input: num_segments x 2 array of torques
            Output: num_segments x num_chambers array of pressures
        """

        pressures = np.zeros(shape=(self.num_segments, self.num_chambers))
        for segment_idx in range(self.num_segments):
            torque_seqment = torques[segment_idx, ...]

            pressures_segment = self.pressure_offset + self.A_p @ torque_seqment

            pressures_segment_sanitized = np.zeros_like(pressures_segment)
            if self.num_chambers == 4:
                pressures_segment_sanitized[0] = pressures_segment[0] - np.min(np.stack([0, pressures_segment[0], pressures_segment[2]], axis=0))
                pressures_segment_sanitized[1] = pressures_segment[1] - np.min(np.stack([0, pressures_segment[1], pressures_segment[3]], axis=0))
                pressures_segment_sanitized[2] = pressures_segment[2] - np.min(np.stack([0, pressures_segment[0], pressures_segment[2]], axis=0))
                pressures_segment_sanitized[3] = pressures_segment[3] - np.min(np.stack([0, pressures_segment[1], pressures_segment[3]], axis=0))
            elif self.num_chambers == 3:
                pressures_segment_sanitized = pressures_segment - np.min(np.concatenate([np.array([0]), pressures_segment], axis=0))
            else:
                raise NotImplementedError

            pressures[segment_idx, ...] = pressures_segment_sanitized

        return pressures

    def prep_fluid_pressures_msg(self) -> FluidPressures:
        msg = FluidPressures()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = []
        for pressure in self.commanded_pressures.reshape(-1):
            fluid_pressure_msg = FluidPressure()
            fluid_pressure_msg.header.stamp = self.get_clock().now().to_msg()
            fluid_pressure_msg.fluid_pressure = pressure
            msg.data.append(fluid_pressure_msg)
        return msg

    def shutdown(self):
        self.destroy_node()
        # rclpy.shutdown()
        exit(0)


def main(args=None):
    rclpy.init(args=args)

    pressure_trajectory_node = PressureTrajectoryNode()

    rclpy.spin(pressure_trajectory_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pressure_trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()