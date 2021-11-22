from enum import IntEnum
import numpy as np
import rclpy
from rclpy.node import Node

from pneumatic_actuation_msgs.msg import FluidPressures
from sensor_msgs.msg import FluidPressure


class ExperimentState(IntEnum):
    """
    Enum class for the experiment state.
    """
    INFLATE = 0
    RUNNING = 1
    DEFLATE = 2


class PressureTrajectoryNode(Node):

    def __init__(self):
        super().__init__('pressure_trajectory_node')

        self.declare_parameter('num_segments', 1)
        self.num_segments = self.get_parameter('num_segments').value
        self.declare_parameter('num_chambers', 3) # air chambers per segment
        self.num_chambers = self.get_parameter('num_chambers').value

        self.declare_parameter('input_pressures_topic', '/vtem_control/input_pressures')
        vtem_input_pressures_topic = self.get_parameter('input_pressures_topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(FluidPressures, vtem_input_pressures_topic, 10)

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
        self.declare_parameter('deflate_time', 5)
        self.deflate_time = self.get_parameter('deflate_time').value

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.counter = 0

        self.state = ExperimentState.INFLATE
        self.state_counter = 0

        self.commanded_pressures = np.zeros(shape=(self.num_segments, self.num_chambers))
        # self.commanded_pressures_history = []

        self.msg: FluidPressures = self.prepare_fluid_pressures_msg()

    def timer_callback(self):
        if self.state == ExperimentState.INFLATE:
            self.commanded_pressures = (self.state_counter+1)*self.timer_period/self.inflate_time*np.ones_like(self.commanded_pressures)*self.pressure_offset

            if self.state_counter*self.timer_period >= self.inflate_time:
                self.state = ExperimentState.RUNNING
                self.state_counter = 0
            else:
                self.state_counter += 1
        elif self.state == ExperimentState.RUNNING:
            self.commanded_pressures = self.torques_to_pressures(100*np.ones(shape=(self.num_segments, 2)))

            if self.state_counter*self.timer_period >= 10:
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

        # self.commanded_pressures_history.append(self.commanded_pressures.copy())

        self.msg = self.prepare_fluid_pressures_msg()

        self.publisher_.publish(self.msg)
        # self.get_logger().info(f'Publishing msg {self.counter}: {self.msg.data}')
        self.counter += 1

    def torques_to_pressures(self, torques: np.array) -> np.array:
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

    def prepare_fluid_pressures_msg(self) -> FluidPressures:
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