from enum import IntEnum
import numpy as np
import rclpy
from rclpy.node import Node
from pygbn import gbn

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
    BENDING_1D = 0
    CIRCLE = 10
    HALF_8_SHAPE = 20
    FULL_8_SHAPE = 21
    # System identification signals
    # Chirp: https://en.wikipedia.org/wiki/Sine_wave
    CHIRP = 30
    # Generalized binary noise
    # https://www.sciencedirect.com/science/article/abs/pii/000510989090156C
    # switches constantly with random timing between one positive and negative amplitude
    GBN = 32
    # Randomly samples the amplitude from a random distribution when switching the state
    GBN_RAND_AMPLITUDE = 34
    GBN_RAND_AMPLITUDE_RAND_ANGLE = 36
    # Staircase: sequence of step responses to higher amplitudes
    STAIRCASE = 38

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

        # Option if we should wait with trajectory for VTEM boot-up
        self.declare_parameter('wait_for_vtem', True)
        self.wait_for_vtem = self.get_parameter('wait_for_vtem').value

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
            self.A_p = 1 / self.radius_CoP * np.array([[0, -1/2], [0, 1/2], [1/2, 0], [-1/2, 0],])
        elif self.num_chambers == 3:
            self.A_p = 1 / self.radius_CoP * np.array([[0, 2/3], [-1/np.sqrt(3), -1/3], [1/np.sqrt(3), -1/3]])
        else:
            raise NotImplementedError

        self.declare_parameter('inflate_time', 5)
        self.inflate_time = self.get_parameter('inflate_time').value
        self.declare_parameter('experiment_duration', 60)
        self.experiment_duration = self.get_parameter('experiment_duration').value
        self.declare_parameter('deflate_time', 5)
        self.deflate_time = self.get_parameter('deflate_time').value

        self.declare_parameter('segment_trajectories', [SegmentTrajectoryType.BENDING_1D for i in range(self.num_segments)])
        self.segment_trajectories = self.get_parameter('segment_trajectories').value
        assert len(self.segment_trajectories) == self.num_segments

        self.declare_parameter('trajectory_frequencies', [0.1 for i in range(self.num_segments)])
        self.trajectory_frequencies = self.get_parameter('trajectory_frequencies').value
        assert len(self.trajectory_frequencies) == self.num_segments
        self.trajectory_periods = [1/x for x in self.trajectory_frequencies]

        self.declare_parameter('pressure_peaks', [1500 for i in range(self.num_segments)]) # [N]
        self.pressure_peaks = self.get_parameter('pressure_peaks').value # [N]
        assert len(self.pressure_peaks) == self.num_segments

        self.declare_parameter('torque_angles', [0.0 for i in range(self.num_segments)]) # [rad]
        self.torque_angles = self.get_parameter('torque_angles').value # [rad]
        assert len(self.torque_angles) == self.num_segments

        self.declare_parameter('node_frequency', 10.)
        self.node_frequency = self.get_parameter('node_frequency').value # [Hz]
        self.timer_period = 1 / self.node_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.counter = 0

        self.state = ExperimentState.BOOT_UP
        self.state_counter = 0

        self.commanded_pressures = np.zeros(shape=(self.num_segments, self.num_chambers))

        self.msg: FluidPressures = self.prep_fluid_pressures_msg()

        # random seed
        self.declare_parameter('seed', 101)
        self.seed = self.get_parameter('seed').value
        np.random.seed(self.seed)

        # system identification signals parameters
        self.chirp_freq0 = 0. # [Hz] starting frequency of chirp
        self.declare_parameter('chirp_rate', 0.01) # [Hz/s] chirp rate
        self.chirp_rate = self.get_parameter('chirp_rate').value

        if SegmentTrajectoryType.GBN in self.segment_trajectories \
            or SegmentTrajectoryType.GBN_RAND_AMPLITUDE in self.segment_trajectories \
                or SegmentTrajectoryType.GBN_RAND_AMPLITUDE_RAND_ANGLE in self.segment_trajectories:
            self.gbn_sequences = []
            self.gbn_amplitudes = []
            for trajectory_period in self.trajectory_periods:
                self.gbn_sequences.append(gbn(h=self.timer_period, T=1.1*self.experiment_duration, 
                                          A=1, ts=trajectory_period, flag=1, seed=self.seed))
                self.gbn_amplitudes.append(0)

        if SegmentTrajectoryType.STAIRCASE in self.segment_trajectories:
            self.declare_parameter('step_periods', [10. for i in range(self.num_segments)]) # [s]
            self.step_periods = self.get_parameter('step_periods').value # [s]
            assert len(self.step_periods) == self.num_segments
        
            self.staircase_sequences = []
            for step_period, trajectory_period in zip(self.step_periods, self.trajectory_periods):
                assert step_period <= trajectory_period
                num_steps = trajectory_period // (2*step_period)
                step_amplitude = 1 / num_steps

                timestep_sequence = np.arange(start=0, stop=trajectory_period, step=self.timer_period)
                condlist = []
                valuelist = []
                time = 0.
                amplitude = 0.
                while time < trajectory_period:
                    condlist.append((time <= timestep_sequence) * (timestep_sequence < (time + step_period)))

                    if time < (trajectory_period // 2) and amplitude < 1.:
                        amplitude += step_amplitude
                    else:
                        amplitude -= step_amplitude
                    valuelist.append(amplitude)

                    time += step_period

                sequence = np.piecewise(timestep_sequence, condlist, valuelist)
                self.staircase_sequences.append(sequence)

    def timer_callback(self):
        if self.state == ExperimentState.BOOT_UP:
            if self.vtem_status == True or self.wait_for_vtem == False:
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
                trajectory_counter = self.state_counter - int(num_completed_periods * self.trajectory_periods[segment_idx] / self.timer_period)
                force_peak = self.pressure_peaks[segment_idx] / np.max(self.A_p)

                if trajectory_type == SegmentTrajectoryType.BENDING_1D:
                    self.commanded_forces[segment_idx] = self.bending_1d_trajectory(segment_idx, trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.CIRCLE:
                    self.commanded_forces[segment_idx] = self.circle_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.HALF_8_SHAPE:
                    self.commanded_forces[segment_idx] = self.half_8_shape_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.FULL_8_SHAPE:
                    self.commanded_forces[segment_idx] = self.full_8_shape_trajectory(trajectory_time, self.trajectory_periods[segment_idx], force_peak)
                elif trajectory_type == SegmentTrajectoryType.CHIRP:
                    self.commanded_forces[segment_idx] = self.chirp_trajectory(segment_idx, trajectory_time, force_peak)
                elif trajectory_type == SegmentTrajectoryType.GBN:
                    self.commanded_forces[segment_idx] = self.gbn_trajectory(segment_idx, trajectory_counter, force_peak)
                elif trajectory_type in [SegmentTrajectoryType.GBN_RAND_AMPLITUDE, SegmentTrajectoryType.GBN_RAND_AMPLITUDE_RAND_ANGLE]:
                    rand_angle = False
                    if trajectory_type == SegmentTrajectoryType.GBN_RAND_AMPLITUDE_RAND_ANGLE:
                        rand_angle = True
                    self.commanded_forces[segment_idx] = self.gbn_rand_trajectory(segment_idx, trajectory_counter, force_peak, rand_angle)
                elif trajectory_type == SegmentTrajectoryType.STAIRCASE:
                    self.commanded_forces[segment_idx] = self.staircase_trajectory(segment_idx, trajectory_counter, force_peak)
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

    def bending_1d_trajectory(self, segment_idx: int, trajectory_time: float, trajectory_period: float, 
                                force_peak: float) -> np.array:
        if trajectory_time < 0.5*trajectory_period:
            f = force_peak * trajectory_time / (0.5*trajectory_period)
        else:
            f = force_peak * (2-trajectory_time / (0.5*trajectory_period))

        f_x = np.cos(self.torque_angles[segment_idx])*f
        f_y = np.sin(self.torque_angles[segment_idx])*f

        return np.array([f_x, f_y])

    def half_8_shape_trajectory(self, trajectory_time: float, trajectory_period: float, 
                                force_peak: float) -> np.array:
        # for description of trajectory: https://www.overleaf.com/read/dxvsqnksnqgt
        F_x, F_y = 1, 0.5

        f_x = force_peak * np.sin(2*np.pi*F_x*trajectory_time/trajectory_period)
        f_y = force_peak * np.sin(2*np.pi*F_y*trajectory_time/trajectory_period)

        return np.array([f_x, f_y])

    def full_8_shape_trajectory(self, trajectory_time: float, trajectory_period: float, 
                                force_peak: float) -> np.array:
        # for description of trajectory: https://www.overleaf.com/read/dxvsqnksnqgt
        F_x, F_y = 2, 1

        f_x = force_peak * np.sin(2*np.pi*F_x*trajectory_time/trajectory_period)
        f_y = force_peak * np.sin(2*np.pi*F_y*trajectory_time/trajectory_period)

        return np.array([f_x, f_y])

    def circle_trajectory(self, trajectory_time: float, trajectory_period: float, 
                          force_peak: float) -> np.array:
        f_x = force_peak * np.cos(2*np.pi*trajectory_time/trajectory_period)
        f_y = force_peak * np.sin(2*np.pi*trajectory_time/trajectory_period)

        return np.array([f_x, f_y])

    def chirp_trajectory(self, segment_idx: int, trajectory_time: float, force_peak: float) -> np.array:
        freq = self.chirp_freq0 + self.chirp_rate * trajectory_time

        f = force_peak * np.sin(2 * np.pi * freq * trajectory_time)

        f_x = np.cos(self.torque_angles[segment_idx])*f
        f_y = np.sin(self.torque_angles[segment_idx])*f

        return np.array([f_x, f_y])

    def gbn_trajectory(self, segment_idx: int, trajectory_counter: int, force_peak: float) -> np.array:
        gbn_value = self.gbn_sequences[segment_idx][int(trajectory_counter)]

        # we need to correct dbn value so that it is in range 0 to 1
        # gbn_value = (gbn_value + 1) / 2

        f = gbn_value * force_peak

        f_x = np.cos(self.torque_angles[segment_idx])*f
        f_y = np.sin(self.torque_angles[segment_idx])*f

        return np.array([f_x, f_y])

    def gbn_rand_trajectory(self, segment_idx: int, trajectory_counter: int, force_peak: float, random_torque_angle: False) -> np.array:
        gbn_sequence = self.gbn_sequences[segment_idx]

        sample_amplitude = False
        if trajectory_counter > 0 and gbn_sequence[int(trajectory_counter)] != gbn_sequence[int(trajectory_counter)-1]:
            sample_amplitude = True
        else:
            sample_amplitude = False 

        if sample_amplitude:
            if random_torque_angle:
                self.torque_angles[segment_idx] = np.random.uniform(low=0, high=2*np.pi)
                self.gbn_amplitudes[segment_idx] = np.random.uniform(low=0, high=force_peak)
            else:
                self.torque_angles[segment_idx] = self.torque_angles[segment_idx]
                self.gbn_amplitudes[segment_idx] = np.random.uniform(low=-force_peak, high=force_peak)

        f = self.gbn_amplitudes[segment_idx]

        f_x = np.cos(self.torque_angles[segment_idx])*f
        f_y = np.sin(self.torque_angles[segment_idx])*f

        return np.array([f_x, f_y])

    def staircase_trajectory(self, segment_idx: int, trajectory_counter: int, force_peak: float) -> np.array:
        f = self.staircase_sequences[segment_idx][trajectory_counter] * force_peak

        f_x = np.cos(self.torque_angles[segment_idx])*f
        f_y = np.sin(self.torque_angles[segment_idx])*f

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
                pressures_segment_sanitized[0] = pressures_segment[0] - np.min(np.stack([0, pressures_segment[0], pressures_segment[1]], axis=0))
                pressures_segment_sanitized[1] = pressures_segment[1] - np.min(np.stack([0, pressures_segment[0], pressures_segment[1]], axis=0))
                pressures_segment_sanitized[2] = pressures_segment[2] - np.min(np.stack([0, pressures_segment[2], pressures_segment[3]], axis=0))
                pressures_segment_sanitized[3] = pressures_segment[3] - np.min(np.stack([0, pressures_segment[2], pressures_segment[3]], axis=0))
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