import sys
from collections import deque
from threading import Lock

import myactuator_rmd_py as rmd
import rclpy
import std_msgs.msg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs, RMDx8MotorConfig
from lib.motor_state.rmd_motor_state import RMDX8RunSettings

from .rmdx8_motor import RMDx8Motor

# Exponential filter coefficient for velocity smoothing.
# At 50 Hz (dt=0.02s) this gives ~95% reduction in ~1 second (3τ ≈ 1s, τ ≈ 0.33s).
_VEL_SMOOTH_ALPHA = 0.06
_VEL_SMOOTH_DT = 0.02
_VEL_ZERO_THRESHOLD = 1e-3  # rev/s — snap to zero below this to avoid jitter


class RMDx8MotorManager(Node):
    """
    Class to manage the control and storage of RMDx8 motors in the ROS system.
    """

    # String identifier for updating motor state
    _UPDATE_STATE = "UPDATE_STATE"

    def __init__(self) -> None:
        super().__init__("can_rmdx8_node")
        self.get_logger().info(colorStr("Launching can_rmdx8 node", ColorCodes.BLUE_OK))
        self._id_to_rmdx8_motor: dict[int, RMDx8Motor] = {}
        self.driver = rmd.CanDriver("can1")
        self._req_buffer: deque[tuple[int, String]] = deque(maxlen=1000)
        self._buffer_lock = Lock()
        self._target_velocities: dict[int, float] = {}
        self._smoothed_velocities: dict[int, float] = {}
        self._vel_lock = Lock()
        self.createRMDx8Motors()
        # Hardware testing
        self.create_timer(0.005, self._handleRequests)
        self.create_timer(_VEL_SMOOTH_DT, self._stepVelocities)

    def _createSubscriber(self, config: RMDx8MotorConfig) -> Subscription:
        can_id = config.can_id
        return self.create_subscription(
            std_msgs.msg.String,
            config.getInterfaceTopicName(),
            # We pass the create_request lambda with the
            # capture clause as the motor_id's can id
            # so we know who is making the requests
            lambda msg: self._createRequest(can_id, msg),
            1,
            callback_group=ReentrantCallbackGroup(),
        )

    def shutdownMotors(self) -> None:
        """
        Shutdown all motors
        """
        for motor in self._id_to_rmdx8_motor.values():
            motor.shutdownMotor()

    def addMotor(self, config: RMDx8MotorConfig) -> None:
        """
        Adds new rmdx8 motor to the motor dictionary
        """

        motor = RMDx8Motor(
            config,
            self.driver,
            self,
            lambda: self._createRequest(config.can_id, String(data=self._UPDATE_STATE)),
        )
        self._id_to_rmdx8_motor[config.can_id] = motor
        with self._vel_lock:
            self._target_velocities[config.can_id] = 0.0
            self._smoothed_velocities[config.can_id] = 0.0
        self._createSubscriber(config)

    def createRMDx8Motors(self) -> None:
        """
        Create all necessary RMDx8 motors and add them to the dictionary
        """

        for config in MotorConfigs.getAllMotors():
            if not isinstance(config, RMDx8MotorConfig):
                continue
            self.addMotor(config)

    def _createRequest(self, can_id: int, msg: String) -> None:
        """
        Add a request to the ring buffer for later dispatch
        """
        with self._buffer_lock:
            self._req_buffer.append((can_id, msg))

    def _handleRequests(self) -> None:
        """
        Drain the ring buffer and dispatch each command to the correct motor.
        Velocity commands are intercepted here and handed off to the smooth timer
        rather than applied directly, so deceleration takes ~1 second.
        """
        with self._buffer_lock:
            if not self._req_buffer:
                return
            can_id, msg = self._req_buffer.popleft()
        motor = self._id_to_rmdx8_motor.get(can_id)
        if motor is None:
            self.get_logger().error(f"Received request for motor {can_id} that doesnt exist")
            return
        if msg.data == self._UPDATE_STATE:
            motor.publishData()
            return

        run_settings = RMDX8RunSettings.fromJsonMsg(msg)

        if run_settings.set_stop:
            # Emergency stop: apply immediately and reset smooth state.
            with self._vel_lock:
                self._target_velocities[can_id] = 0.0
                self._smoothed_velocities[can_id] = 0.0
            motor.dataInCallback(msg)
            return

        if run_settings.velocity is not None:
            with self._vel_lock:
                self._target_velocities[can_id] = run_settings.velocity
            # Strip velocity so the smooth timer is the sole sender of velocity setpoints.
            stripped = RMDX8RunSettings(
                position=run_settings.position,
                velocity=None,
                velocity_limit=run_settings.velocity_limit,
                set_stop=False,
                current_pi=run_settings.current_pi,
                speed_pi=run_settings.speed_pi,
                position_pi=run_settings.position_pi,
                acceleration=run_settings.acceleration,
                acceleration_type=run_settings.acceleration_type,
                current=run_settings.current,
            )
            motor.dataInCallback(stripped.toMsg())
        else:
            motor.dataInCallback(msg)

    def _stepVelocities(self) -> None:
        """
        Advance each motor's smoothed velocity one step toward its target and
        send the result. Called at 50 Hz; the filter coefficient is chosen so
        the velocity reaches ~95% of any step change in approximately 1 second.
        """
        with self._vel_lock:
            updates: dict[int, tuple[float, float]] = {}
            for can_id in self._target_velocities:
                target = self._target_velocities[can_id]
                current = self._smoothed_velocities[can_id]
                stepped = current + (target - current) * _VEL_SMOOTH_ALPHA
                if abs(stepped) < _VEL_ZERO_THRESHOLD:
                    stepped = 0.0
                self._smoothed_velocities[can_id] = stepped
                updates[can_id] = (current, stepped)

        for can_id, (prev, stepped) in updates.items():
            if stepped == 0.0 and prev == 0.0:
                continue
            motor = self._id_to_rmdx8_motor.get(can_id)
            if motor is None:
                continue
            synthetic = RMDX8RunSettings(velocity=stepped, set_stop=False)
            motor.dataInCallback(synthetic.toMsg())

    def motorCount(self) -> int:
        """
        Returns the number of motors
        """
        return len(self._id_to_rmdx8_motor)


# Main function
def main(args: list[str] | None = None) -> None:
    """
    The entry point for RMDx8
    """

    rclpy.init(args=args)
    node = None
    try:
        node = RMDx8MotorManager()
        # Each motor has a timer + subscriber callback that can run concurrently,
        # so allocate 2 threads per motor with a minimum of 2 to avoid spin_once crashes.
        num_threads = max(2 * node.motorCount() + 2, 4)
        executor = MultiThreadedExecutor(num_threads=num_threads)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.shutdownMotors()
        rclpy.shutdown()


# If script is run directly, then create a RMDx8MotorManager object and run the main function
if __name__ == "__main__":
    main(sys.argv)
