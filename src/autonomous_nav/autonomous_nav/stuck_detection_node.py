import sys
import time
from dataclasses import dataclass
from typing import Callable, Dict, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import Bool, String, UInt8

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusMotorState, MoteusRunSettings

NODE_NAME = "stuck_detection_node"
DRIVE_MOTOR_CONFIGS = [
    MotorConfigs.FRONT_LEFT_DRIVE_MOTOR,
    MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR,
    MotorConfigs.MID_LEFT_DRIVE_MOTOR,
    MotorConfigs.MID_RIGHT_DRIVE_MOTOR,
    MotorConfigs.REAR_LEFT_DRIVE_MOTOR,
    MotorConfigs.REAR_RIGHT_DRIVE_MOTOR,
]


@dataclass
class StuckReason:
    ros_message: str
    """ The message sent to the ROS topic. """
    stuck_duration: float
    """ How many seconds tripped before marked as stuck. """
    mask: int
    """ How to encode/decode this reason. """

    def __hash__(self) -> int:
        return self.mask


class StuckReasons:
    """Reasons for why we are stuck."""

    WHEELS_CANT_SPIN = StuckReason("wheels-cant-spin", 1.0, 1)
    FREE_SPINNING = StuckReason("free-spinning", 1.0, 1 << 1)


class StuckDetectionNode(Node):
    """
    Determines whether the rover is currently "stuck".

    Publications:
     - /stuck_detection/is_stuck [Bool]
     - /stuck_detection/reason [UInt8]
        - Reasons are encoded by a bitmask (see StuckReasons for usage).
    """

    # Initialization

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        for config in DRIVE_MOTOR_CONFIGS:
            self.subscribe_to_motor(config)

        # TODO: Add lifetimes to the values
        self.target_states: Dict[int, MoteusRunSettings] = {}
        self.current_states: Dict[int, MoteusMotorState] = {}

        self.stuck_start: Dict[StuckReason, float] = {}

        self.stuck_publisher = self.create_publisher(Bool, "/stuck_detection/is_stuck", 10)
        self.stuck_reason_publisher = self.create_publisher(UInt8, "/stuck_detection/reason", 10)

        self.timer = self.create_timer(0.1, self.update)

    def subscribe_to_motor(
        self, motor_config: MoteusMotorConfig
    ) -> Tuple[Subscription, Subscription]:
        """
        Subscribes to the motor's target state and current state.
        """

        def get_target_callback() -> Callable[[String], None]:
            return lambda msg: self.motor_target_state_callback(
                motor_config.can_id, MoteusRunSettings.fromJsonMsg(msg)
            )

        def get_current_callback() -> Callable[[String], None]:
            return lambda msg: self.motor_current_state_callback(
                motor_config.can_id, MoteusMotorState.fromJsonMsg(msg)
            )

        return (
            self.create_subscription(
                String,
                motor_config.getInterfaceTopicName(),
                get_target_callback(),
                10,
            ),
            self.create_subscription(
                String,
                motor_config.getCanTopicName(),
                get_current_callback(),
                10,
            ),
        )

    def motor_current_state_callback(self, can_id: int, state: MoteusMotorState) -> None:
        self.current_states[can_id] = state

    def motor_target_state_callback(self, can_id: int, settings: MoteusRunSettings) -> None:
        self.target_states[can_id] = settings

    # Detection

    def update_stuck_reason(self, reason: StuckReason, is_active: bool) -> bool:
        now = time.time()
        if is_active:
            if reason in self.stuck_start:
                return now - self.stuck_start[reason] > reason.stuck_duration
            else:
                self.stuck_start[reason] = now
        else:
            self.stuck_start.pop(reason, 0.0)

        return False

    def calculate_per_motor_effort(self) -> list[StuckReason]:
        RMX8_25_RATED_TORQUE = 10  # N/m
        FREE_SPINNING_THRESH = (0.2, 0.9)  # <t, >v
        STUCK_THRESH = (0.5, 0.2)  # >t, <v
        STUCK_WHEEL_MIN_COUNT = 3

        cant_spin_count = 0
        free_spinning_count = 0

        for config in DRIVE_MOTOR_CONFIGS:
            target = self.target_states.get(config.can_id)
            state = self.current_states.get(config.can_id)

            if target is None or state is None:
                continue

            if state.velocity is None or target.velocity is None:
                continue

            if state.torque is None:
                # TODO: Fallback to q_current
                continue

            if abs(target.velocity) < 1e-3:
                continue

            v_norm = abs(state.velocity / target.velocity)
            v_norm = min(v_norm, 1.0)

            t_norm = abs(state.torque) / RMX8_25_RATED_TORQUE

            if t_norm < FREE_SPINNING_THRESH[0] and v_norm > FREE_SPINNING_THRESH[1]:
                free_spinning_count += 1

            if t_norm > STUCK_THRESH[0] and v_norm < STUCK_THRESH[1]:
                cant_spin_count += 1

        stuck = []
        if self.update_stuck_reason(
            StuckReasons.WHEELS_CANT_SPIN, cant_spin_count >= STUCK_WHEEL_MIN_COUNT
        ):
            stuck.append(StuckReasons.WHEELS_CANT_SPIN)
        if self.update_stuck_reason(
            StuckReasons.FREE_SPINNING, free_spinning_count >= STUCK_WHEEL_MIN_COUNT
        ):
            stuck.append(StuckReasons.FREE_SPINNING)
        return stuck

    # Update

    def update(self) -> None:
        stuck = self.calculate_per_motor_effort()

        if len(stuck) > 0:
            self.stuck_publisher.publish(Bool(data=True))
            encoded = 0
            for reason in stuck:
                encoded |= reason.mask
            self.stuck_reason_publisher.publish(UInt8(data=encoded))
        else:
            self.stuck_publisher.publish(Bool(data=False))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    stuck_detection_node = None
    try:
        stuck_detection_node = StuckDetectionNode()
        rclpy.spin(stuck_detection_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if stuck_detection_node is not None:
            stuck_detection_node.get_logger().info(
                colorStr(f"Shutting down {NODE_NAME}", ColorCodes.BLUE_OK)
            )
    finally:
        if stuck_detection_node is not None:
            stuck_detection_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
