import sys
import time
from typing import Callable, Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import Bool, String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusMotorState, MoteusRunSettings

NODE_NAME = "stuck_detection_node"


class StuckDetectionNode(Node):
    """
    Determines whether the rover is currently "stuck".
    """

    # Initialization

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.slip_motor_configs = [
            MotorConfigs.FRONT_LEFT_DRIVE_MOTOR,
            MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR,
            MotorConfigs.REAR_LEFT_DRIVE_MOTOR,
            MotorConfigs.REAR_RIGHT_DRIVE_MOTOR,
        ]

        for config in self.slip_motor_configs:
            self.subscribe_to_motor(config)

        # TODO: Add lifetimes to the values
        self.target_states: Dict[int, MoteusRunSettings] = {}
        self.current_states: Dict[int, MoteusMotorState] = {}

        self.effort_stuck_start: Optional[float] = None

        self.stuck_publisher = self.create_publisher(Bool, "/stuck", 10)

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

    def calculate_per_motor_effort(self) -> bool:
        RMX8_25_RATED_TORQUE = 10  # N/m
        FREE_SPINNING_THRESH = (0.2, 0.9)  # <t, >v
        STUCK_THRESH = (0.5, 0.2)  # >t, <v
        STUCK_WHEEL_MIN_COUNT = 3
        DURATION = 1.0

        stuck_count = 0
        free_spinning_count = 0

        for config in self.slip_motor_configs:
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
                stuck_count += 1

        now = time.time()
        # TODO: Differentiate free-spinning vs. stuck
        stuck = stuck_count + free_spinning_count > STUCK_WHEEL_MIN_COUNT
        if stuck:
            if self.effort_stuck_start is not None:
                if now - self.effort_stuck_start > DURATION:
                    return True
            else:
                self.effort_stuck_start = now
        else:
            self.effort_stuck_start = None

        return False

    # Update

    def update(self) -> None:
        effort_stuck = self.calculate_per_motor_effort()

        # TODO: Use custom message with constants for reasons
        if effort_stuck:
            self.stuck_publisher.publish(Bool(data=True))
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
