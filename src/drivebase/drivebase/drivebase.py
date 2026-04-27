import math
import sys
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import Float32, String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs, RMDx8MotorConfig
from lib.interface.robot_interface import RobotInterface
from lib.motor_state.rmd_motor_state import RMDX8MotorState

REVS_TO_RADS = math.pi * 2


class Drivebase(Node):
    SPEED = 6.28 * 0.75
    ACCEL_TICK = SPEED / 10

    def __init__(self) -> None:
        super().__init__("drivebase")
        self.get_logger().info(colorStr("Launching drivebase node", ColorCodes.BLUE_OK))
        self.bot_interface = RobotInterface(self)

        self.left_subscription = self.create_subscription(
            Float32, "move_left_drivebase_side_message", self.moveLeftSide, 10
        )
        self.right_subscription = self.create_subscription(
            Float32, "move_right_drivebase_side_message", self.moveRightSide, 10
        )
        self._last_velocities: dict[int, deque[String]] = {}
        self._velo_sub: list[Subscription] = []
        self._left_target_velo: float = 0.0
        self._right_target_velo: float = 0.0
        self._left_motors = [
            MotorConfigs.MID_LEFT_DRIVE_MOTOR,
            MotorConfigs.FRONT_LEFT_DRIVE_MOTOR,
            MotorConfigs.REAR_LEFT_DRIVE_MOTOR,
        ]
        self._right_motors = [
            MotorConfigs.MID_RIGHT_DRIVE_MOTOR,
            MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR,
            MotorConfigs.REAR_RIGHT_DRIVE_MOTOR,
        ]
        motors = self._left_motors + self._right_motors
        for config in motors:
            if not isinstance(config, RMDx8MotorConfig):
                continue
            if config.can_id is None:
                self.get_logger().error("Invalid can motor found")
                continue
            self._last_velocities[config.can_id] = deque(maxlen=10)
            self._velo_sub.append(
                self.create_subscription(
                    String,
                    config.getCanTopicName(),
                    lambda msg, cid=config.can_id: self._veloCallback(cid, msg),
                    10,
                )
            )
        self._last_received: float = time.time()
        self._vel_timer = self.create_timer(1.0 / 20.0, self._veloTick)
        self.create_timer(0.05, self._stopMotorsIfStale)

    def _veloTick(self) -> None:
        def modifier(curr_speed: float, target_speed: float) -> float:
            if curr_speed < target_speed:
                return min(curr_speed + self.ACCEL_TICK, target_speed)
            else:
                return max(curr_speed - self.ACCEL_TICK, target_speed)

        for motor in self._left_motors:
            if motor.can_id is None:
                self.get_logger().error("Invalid motor")
                continue
            if not self._last_velocities[motor.can_id]:
                continue
            state = RMDX8MotorState.fromJsonMsg(self._last_velocities[motor.can_id][-1])
            motor_speed = (state.velocity or 0.0) * REVS_TO_RADS
            self.bot_interface.runMotorSpeed(motor, modifier(motor_speed, self._left_target_velo))

        for motor in self._right_motors:
            if motor.can_id is None:
                self.get_logger().error("Invalid motor")
                continue
            if not self._last_velocities[motor.can_id]:
                continue
            state = RMDX8MotorState.fromJsonMsg(self._last_velocities[motor.can_id][-1])
            motor_speed = (state.velocity or 0.0) * REVS_TO_RADS
            self.bot_interface.runMotorSpeed(motor, modifier(motor_speed, self._right_target_velo))

    def _veloCallback(self, can_id: int, msg: String) -> None:
        self._last_velocities[can_id].append(msg)

    def _stopMotorsIfStale(self) -> None:
        if time.time() - self._last_received > 0.25:
            self.get_logger().info("STOPPING ALL MOTORS")
            for motor in self._left_motors + self._right_motors:
                self.bot_interface.stopMotor(motor)

    def moveLeftSide(self, msg: Float32) -> None:
        self._last_received = time.time()
        self._left_target_velo = msg.data * self.SPEED * -1

    def moveRightSide(self, msg: Float32) -> None:
        self._last_received = time.time()
        self._right_target_velo = msg.data * self.SPEED

    def turnLeft(self, msg: Float32) -> None:
        self.moveLeftSide(msg)
        self.moveRightSide(-msg)

    def turnRight(self, msg: Float32) -> None:
        self.moveLeftSide(-msg)
        self.moveRightSide(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        drivebase = Drivebase()
        rclpy.spin(drivebase)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        drivebase.get_logger().info(colorStr("Shutting down drivebase", ColorCodes.BLUE_OK))
        drivebase.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
