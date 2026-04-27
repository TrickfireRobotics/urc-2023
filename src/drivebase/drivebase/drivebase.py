import sys
from collections import deque

import rclpy
from rclpy import subscription
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import Float32, String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs, RMDx8MotorConfig
from lib.interface.robot_interface import RobotInterface
from lib.motor_state.rmd_motor_state import RMDX8MotorState


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
        self._left_target_velo: Float32 = 0.0
        self._right_target_velo: Float32 = 0.0
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
            # Store the last seconds worth of velocity commands for each motor
            self._last_velocities[config.can_id] = deque(maxlen=10)
            self._velo_sub.append(
                self.create_subscription(
                    String,
                    config.getCanTopicName(),
                    lambda msg, cid=config.can_id: self._veloCallback(cid, msg),
                    10,
                )
            )
        self._vel_timer = self.create_timer(1.0 / 20.0, self._veloTick())

    def _veloTick(self) -> int:
        def modifier(curr_speed: Float32, target_speed: Float32) -> Float32:
            if target_speed < curr_speed:
                return min(curr_speed + self.ACCEL_TICK, self.SPEED)
            else:
                return max(curr_speed - self.ACCEL_TICK, self.SPEED * -1)

        for motor in self._left_motors:
            if motor.can_id is None:
                self.get_logger().error("Invalid motor")
                continue

            msg = self._last_velocities[motor.can_id][-1]
            msg = RMDX8MotorState.fromJsonMsg(msg)
            motor_speed = msg.velocity
            self.bot_interface.runMotorSpeed(
                motor, modifier(motor_speed, modifier(motor_speed, self._left_target_velo))
            )

        for motor in self._right_motors:
            if motor.can_id is None:
                self.get_logger().error("Invalid motor")
                continue

            msg = self._last_velocities[motor.can_id][-1]
            msg = RMDX8MotorState.fromJsonMsg(msg)
            motor_speed = msg.velocity
            self.bot_interface.runMotorSpeed(
                motor, modifier(motor_speed, modifier(motor_speed, self._right_target_velo))
            )

        return 0

    def _veloCallback(self, can_id: int, msg: String) -> None:
        self._last_velocities[can_id].append(msg)

    def moveLeftSide(self, msg: Float32) -> None:
        self._left_target_velo = msg.data * self.SPEED * -1

    def moveRightSide(self, msg: Float32) -> None:
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
        rclpy.spin(drivebase)  # prints callbacks
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        drivebase.get_logger().info(colorStr("Shutting down drivebase", ColorCodes.BLUE_OK))
        drivebase.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
