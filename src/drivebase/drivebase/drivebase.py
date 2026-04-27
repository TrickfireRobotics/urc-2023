import math
import sys
import threading
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
    ACCEL_TICK = SPEED / 100
    TICK_RATE = 0.01

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

        self._velo_lock = threading.Lock()
        self._target_lock = threading.Lock()
        self._can_lock = threading.Lock()
        self._running = True

        self._left_thread = threading.Thread(
            target=self._leftMotorLoop, daemon=True, name="left_motor"
        )
        self._right_thread = threading.Thread(
            target=self._rightMotorLoop, daemon=True, name="right_motor"
        )
        self._left_thread.start()
        self._right_thread.start()

    def _modifier(self, curr_speed: float, target_speed: float) -> float:
        if curr_speed < target_speed:
            return min(curr_speed + self.ACCEL_TICK, target_speed)
        else:
            return max(curr_speed - self.ACCEL_TICK, target_speed)

    def _leftMotorLoop(self) -> None:
        while self._running:
            with self._target_lock:
                target = self._left_target_velo
            for motor in self._left_motors:
                if motor.can_id is None:
                    continue
                with self._velo_lock:
                    if not self._last_velocities[motor.can_id]:
                        continue
                    last_msg = self._last_velocities[motor.can_id][-1]
                state = RMDX8MotorState.fromJsonMsg(last_msg)
                motor_speed = (state.velocity or 0.0) * REVS_TO_RADS
                run_speed = self._modifier(motor_speed, target)
                while run_speed != target:
                    with self._can_lock:
                        self.bot_interface.runMotorSpeed(motor, run_speed)
                    run_speed = self._modifier(run_speed, target)
            time.sleep(self.TICK_RATE)

    def _rightMotorLoop(self) -> None:
        while self._running:
            with self._target_lock:
                target = self._right_target_velo
            for motor in self._right_motors:
                if motor.can_id is None:
                    continue
                with self._velo_lock:
                    if not self._last_velocities[motor.can_id]:
                        continue
                    last_msg = self._last_velocities[motor.can_id][-1]
                state = RMDX8MotorState.fromJsonMsg(last_msg)
                motor_speed = (state.velocity or 0.0) * REVS_TO_RADS
                run_speed = self._modifier(motor_speed, target)
                while run_speed != target:
                    with self._can_lock:
                        self.bot_interface.runMotorSpeed(motor, run_speed)
                    run_speed = self._modifier(run_speed, target)
            time.sleep(self.TICK_RATE)

    def _veloCallback(self, can_id: int, msg: String) -> None:
        with self._velo_lock:
            self._last_velocities[can_id].append(msg)

    def moveLeftSide(self, msg: Float32) -> None:
        with self._target_lock:
            self._left_target_velo = msg.data * self.SPEED * -1

    def moveRightSide(self, msg: Float32) -> None:
        with self._target_lock:
            self._right_target_velo = msg.data * self.SPEED

    def turnLeft(self, msg: Float32) -> None:
        self.moveLeftSide(msg)
        self.moveRightSide(-msg)

    def turnRight(self, msg: Float32) -> None:
        self.moveLeftSide(-msg)
        self.moveRightSide(msg)

    def destroy_node(self) -> None:
        self._running = False
        self._left_thread.join(timeout=1.0)
        self._right_thread.join(timeout=1.0)
        super().destroy_node()


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
