"""
This file contains the InverseKinematics arm control type, as well as supporting constants
"""

import math
import os
import time
from typing import Callable

import roboticstoolbox as rtb
import spatialmath as sm
from rclpy.node import Node
from roboticstoolbox import ERobot
from std_msgs.msg import Float32

from lib.configs import MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface

# constants
REVS_TO_RADIANS = math.pi * 2.0
DEGREES_TO_RADIANS = math.pi / 180.0
RADIANS_TO_DEGREES = 1.0 / DEGREES_TO_RADIANS
RADIANS_TO_REVS = 1 / (math.pi * 2.0)


class InverseKinematics:
    """
    Represents an arm control type that uses InverseKinematics.
    """

    def __init__(self, ros_node: Node, interface: RobotInterface, _: RobotInfo):

        self._can_send = False
        self._ros_node = ros_node
        self._interface = interface

        # Initialise model
        self.viator = ERobot.URDF(os.path.join(os.path.dirname(__file__), "../resource/arm.urdf"))

        # Initialize target
        self.target: sm.SE3 = self.viator.fkine(self.viator.q)
        self._ros_node.get_logger().info("Starting pos: " + str(self.target.t))
        self.last_target: sm.SE3 = sm.SE3(self.target)

        # Make our solver
        self.solver = rtb.IK_LM()

        # Initialize subs
        self._last_received: list[float] = []
        ros_node.create_subscription(
            Float32, "shoulder_up", self._createSub(sm.SE3.Trans(0.1, 0, 0)), 10
        )
        ros_node.create_subscription(
            Float32, "shoulder_down", self._createSub(sm.SE3.Trans(-0.1, 0, 0)), 10
        )
        ros_node.create_subscription(
            Float32, "elbow_up", self._createSub(sm.SE3.Trans(0, 0, 0.1)), 10
        )
        ros_node.create_subscription(
            Float32, "elbow_down", self._createSub(sm.SE3.Trans(0, 0, -0.1)), 10
        )
        ros_node.create_subscription(
            Float32, "turntable_cw", self._createSub(sm.SE3.Trans(0, 0.1, 0)), 10
        )
        ros_node.create_subscription(
            Float32, "turntable_ccw", self._createSub(sm.SE3.Trans(0, -0.1, 0)), 10
        )
        ros_node.create_subscription(Float32, "e_stop", self._eStop, 10)

    @property
    def can_send(self) -> bool:
        """
        True if inverse_kinematics should be the one controlling the arm, else False.
        """
        return self._can_send

    @can_send.setter
    def can_send(self, val: bool) -> None:
        if not val:
            self.stopAllMotors()
        self._can_send = val

    def runArmToTarget(self) -> None:
        """
        Makes the arm go to the target position.

        Note, if the position is unreachable, the arm will not move.
        """
        if not self.can_send:
            return

        # Solve for the target position
        sol = self.solver.solve(self.viator.ets(), self.target)
        if not sol.success:
            self._ros_node.get_logger().warning(
                f"IK Solver failed: {sol.reason}\ntarget: {self.target.t}"
            )
            self.target = self.last_target
            return

        self._ros_node.get_logger().info(
            f"target: {self.target.t}\nturntable: {sol.q[0]} ({sol.q[0] * RADIANS_TO_REVS})"
            + f"\nshoulder: {sol.q[1]} ({sol.q[1] * RADIANS_TO_REVS})"
        )

        # Make motors move to the positions that were found
        self._interface.runMotorPosition(
            MotorConfigs.ARM_TURNTABLE_MOTOR, sol.q[0] * RADIANS_TO_REVS
        )
        self._interface.runMotorPosition(
            MotorConfigs.ARM_SHOULDER_MOTOR, -sol.q[1] * RADIANS_TO_REVS
        )

    def stopAllMotors(self) -> None:
        """
        Stops, not diables, all motors in the arm
        """
        self._interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)
        self._interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)
        self._interface.stopMotor(MotorConfigs.ARM_ELBOW_MOTOR)

    def _createSub(self, delta: sm.SE3) -> Callable[[Float32], None]:
        # Use list as way to track when this sub was last received
        last_receieved_index = len(self._last_received) - 1
        self._last_received.append(time.time())

        def sub(msg: Float32) -> None:
            # Don't care about 0's
            if msg.data == 0:
                return

            # Calculate time since last received sub
            time_delta = time.time() - self._last_received[last_receieved_index]
            self._ros_node.get_logger().info(
                str(self.solver.solve(self.viator.ets(), self.target).success)
            )
            # Calculate new target
            self.last_target = sm.SE3(self.target)
            self.target *= sm.SE3(delta, check=False)
            self._ros_node.get_logger().debug(
                f"delta: {delta.t}\ndata: {msg.data}\ntime_delta: {time_delta}"
            )

            # Move arm to new target
            self.runArmToTarget()

            # Update when this sub was last received
            self._last_received[last_receieved_index] = time.time()

        return sub

    def _eStop(self, _: Float32) -> None:
        self.stopAllMotors()
        self._ros_node.get_logger().info("emergency stopping")
