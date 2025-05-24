"""
This file contains the InverseKinematics arm control type, as well as supporting constants
"""

import asyncio
import math
import os
import time
from typing import Callable

import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import websockets
from rclpy.node import Node
from roboticstoolbox import ERobot
from std_msgs.msg import Float32
from swift import Swift
from swift.SwiftRoute import SwiftSocket

from lib.configs import MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface

# constants
REVS_TO_RADIANS = math.pi * 2.0
DEGREES_TO_RADIANS = math.pi / 180.0
RADIANS_TO_DEGREES = 1.0 / DEGREES_TO_RADIANS
RADIANS_TO_REVS = 1 / (math.pi * 2.0)


# websockets really didn't want to run with localhost, so i added 127.0.0.1
def socket_init(self, outq, inq, run) -> None:  # type: ignore

    self.pcs = set()
    self.run = run
    self.outq = outq
    self.inq = inq
    self.USERS = set()
    self.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(self.loop)

    started = False

    port = 53000
    while not started and port < 62000:
        try:
            start_server = websockets.serve(self.serve, "127.0.0.1", port)
            self.loop.run_until_complete(start_server)
            started = True
        except OSError:
            port += 1

    self.inq.put(port)
    self.loop.run_forever()


SwiftSocket.__init__ = socket_init


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
            Float32, "shoulder_up", self._createSub(lambda mul: sm.SE3.Trans(0.1 * mul, 0, 0)), 10
        )
        ros_node.create_subscription(
            Float32,
            "shoulder_down",
            self._createSub(lambda mul: sm.SE3.Trans(-0.1 * mul, 0, 0)),
            10,
        )
        ros_node.create_subscription(
            Float32, "elbow_up", self._createSub(lambda mul: sm.SE3.Trans(0, 0, 0.1 * mul)), 10
        )
        ros_node.create_subscription(
            Float32, "elbow_down", self._createSub(lambda mul: sm.SE3.Trans(0, 0, -0.1 * mul)), 10
        )
        ros_node.create_subscription(
            Float32,
            "turntable_cw",
            self._createSub(lambda mul: sm.SE3.Rz(25 * mul, unit="deg"), pre=True),
            10,
        )
        ros_node.create_subscription(
            Float32,
            "turntable_ccw",
            self._createSub(lambda mul: sm.SE3.Rz(-25 * mul, unit="deg"), pre=True),
            10,
        )
        ros_node.create_subscription(Float32, "e_stop", self._eStop, 10)

        self._env = Swift()
        self._env.launch(realtime=True, comms="websocket")
        self._env.add(self.viator)
        self._target_axes = sg.Axes(0.1, pose=self.target)
        self._env.add(self._target_axes)

        self._ros_node.create_timer(0.05, self._env.step)

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
        self.viator.q = sol.q
        self._target_axes.T = self.target
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

    def _createSub(
        self, delta: Callable[[float], sm.SE3], pre: bool = False
    ) -> Callable[[Float32], None]:
        # Use list as way to track when this sub was last received
        last_receieved_index = len(self._last_received) - 1
        self._last_received.append(time.time())

        def sub(msg: Float32) -> None:
            # Don't care about 0's
            if msg.data == 0:
                return

            # Calculate time since last received sub
            time_delta = time.time() - self._last_received[last_receieved_index]

            # Reduce magnitude of delta by mult
            mult = min(time_delta, 0.1) * msg.data

            # Calculate new target
            self.last_target = sm.SE3(self.target)
            if pre:
                self.target = delta(mult) * self.target
            else:
                self.target *= delta(mult)
            self._ros_node.get_logger().debug(
                f"delta: {delta(mult).t}\ndata: {msg.data}\ntime_delta: {time_delta}"
            )

            # Move arm to new target
            self.runArmToTarget()

            # Update when this sub was last received
            self._last_received[last_receieved_index] = time.time()

        return sub

    def _eStop(self, _: Float32) -> None:
        self.stopAllMotors()
        self._ros_node.get_logger().info("emergency stopping")
