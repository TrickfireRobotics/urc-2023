"""
This file contains the InverseKinematics arm control type, as well as supporting constants
"""

import asyncio
import math
import os
import queue
import threading
import time
from typing import Callable

import can
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import websockets.asyncio.server
from rclpy.node import Node
from roboticstoolbox import ERobot
from std_msgs.msg import Float32
from swift import Swift
from swift.SwiftRoute import SwiftSocket

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface


# websockets really didn't want to run with localhost, so i added 127.0.0.1
# websockets needs an asyncio context so i impled smth based on this pr:
# https://github.com/jhavl/swift/pull/58/files
def socket_init(self, outq, inq, run) -> None:  # type: ignore

    self.pcs = set()
    self.run = run
    self.outq = outq
    self.inq = inq
    self.USERS = set()
    self.loop = asyncio.new_event_loop()
    asyncio.set_event_loop(self.loop)

    async def _startServer() -> None:
        # There is an extra arg that swift expects that isn't passed in by websockets anymore. It
        # isn't used by swift, so lets just pass in a dummy value
        async def _serve(websocket: websockets.asyncio.server.ServerConnection) -> None:
            return await self.serve(websocket, "")

        port = 53000
        while port < 62000:
            try:
                async with websockets.asyncio.server.serve(_serve, "127.0.0.1", port) as server:
                    self.inq.put(port)
                    await server.serve_forever()
            except OSError:
                port += 1
        raise OSError("No ports available to run the websocket server on")

    self.loop.create_task(_startServer())
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
        self._target_axes = sg.Axes(0.1, pose=self.target)
        threading.Thread(target=self._startSwift).start()

        self.turntableMotorPos = 0
        self.turntableMotorSpeed = 0
        self.turntableMotorFaultCode = 0

        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            # send zero velocity
            msg = can.Message(
                arbitration_id=0x00000305,  # velocity loop
                data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=True,
            )
            try:
                bus.send(msg)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")
        self.updateTurntableMotorState()

    def updateTurntableMotorState(self) -> None:
        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            for msg in bus:
                self._ros_node.get_logger().info(f"{msg.arbitration_id:X}: {msg.data}")
                if msg.arbitration_id == 0x00002905:
                    self.turntableMotorPos = int(
                        self.uint8_to_bin(msg.data[0]) + self.uint8_to_bin(msg.data[1]), 2
                    )
                    self._ros_node.get_logger().info(
                        f"Turntable motor position: {self.turntableMotorPos}"
                    )
                    self.turntableMotorSpeed = int(
                        self.uint8_to_bin(msg.data[2]) + self.uint8_to_bin(msg.data[3]), 2
                    )
                    self._ros_node.get_logger().info(
                        f"Turntable motor speed {self.turntableMotorSpeed}"
                    )
                    self.turntableMotorFaultCode = msg.data[7]
                    if self.turntableMotorFaultCode != 0:
                        self._ros_node.get_logger().info(
                            f"TURNTABLE FAULT: {self.turntableMotorFaultCode}"
                        )

    def uint8_to_bin(self, value: int) -> str:
        if 0 <= value <= 255:  # Ensure it's within uint8 range
            return format(value, "08b")  # Format as an 8-bit binary string
        else:
            raise ValueError("Value must be between 0 and 255")

    def _startSwift(self) -> None:
        try:
            self._env.launch(realtime=True, comms="websocket")
            self._env.add(self.viator)
            self._env.add(self._target_axes)

            self._ros_node.create_timer(0.05, self._env.step)
        except queue.Empty:
            self._ros_node.get_logger().warn(
                colorStr("Swift client failed to start", ColorCodes.WARNING_YELLOW)
            )

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
            f"target: {self.target.t}\nturntable: {sol.q[0]}" + f"\nshoulder: {sol.q[1]}"
        )

        # Make motors move to the positions that were found
        # solver operates in radians, turntable operates in degrees
        # todo this guy not implemented yet
        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            msg = can.Message(
                arbitration_id=0x00000405,
                data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=True,
            )
            try:
                bus.send(msg)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")
        self._interface.runMotorPosition(MotorConfigs.ARM_TURNTABLE_MOTOR, sol.q[0])
        self._interface.runMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR, -sol.q[1])

    def stopAllMotors(self) -> None:
        """
        Stops, not diables, all motors in the arm
        """
        # self._interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)
        # self._interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)
        # self._interface.stopMotor(MotorConfigs.ARM_ELBOW_MOTOR)

        # give zero velocity to turntable motor
        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            msg = can.Message(
                arbitration_id=0x00000305,
                data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=True,
            )
            try:
                bus.send(msg)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")

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
