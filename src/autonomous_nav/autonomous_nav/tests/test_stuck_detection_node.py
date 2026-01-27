import sys
import threading
import time
from pathlib import Path
from typing import Callable, Generator

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8

# TODO: This is the only way I've gotten `pytest` to work.
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent))
from stuck_detection_node import DRIVE_MOTOR_CONFIGS, StuckDetectionNode, StuckReasons

from lib.configs import MotorConfigs
from lib.moteus_motor_state import MoteusMotorState, MoteusRunSettings

# -- Fixtures -- #


@pytest.fixture(scope="session", autouse=True)
def ros2_init_shutdown() -> Generator:
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def spun_node() -> Generator[StuckDetectionNode, None, None]:
    node = StuckDetectionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    running = True

    def spin() -> None:
        while running:
            executor.spin_once(timeout_sec=0.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    try:
        yield node
    finally:
        locals()["running"] = False
        running = False
        thread.join(timeout=1)
        executor.remove_node(node)
        node.destroy_node()


# -- Constants -- #

MOTOR_CAN_IDS = [config.can_id for config in DRIVE_MOTOR_CONFIGS]

MAX_DETECTION_LATENCY = 2
"""The maximum latency between data publishing and stuck triggering. Dependent on `StuckReason.stuck_duration`."""


# -- Tests -- #


class TestStuckDetectionNode:
    def test_stuck(self, spun_node: StuckDetectionNode) -> None:
        self.is_stuck = False
        self.reason = 0

        spun_node.create_subscription(Bool, "/stuck_detection/is_stuck", self.is_stuck_callback, 10)
        spun_node.create_subscription(UInt8, "/stuck_detection/reason", self.reason_callback, 10)

        self.publishers = {
            config.can_id: [
                spun_node.create_publisher(String, config.getInterfaceTopicName(), 10),
                spun_node.create_publisher(String, config.getCanTopicName(), 10),
            ]
            for config in DRIVE_MOTOR_CONFIGS
        }

        print("")

        self._test(
            "At Speed, Rated Torque, All Wheels = Not Stuck",
            MoteusRunSettings(velocity=10),
            lambda id: MoteusMotorState(id, velocity=10, torque=10),
            should_be_stuck=False,
            min_latency=1,
        )

        self._test(
            "At Speed, Low Torque (10% rated), All Wheels = Stuck, Free Spinning",
            MoteusRunSettings(velocity=10),
            lambda id: MoteusMotorState(id, velocity=10, torque=1),
            should_be_stuck=True,
            reasons_mask=StuckReasons.FREE_SPINNING.mask,
        )

        self._test(
            "At Speed, Low Torque (10% rated), 2 Wheels = Not Stuck",
            MoteusRunSettings(velocity=10),
            lambda id: MoteusMotorState(id, velocity=10, torque=1),
            ids=[
                MotorConfigs.FRONT_LEFT_DRIVE_MOTOR.can_id,
                MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR.can_id,
            ],
            should_be_stuck=False,
            min_latency=1,
        )

        self._test(
            "Low Speed, High Torque (75% rated), All Wheels = Stuck, Can't Spin",
            MoteusRunSettings(velocity=10),
            lambda id: MoteusMotorState(id, velocity=1, torque=7.5),
            should_be_stuck=True,
            reasons_mask=StuckReasons.WHEELS_CANT_SPIN.mask,
        )

    def is_stuck_callback(self, is_stuck: Bool) -> None:
        self.is_stuck = is_stuck.data

    def reason_callback(self, reason: UInt8) -> None:
        self.reason = reason.data

    def _test(
        self,
        desc: str,
        target: MoteusRunSettings,
        state_fn: Callable[[int], MoteusMotorState],
        ids: list[int] = MOTOR_CAN_IDS,
        min_latency: float = 0,
        max_latency: float = MAX_DETECTION_LATENCY,
        should_be_stuck: bool = True,
        reasons_mask: int = 0,
    ) -> None:
        for id in ids:
            [tx, rx] = self.publishers[id]
            tx.publish(target.toMsg())
            rx.publish(state_fn(id).toMsg())

        passed = False
        start = time.time()
        now = 0.0
        while (not (passed or now > max_latency)) or now < min_latency:
            passed = (self.is_stuck == should_be_stuck) and (
                self.reason & reasons_mask == reasons_mask
            )
            now = time.time() - start

        assert passed, desc + f" | is_stuck={self.is_stuck}, reason={self.reason}"
        print("PASSED: " + desc)
        self.reset()

    def reset(self) -> None:
        for [id, [tx, rx]] in self.publishers.items():
            tx.publish(MoteusRunSettings(velocity=0).toMsg())
            rx.publish(MoteusMotorState(id, velocity=0).toMsg())
