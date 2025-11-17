import sys
import threading
import time
from pathlib import Path
from typing import Generator

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

# TODO: This is the only way I've gotten `pytest` to work.
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent))
from stuck_detection_node import DRIVE_MOTOR_CONFIGS, StuckDetectionNode

from lib.moteus_motor_state import MoteusMotorState, MoteusRunSettings


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


class TestStuckDetectionNode:
    def test_stuck(self, spun_node: StuckDetectionNode) -> None:
        self.passed = False
        spun_node.create_subscription(Bool, "/stuck_detection/is_stuck", self.is_stuck_callback, 10)

        for config in DRIVE_MOTOR_CONFIGS:
            target_pub = spun_node.create_publisher(String, config.getInterfaceTopicName(), 10)
            target_pub.publish(MoteusRunSettings(velocity=100).toMsg())
            state_pub = spun_node.create_publisher(String, config.getCanTopicName(), 10)
            state_pub.publish(MoteusMotorState(config.can_id, velocity=100, torque=1).toMsg())

        start = time.time()
        while not self.passed and (time.time() - start) < 2:
            pass

        assert self.passed, "Node should have been reporting stuck but was " + str(self.passed)

    def is_stuck_callback(self, is_stuck: Bool) -> None:
        self.passed |= is_stuck.data

    def test_stuck2(self, spun_node: Node) -> None:
        pass
