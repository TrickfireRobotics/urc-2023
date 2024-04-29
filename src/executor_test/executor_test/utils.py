from rclpy.executors import Executor
from concurrent.futures import ThreadPoolExecutor
from rclpy.node import Node
from std_msgs.msg import String
import os

"""
This class is useful for the camera service and topics.
Service to shut down topics should be put as the priority node
and all camera publishers should be low priority.
Service gets its own thread and publishers get more
"""
class CameraExecutor(Executor):
    def __init__(self):
        super().__init__()
        self.high_priority_nodes = set()
        self.hp_executor = ThreadPoolExecutor(max_workers = 1)
        self.lp_executor = ThreadPoolExecutor(max_workers = os.cpu_count() or 4)

    def add_hp_node(self, node):
        self.high_priority_nodes.add(node)
        self.add_node(node) # inherited
    
    # Function override.
    # Executes the callbacks
    def spin_once(self, timeout_sec=None):
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            if node in self.high_priority_nodes:
                self.hp_executor.submit(handler)
            else:
                self.lp_executor.submit(handler)


class CameraNode(Node):
    def __init__(self, topic_id: int):
        super().__init_("executor_camera_test_node")

        self.i = 0
        self.pub = self.create_publisher(String, '/exec_frames' + str(topic_id) + 10)

        self.timer = self.create_timer(2, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: {0}".format(self.i)
        self.i += 1
        self.pub.publish(msg)