import rclpy
from . import moteus_motor
from . import moteus_multiprocess
from rclpy.node import Node
import moteus
from multiprocessing import Queue


class RosMotuesBridge(Node):

    def __init__(self):
        super().__init__("can_motues_node")
        self.get_logger().info("Launching can_motues node")
        self.createMoteusMotors()

    def createMoteusMotors(self):
        self.get_logger().info("Creating motors")

        
        moteusMultiprocess = moteus_multiprocess.MoteusMultiprocess(self)


        #EXAMPLES
        moteusPubList = [moteus.Register.VELOCITY]
        moteusMultiprocess.addMotor(
            6,
            "front_left_drive_motor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )

        # moteusPubList = [moteus.Register.VELOCITY]
        # moteusMultiprocess.addMotor(
        #     4,
        #     "front_right_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )

        # moteusPubList = [moteus.Register.VELOCITY]
        # moteusMultiprocess.addMotor(
        #     3,
        #     "mid_left_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )






        moteusMultiprocess.start()


def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
