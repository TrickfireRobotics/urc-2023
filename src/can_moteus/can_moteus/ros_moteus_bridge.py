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



        moteusPubList = [moteus.Register.VELOCITY]
        moteusMultiprocess.addMotor(
            1,
            "topmotor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )

        moteusPubList2 = [moteus.Register.POSITION]
        moteusMultiprocess.addMotor(
            2,
            "bottommotor",
            moteus_motor.Mode.POSITION,
            moteusPubList2,
        )





        moteusMultiprocess.start()


def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
