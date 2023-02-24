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

        # Creating a moteus motor
        moteusPubList = [moteus.Register.VELOCITY]
        mymotor = moteus_motor.MoteusMotor(
            3,
            "mymotor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
            self)
        
        moteusPubList2 = [moteus.Register.VELOCITY]
        mymotor2 = moteus_motor.MoteusMotor(
            2,
            "mymotor2",
            moteus_motor.Mode.POSITION,
            moteusPubList2,
            self)
        

        moteusMultiprocess.addMotor(mymotor)
        moteusMultiprocess.addMotor(mymotor2)

        moteusMultiprocess.start()


def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
