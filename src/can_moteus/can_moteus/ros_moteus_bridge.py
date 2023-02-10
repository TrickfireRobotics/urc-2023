import rclpy
from . import moteus_motor
from rclpy.node import Node
import moteus


class RosMotuesBridge(Node):


    def __init__(self):
        super().__init__("can_motues_node")
        self.get_logger().info("Launching can_motues node")
        self.createMoteusMotors()

    def createMoteusMotors(self):
        self.get_logger().info("Creating motors")

        #Creating a moteus motor
        moteusPubList = [moteus.Register.POSITION]
        moteusSubList = [moteus.Register.POSITION]
        motor = moteus_motor.MoteusMotor(
            moteus_motor.MoteusMotor.Mode.POSITION,
            1, 
            "mymotortest", 
            moteusPubList, 
            moteusSubList, 
            self)



def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
