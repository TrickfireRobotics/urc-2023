import rclpy
from . import moteus_motor
from rclpy.node import Node
import moteus


class RosMotuesBridge(Node):




    def __init__(self):
        super().__init__("can_motues_node")
        self.get_logger().info("LAUNCHING can_motues node")
        self.createMoteusMotors()

    def createMoteusMotors(self):
        self.get_logger().info("Creating motors")

        motuesTopicsList = [
            moteus.Register.MODE, 
            moteus.Register.HOME_STATE,
            moteus.Register.POSITION]

        motuesTopicsList2 = [
            moteus.Register.ABS_POSITION, 
            moteus.Register.HOME_STATE,
            moteus.Register.POSITION]

        test = moteus_motor.MoteusMotor(1,"myname", motuesTopicsList, motuesTopicsList2, self)



def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
