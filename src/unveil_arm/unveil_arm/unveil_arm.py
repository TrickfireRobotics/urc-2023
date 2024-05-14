import rclpy
from rclpy.node import Node

class UnveilArm(Node):

    def __init__(self):
        super().__init__("unveil_arm_node")
        self.get_logger().info("Launching unveil_arm node")



def main(args=None):
    rclpy.init(args=args)

    unveilArm = UnveilArm()

    rclpy.spin(unveilArm)
    unveilArm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()