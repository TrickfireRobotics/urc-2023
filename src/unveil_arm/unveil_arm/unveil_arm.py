import rclpy
from rclpy.node import Node

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from interface.robot_interface import RobotInterface

from std_msgs.msg import String, Float32

class UnveilArm(Node):

    def __init__(self):
        super().__init__("unveil_arm_node")
        self.get_logger().info("Launching unveil_arm node")

        self.botInterface = RobotInterface(self)
        self.SPEED = 0.05

        # Left wrist motor
        self.left_wrist_motor_sub = self.create_subscription(
            Float32, "left_wrist_cw", self.left_wrist_cw, 10)
        self.left_wrist_motor_sub = self.create_subscription(
            Float32, "left_wrist_ccw", self.left_wrist_ccw, 10)
        
        # Right wrist motor
        self.left_wrist_motor_sub = self.create_subscription(
            Float32, "right_wrist_cw", self.right_wrist_cw, 10)
        self.left_wrist_motor_sub = self.create_subscription(
            Float32, "right_wrist_ccw", self.right_wrist_ccw, 10)
        
        #Elbow motor
        self.elbow_up_sub = self.create_subscription(
            Float32, "elbow_up", self.elbow_up, 10)
        
        self.elbow_down_sub = self.create_subscription(
            Float32, "elbow_down", self.elbow_down, 10)
        
        #Shoulder motor
        self.shoulder_up_sub = self.create_subscription(
            Float32, "shoulder_up", self.shoulder_up, 10)
        
        self.elboshoulder_down_sub = self.create_subscription(
            Float32, "shoulder_down", self.shoulder_down, 10)



    def left_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Left Wrist CW" + str(joystick))
            self.botInterface.arm_left_wrist_motor(self.SPEED)
        else:
            self.botInterface.arm_left_wrist_motor(0.0)

    def left_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Left Wrist CCW" + str(joystick))
            self.botInterface.arm_left_wrist_motor(-self.SPEED)
        else:
            self.botInterface.arm_left_wrist_motor(0.0)

    def right_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Right Wrist CW" + str(joystick))
            self.botInterface.arm_right_wrist_motor(self.SPEED)
        else:
            self.botInterface.arm_right_wrist_motor(0.0)

    def right_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Right Wrist CCW" + str(joystick))
            self.botInterface.arm_right_wrist_motor(-self.SPEED)
        else:
            self.botInterface.arm_right_wrist_motor(0.0)

    def elbow_up(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Elbow up" + str(data))
            self.botInterface.arm_elbow_motor(self.SPEED)
        else: 
            self.botInterface.arm_elbow_motor(0.0)

    def elbow_down(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Elbow down" + str(data))
            self.botInterface.arm_elbow_motor(-self.SPEED)
        else:
            self.botInterface.arm_elbow_motor(0.0)


    def shoulder_up(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Shoulder up" + str(data))
            self.botInterface.arm_shoulder_motor(self.SPEED)
        else:
            self.botInterface.arm_shoulder_motor(0.0)


    def shoulder_down(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Shoulder down" + str(data))
            self.botInterface.arm_shoulder_motor(-self.SPEED)
        else:
            self.botInterface.arm_shoulder_motor(0.0)
    



def main(args=None):
    rclpy.init(args=args)

    unveilArm = UnveilArm()

    rclpy.spin(unveilArm)
    unveilArm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()