import rclpy
from rclpy.node import Node

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from interface.robot_interface import RobotInterface
from std_msgs.msg import String, Float32

import RPi.GPIO as GPIO
import time


class UnveilArm(Node):

    def __init__(self):
        super().__init__("unveil_arm_node")
        self.get_logger().info("Launching unveil_arm node")

        self.botInterface = RobotInterface(self)
        self.SPEED = 0.05
        self.WRIST_SPEED = 0.3
        
        # self.botInterface.arm_left_wrist_motor(0.0)
        # self.botInterface.arm_right_wrist_motor(0.0)
        # self.botInterface.arm_shoulder_motor(0.0)
        # self.botInterface.arm_elbow_motor(0.0)
        
        #PWM Gripper stuff
        self.GRIPPER_ROT_PIN = 18
        self.GRIPPER_LIN_PIN = 15

        self.gripperRotDuty = 7 # 5% duty cycle = 0 degrees
        self.gripperLinDuty = 7
        self.gripperDeltaDuty = 0.5
        
        
        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(self.GRIPPER_ROT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.GRIPPER_LIN_PIN, GPIO.OUT, initial=GPIO.HIGH)
        
        self.gripperRotPWM = GPIO.PWM(self.GRIPPER_ROT_PIN, 50)
        self.gripperRotPWM.start(self.gripperRotDuty)
        
        self.gripperLinPWM = GPIO.PWM(self.GRIPPER_LIN_PIN, 50)
        self.gripperLinPWM.start(self.gripperLinDuty)
        
        
        self.gripperRotationOpen = self.create_subscription(
            Float32, "gripRotOpen", self.gripRotOpen, 10)
        
        self.gripperRotationClose = self.create_subscription(
            Float32, "gripRotClose", self.gripRotClose, 10)
        
        self.gripperLinearOpen = self.create_subscription(
            Float32, "gripLinearOpen", self.gripLinearOpen, 10)
        
        self.gripperLinearClose = self.create_subscription(
            Float32, "gripLinearClose", self.gripLinearClose, 10)

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
        
        #Turn table
        self.turntable_clockwise_sub = self.create_subscription(
            Float32, "turntable_cw", self.turntable_cw, 10)

        self.turntable_counter_clockwise_sub = self.create_subscription(
            Float32, "turntable_ccw", self.turntable_ccw, 10)


    def left_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Left Wrist CW" + str(joystick))
            self.botInterface.arm_left_wrist_motor(self.WRIST_SPEED)
        else:
            self.get_logger().info("Left Wrist STOP")
            self.botInterface.arm_left_wrist_motor(0.0)

    def left_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Left Wrist CCW" + str(joystick))
            self.botInterface.arm_left_wrist_motor(-self.WRIST_SPEED)
        else:
            self.get_logger().info("Left Wrist STOP")
            self.botInterface.arm_left_wrist_motor(0.0)

    def right_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Right Wrist CW" + str(joystick))
            self.botInterface.arm_right_wrist_motor(self.WRIST_SPEED)
        else:
            self.get_logger().info("Right Wrist STOP")
            self.botInterface.arm_right_wrist_motor(0.0)

    def right_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.get_logger().info("Right Wrist CCW" + str(joystick))
            self.botInterface.arm_right_wrist_motor(-self.WRIST_SPEED)
        else:
            self.get_logger().info("Right Wrist STOP")
            self.botInterface.arm_right_wrist_motor(0.0)

    def elbow_up(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Elbow up" + str(data))
            self.botInterface.arm_elbow_motor(-self.SPEED)
        else: 
            self.get_logger().info("Elbow down STOP")
            self.botInterface.arm_elbow_motor(0.0)

    def elbow_down(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Elbow down" + str(data))
            self.botInterface.arm_elbow_motor(self.SPEED)
        else:
            self.get_logger().info("Elbow down STOP")
            self.botInterface.arm_elbow_motor(0.0)


    def shoulder_up(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Shoulder up" + str(data))
            self.botInterface.arm_shoulder_motor(-self.SPEED)
        else:
            self.get_logger().info("Shoulder STOP")
            self.botInterface.arm_shoulder_motor(0.0)


    def shoulder_down(self, msg):
        data = msg.data

        if data > 0:
            self.get_logger().info("Shoulder down" + str(data))
            self.botInterface.arm_shoulder_motor(self.SPEED)
        else:
            self.get_logger().info("Shoulder STOP")
            self.botInterface.arm_shoulder_motor(0.0)
            
    def turntable_cw(self, msg):
        data = msg.data
        
        if  data > 0:
            self.get_logger().info("Turntable clock wise")
            self.botInterface.arm_turntable(self.SPEED)
        else:
            self.get_logger().info("Turntable STOP")
            self.botInterface.arm_turntable(0.0)

    def turntable_ccw(self, msg):
        data = msg.data
        
        if  data > 0:
            self.get_logger().info("Turntable counter clock wise")
            self.botInterface.arm_turntable(-self.SPEED)
        else:
            self.get_logger().info("Turntable STOP")
            self.botInterface.arm_turntable(0.0)
            
    def gripRotOpen(self, msg):
        data = msg.data
        if data > 0:
            self.get_logger().info("Gripper Rotation Open")
            self.gripperRotDuty += self.gripperDeltaDuty
            self.get_logger().info("Gripper Rotation DUTY " + str(self.gripperRotDuty))
            self.gripperRotPWM.ChangeDutyCycle(self.gripperRotDuty)
            
    def gripRotClose(self, msg):
        data = msg.data
        if data > 0:
            self.get_logger().info("Gripper Rotation Close")
            self.gripperRotDuty -= self.gripperDeltaDuty
            self.get_logger().info("Gripper Rotation DUTY " + str(self.gripperRotDuty))
            self.gripperRotPWM.ChangeDutyCycle(self.gripperRotDuty)
        
    def gripLinearOpen(self, msg):
        data = msg.data
        
        if data > 0:
            self.get_logger().info("Gripper Linear Open")
            self.gripperLinDuty += self.gripperDeltaDuty
            self.get_logger().info("Gripper Linear DUTY " + str(self.gripperLinDuty))
            self.gripperLinPWM.ChangeDutyCycle(self.gripperLinDuty)
            
    def gripLinearClose(self, msg):
        data = msg.data
        if data > 0:
            self.get_logger().info("Gripper Linear Close")
            self.gripperLinDuty -= self.gripperDeltaDuty
            self.get_logger().info("Gripper Linear DUTY " + str(self.gripperLinDuty))
            self.gripperLinPWM.ChangeDutyCycle(self.gripperLinDuty)


class UnveilArmTest(Node):
    def __init__(self):
        super().__init__("unveil_arm_node")
        self.get_logger().info("Launching unveil_arm node")
        
        # PWM Gripper stuff
        self.GRIPPER_ROT_PIN = 18
        self.GRIPPER_LIN_PIN = 15

        self.Duty = 7 # 5% duty cycle = 0 degrees
        self.gripperDeltaDuty = 0.5
    
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.GRIPPER_LIN_PIN, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.GRIPPER_ROT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.p1 = GPIO.PWM(self.GRIPPER_LIN_PIN, 50)
        self.p2 = GPIO.PWM(self.GRIPPER_ROT_PIN, 50)
        self.p1.start(5)
        self.p2.start(5)
        
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self, event=None):
        if self.Duty >= 5:
            self.gripperDeltaDuty = -self.gripperDeltaDuty
        if self.Duty <= 10:
            self.gripperDeltaDuty = -self.gripperDeltaDuty
        
        self.Duty += self.gripperDeltaDuty
        self.get_logger().info("Duty cycle: " + str(self.Duty))
        self.p1.ChangeDutyCycle(self.Duty)
        self.p2.ChangeDutyCycle(self.Duty)
    
        
        

def main(args=None):
    rclpy.init(args=args)

    unveilArm = UnveilArm()

    rclpy.spin(unveilArm)
    unveilArm.destroy_node()
    
    # unveilArm = UnveilArmTest()
    # rclpy.spin(unveilArm)
    # unveil_arm.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()