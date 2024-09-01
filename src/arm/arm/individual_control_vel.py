import sys
sys.path.append("/home/trickfire/urc-2023/src")
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from interface.robot_interface import RobotInterface

class IndividualControlVel():
    
    
    def __init__(self, rosNode, interface):
        self.rosNode = rosNode
        self.botInterface = interface
        
        self.WRIST_VEL = 0.3
        self.VEL = 1.0

        # Left wrist motor
        self.left_wrist_motor_sub = rosNode.create_subscription(
            Float32, "left_wrist_cw", self.left_wrist_cw, 10)
        self.left_wrist_motor_sub = rosNode.create_subscription(
            Float32, "left_wrist_ccw", self.left_wrist_ccw, 10)
        
        # Right wrist motor
        self.left_wrist_motor_sub = rosNode.create_subscription(
            Float32, "right_wrist_cw", self.right_wrist_cw, 10)
        self.left_wrist_motor_sub = rosNode.create_subscription(
            Float32, "right_wrist_ccw", self.right_wrist_ccw, 10)
        
        #Elbow motor
        self.elbow_up_sub = rosNode.create_subscription(
            Float32, "elbow_up", self.elbow_up, 10)
        
        self.elbow_down_sub = rosNode.create_subscription(
            Float32, "elbow_down", self.elbow_down, 10)
        
        #Shoulder motor
        self.shoulder_up_sub = rosNode.create_subscription(
            Float32, "shoulder_up", self.shoulder_up, 10)
        
        self.elboshoulder_down_sub = rosNode.create_subscription(
            Float32, "shoulder_down", self.shoulder_down, 10)
        
        #Turn table
        self.turntable_clockwise_sub = rosNode.create_subscription(
            Float32, "turntable_cw", self.turntable_cw, 10)

        self.turntable_counter_clockwise_sub = rosNode.create_subscription(
            Float32, "turntable_ccw", self.turntable_ccw, 10)
    
    
    def left_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Left Wrist CW" + str(joystick))
            self.botInterface.leftWrist_velocity(self.WRIST_VEL)
            
        else:
            self.rosNode.get_logger().info("Left Wrist STOP")
            self.botInterface.leftWrist_velocity(0.0)
            

    def left_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Left Wrist CCW" + str(joystick))
            self.botInterface.leftWrist_velocity(-self.WRIST_VEL)
            
        else:
            self.rosNode.get_logger().info("Left Wrist STOP")
            self.botInterface.leftWrist_velocity(0.0)
            

    def right_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Right Wrist CW" + str(joystick))
            self.botInterface.rightWrist_velocity(-self.WRIST_VEL)
            
        else:
            self.rosNode.get_logger().info("Right Wrist STOP")
            self.botInterface.rightWrist_velocity(0.0)
            

    def right_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Right Wrist CCW" + str(joystick))
            self.botInterface.rightWrist_velocity(-self.WRIST_VEL)
            
        else:
            self.rosNode.get_logger().info("Right Wrist STOP")
            self.botInterface.rightWrist_velocity(0.0)
            

    def elbow_up(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Elbow up" + str(data))
            self.botInterface.elbow_velocity(-self.VEL)
            
        else: 
            self.rosNode.get_logger().info("Elbow down STOP")
            self.botInterface.elbow_velocity(0.0)
            

    def elbow_down(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Elbow down" + str(data))
            self.botInterface.elbow_velocity(self.VEL)
            
        else:
            self.rosNode.get_logger().info("Elbow down STOP")
            self.botInterface.elbow_velocity(0.0)
            


    def shoulder_up(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Shoulder up" + str(data))
            self.botInterface.shoulder_velocity(-self.VEL)
            
        else:
            self.rosNode.get_logger().info("Shoulder STOP")
            self.botInterface.shoulder_velocity(0.0)
            


    def shoulder_down(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Shoulder down" + str(data))
            self.botInterface.shoulder_velocity(self.VEL)
            
        else:
            self.rosNode.get_logger().info("Shoulder STOP")
            self.botInterface.shoulder_velocity(0.0)
            
            
    def turntable_cw(self, msg):
        data = msg.data
        
        if  data > 0:
            self.rosNode.get_logger().info("Turntable clock wise")
            self.botInterface.arm_turntable_velocity(self.VEL)
            
        else:
            self.rosNode.get_logger().info("Turntable STOP")
            self.botInterface.arm_turntable_velocity(0.0)

    def turntable_ccw(self, msg):
        data = msg.data
        
        if  data > 0:
            self.rosNode.get_logger().info("Turntable counter clock wise")
            self.botInterface.arm_turntable_velocity(-self.VEL)
            
        else:
            self.rosNode.get_logger().info("Turntable STOP")
            self.botInterface.arm_turntable_velocity(0.0)
            
            