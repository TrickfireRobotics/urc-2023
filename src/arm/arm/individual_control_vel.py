import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class IndividualControlVel():
    
    
    def __init__(self, rosNode, interface):
        self.rosNode = rosNode
        self.botInterface = interface

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
            
        else:
            self.rosNode.get_logger().info("Left Wrist STOP")
            

    def left_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Left Wrist CCW" + str(joystick))
            
        else:
            self.rosNode.get_logger().info("Left Wrist STOP")
            

    def right_wrist_cw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Right Wrist CW" + str(joystick))
            
        else:
            self.rosNode.get_logger().info("Right Wrist STOP")
            

    def right_wrist_ccw(self, msg):
        joystick = msg.data

        if joystick > 0:
            self.rosNode.get_logger().info("Right Wrist CCW" + str(joystick))
            
        else:
            self.rosNode.get_logger().info("Right Wrist STOP")
            

    def elbow_up(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Elbow up" + str(data))
            
        else: 
            self.rosNode.get_logger().info("Elbow down STOP")
            

    def elbow_down(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Elbow down" + str(data))
            
        else:
            self.rosNode.get_logger().info("Elbow down STOP")
            


    def shoulder_up(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Shoulder up" + str(data))
            
        else:
            self.rosNode.get_logger().info("Shoulder STOP")
            


    def shoulder_down(self, msg):
        data = msg.data

        if data > 0:
            self.rosNode.get_logger().info("Shoulder down" + str(data))
            
        else:
            self.rosNode.get_logger().info("Shoulder STOP")
            
            
    def turntable_cw(self, msg):
        data = msg.data
        
        if  data > 0:
            self.rosNode.get_logger().info("Turntable clock wise")
            
        else:
            self.rosNode.get_logger().info("Turntable STOP")
            

    def turntable_ccw(self, msg):
        data = msg.data
        
        if  data > 0:
            self.rosNode.get_logger().info("Turntable counter clock wise")
            
        else:
            self.rosNode.get_logger().info("Turntable STOP")
            
            