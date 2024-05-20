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
        self.moteusMultiprocess = None

    def createMoteusMotors(self):
        self.get_logger().info("Creating motors")

        
        moteusMultiprocess = moteus_multiprocess.MoteusMultiprocess(self)




        # DRIVE BASE
        moteusPubList = [moteus.Register.VELOCITY]
        # moteusMultiprocess.addMotor(
        #     20,
        #     "rear_right_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )
        
        # moteusMultiprocess.addMotor(
        #     21,
        #     "mid_right_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )
        
        # moteusMultiprocess.addMotor(
        #     22,
        #     "front_right_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )
        
        # moteusMultiprocess.addMotor(
        #     23,
        #     "rear_left_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )

        # moteusMultiprocess.addMotor(
        #     24,
        #     "mid_left_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )
        
        # moteusMultiprocess.addMotor(
        #     25,
        #     "front_left_drive_motor",
        #     moteus_motor.Mode.VELOCITY,
        #     moteusPubList,
        # )


        # ARM

        moteusMultiprocess.addMotor(
            1,
            "shoulder_motor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )

        moteusMultiprocess.addMotor(
            2,
            "elbow_motor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )
        
        moteusMultiprocess.addMotor(
            3,
            "left_wrist_motor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )

        moteusMultiprocess.addMotor(
            4,
            "right_wrist_motor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )

        moteusMultiprocess.addMotor(
            5,
            "arm_turntable_motor",
            moteus_motor.Mode.VELOCITY,
            moteusPubList,
        )




        moteusMultiprocess.start()


def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.moteusMultiprocess.terminate()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
