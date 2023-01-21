import moteus
import std_msgs.msg


class MoteusMotor:
    canID = None  # int
    name = None  # string
    moteusRegToDataHashmap = None
    moteusRegToPubObjHashmap = None
    rosNode = None

    def __init__(self, canID, name, moteusPubList, moteusSubList, rosNode):
        rosNode.get_logger().info("Inside moteusmotor")
        self.canID = canID
        self.name = name
        self.moteusRegToDataHashmap = {}
        self.rosNode = rosNode

        self.createMoteusToDataHashmap(moteusSubList)
        self.createSubscribers(moteusSubList,rosNode)


    def createMoteusToDataHashmap(self, moteusSubList):
        for moteusRegister in moteusSubList:
            self.moteusRegToDataHashmap[moteusRegister] = "data"

    def createSubscribers(self,moteusSubList,rosNode):
        for register in moteusSubList:
            topicName = self.name + "_" + str(register).replace("Register.","") + "_from_robot_interface"
            rosNode.subscription = rosNode.create_subscription (
                std_msgs.msg.Int32,
                topicName,
                (lambda self, moteus_motor=self: moteus_motor.rosNode.get_logger().info("hello")),
                10
            )

            rosNode.get_logger().info(str(topicName))
