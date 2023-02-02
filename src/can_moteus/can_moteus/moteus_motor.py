import moteus
import std_msgs.msg
import asyncio
import math


class MoteusMotor:
    canID = None  # int
    name = None  # string
    moteusRegToDataHashmap = None #Moteus.Register -> some value
    moteusRegToPubObjHashmap = None #Moteus.Register -> some value
    rosNode = None #The ros_moteus_bridge.py
    moteusController = None #moteus.Controller
    returnedData = None #moteus.Result

    def __init__(self, canID, name, moteusPubList, moteusSubList, rosNode):
        rosNode.get_logger().info("Creating motor with name: " + name)
        self.canID = canID
        self.name = name
        self.moteusRegToDataHashmap = {}
        self.rosNode = rosNode

        self.createMoteusToDataHashmap(moteusSubList)
        self.createSubscribers(moteusSubList,rosNode)

        self.setupMoteusController()
        self.startLoop()

    async def startLoop(self):
        while True:
            self.returnedData = self.moteusController


    async def setupMoteusController(self):
        self.moteusController = moteus.Controller(self.canID)
        await self.moteusController.set_stop()
        await self.moteusController.set_position(position=math.nan, query=False)


    def createMoteusToDataHashmap(self, moteusSubList):
        for moteusRegister in moteusSubList:
            self.moteusRegToDataHashmap[moteusRegister] = "data" + str(moteusRegister)

    def createSubscribers(self,moteusSubList,rosNode):
        for register in moteusSubList:
            topicName = self.name + "_" + str(register).replace("Register.","") + "_from_robot_interface"
            rosNode.subscription = rosNode.create_subscription (
                std_msgs.msg.Int32,
                topicName,
                (lambda self, moteus_motor=self,reg=register: moteus_motor.updateDataHashmap(reg,self)),
                10
            )

            rosNode.get_logger().info(str(topicName))

    def updateDataHashmap(self, register, data):
        self.rosNode.get_logger().info("Change data for register: " + str(register))
        self.moteusRegToDataHashmap[register] = data

