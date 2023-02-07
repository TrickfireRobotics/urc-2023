import moteus
import std_msgs.msg
import asyncio
import math


class MoteusMotor:
    canID = None  # int
    name = None  # string
    moteusRegToDataHashmap = None  # Moteus.Register -> some value
    moteusRegToPubObjHashmap = None  # Moteus.Register -> some value
    rosNode = None  # The ros_moteus_bridge.py
    moteusController = None  # moteus.Controller

    def __init__(self, canID, name, moteusPubList, moteusSubList, rosNode):
        rosNode.get_logger().info("Creating motor with name: " + name)
        self.canID = canID
        self.name = name
        self.moteusRegToDataHashmap = dict()
        self.moteusRegToPubObjHashmap = dict()
        self.rosNode = rosNode

        # Setup subscriber stuff
        self.createMoteusToDataHashmap(moteusSubList)
        self.createSubscribers(moteusSubList, rosNode)

        # Setup publisher stuff
        self.createPublishers(moteusPubList)

        isConnected = asyncio.run(self.setupMoteusController())

        if(isConnected):
            asyncio.run(self.startLoop())
        else:
            rosNode.get_logger().error("FAILED TO CONNECT TO CONTROLLER: NAME: " + name + " CAN ID: " + str(self.canID))

        

    async def startLoop(self):
        while True:
            # For now, the only thing that will be sent to the
            # moteus controller is the position that we want
            returnedData = await self.moteusController.set_position(
                position=self.moteusRegToDataHashmap[moteus.Register.POSITION], maximum_torque=15, query=True)

            # Publish data to ROS
            self.publishDataToRos(returnedData)
            asyncio.sleep(0.02)
            


    #Goes through all of the publishers and sends the correct data from the
    #data we got from the motors
    def publishDataToRos(self, moteusResult):
        for register in self.moteusRegToPubObjHashmap:
            publisher = self.moteusRegToPubObjHashmap[register]
            data = moteusResult.values(register)

            publisher.publish(data)

    #Creates a publisher for each register 
    def createPublishers(self, moteusPubList):
        for register in moteusPubList:
            topicName = self.name + "_" + \
                str(register).replace("Register.", "") + "_from_can"
            publisher = self.rosNode.create_publisher(
                std_msgs.msg.Int32, topicName, 10)

            self.moteusRegToPubObjHashmap[register] = publisher

    #Create a new moteus object
    async def setupMoteusController(self):
        self.moteusController = moteus.Controller(self.canID)

        try:
            await self.moteusController.set_stop()
            await self.moteusController.set_position(position=math.nan, query=False)
        except RuntimeError as error:
            self.rosNode.get_logger().error(error.__str__())
            return False


        return True

    #Creates a map between the moteus registers and their respective data
    def createMoteusToDataHashmap(self, moteusSubList):
        for moteusRegister in moteusSubList:
            self.moteusRegToDataHashmap[moteusRegister] = 0

    #Create a subscriber for each of the registers to listen to
    def createSubscribers(self, moteusSubList, rosNode):
        for register in moteusSubList:
            topicName = self.name + "_" + \
                str(register).replace("Register.", "") + \
                "_from_robot_interface"
            rosNode.subscription = rosNode.create_subscription(
                std_msgs.msg.Int32,
                topicName,
                (lambda self, moteus_motor=self,
                 reg=register: moteus_motor.updateDataHashmap(reg, self)),
                10
            )

            rosNode.get_logger().info(str(topicName))

    #Update the data when we got it from the subscriber
    def updateDataHashmap(self, register, data):
        self.rosNode.get_logger().info("Change data for register: " + str(register))
        self.moteusRegToDataHashmap[register] = data
