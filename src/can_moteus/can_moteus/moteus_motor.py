import moteus
import std_msgs.msg
import asyncio
import math

class Mode:
    POSITION = 0 # Asks for int8/int16/int32. Measured in revolutions
    CURRENT = 1 # Asks for int8/int16/int32. Measured in Amps
    VOLTAGE = 2 # Asks for int8/int16/int32. Measured in Volts

class _PositionData:
    position = None # In revoutions

class _CurrentData:
    dCurrent = None # In Amps
    qCurrent = None # In Amps

# make_vfoc()
class _VoltageData:
    theta = None # Rate of change of electrical phase
    voltate = None # In Amps

class MoteusMotor: 
    _canID = None  # int
    _name = None  # string
    _modeData = None # Stores the corresponding data to the control type
    _moteusRegToPubObjHashmap = None  # Moteus.Register -> ROS Publisher object
    _rosNode = None  # The ros_moteus_bridge.py
    moteusController = None  # moteus.Controller
    _motorMode = None  # The mode in which the motor is in
        

    def __init__(self, canID, name, motorMode, moteusPubList, rosNode):
        rosNode.get_logger().info("Creating motor with name: " + name)
        self._canID = canID
        self._name = name
        self._modeData = self.createModeData(motorMode)
        self._moteusRegToPubObjHashmap = dict()
        self._rosNode = rosNode
        self._motorMode = motorMode

        #Create subscribers and publishers
        self.createPublishers(moteusPubList)
        self.createSubscribers()
        

    def createModeData(self, motorMode):
        if(motorMode == Mode.POSITION):
            return _PositionData()
        elif (motorMode == Mode.CURRENT):
            return _CurrentData()
        elif (motorMode == Mode.VOLTAGE):
            return _VoltageData()
        return None

    # For each register/data that we want to publish to ROS,
    # create a specific publisher topic for it.
    # The format of the publisher is:
    # <name of the motor>_<the data being sent via moteus.register>_from_can
    def createPublishers(self, moteusPubList):
        for register in moteusPubList:
            topicName = self._name + "_" + \
                str(register).replace("Register.", "").lower() + "_from_can"
            publisher = self._rosNode.create_publisher(
                std_msgs.msg.Int32, topicName, 10)

            self._moteusRegToPubObjHashmap[register] = publisher

    def createSubscribers(self):

        dataName = ""
        callbackFunction = None

        if(self._motorMode == Mode.POSITION):
            dataName = "position"
            callbackFunction = self._positionCallback
        elif (self._motorMode == Mode.CURRENT):
            dataName = "current"
            callbackFunction = self._currentCallback
        elif (self._motorMode == Mode.VOLTAGE):
            dataName = "voltage"
            callbackFunction = self._voltageCallback

        topicName = self._name + "_" + dataName + "_from_robot_interface"

        self._rosNode.subscription = self._rosNode.create_subscription(
                std_msgs.msg.Int32,
                topicName,
                callbackFunction,
                10
            )
        
        
    def _positionCallback(self, data):
        self._rosNode.get_logger().info("position callback")

    def _currentCallback(self, data):
        self._rosNode.get_logger().info("current callback")

    def _voltageCallback(self, data):
        self._rosNode.get_logger().info("voltage callback")












    # _canID = None  # int
    # _name = None  # string
    # _moteusRegToDataHashmap = None  # Moteus.Register -> int32 (for now)
    # _moteusRegToPubObjHashmap = None  # Moteus.Register -> ROS Publisher object
    # rosNode = None  # The ros_moteus_bridge.py
    # moteusController = None  # moteus.Controller
    # _motorMode = None  # The mode in which the motor is in

    # def __init__(self, mode, canID, name, moteusPubList, moteusSubList, rosNode):
    #     rosNode.get_logger().info("Creating motor with name: " + name)
    #     self._canID = canID
    #     self._name = name
    #     self._moteusRegToDataHashmap = dict()
    #     self._moteusRegToPubObjHashmap = dict()
    #     self.rosNode = rosNode
    #     self._motorMode = mode

    #     # Setup subscriber stuff
    #     self.createMoteusToDataHashmap(moteusSubList)
    #     self.createSubscribers(moteusSubList, rosNode)

    #     # Setup publisher stuff
    #     self.createPublishers(moteusPubList)

    #     # If for some reason it could not find the motor, do not start anything
    #     isConnected = asyncio.run(self.setupMoteusController())
    #     if (isConnected):
    #         asyncio.run(self.startLoop())
    #     else:
    #         rosNode.get_logger().error("FAILED TO CONNECT TO CONTROLLER: NAME: " +
    #                                    name + " CAN ID: " + str(self._canID))

    # async def startLoop(self):
    #     while True:
    #         returnedData = ""

    #         if (self._motorMode == self.Mode.POSITION):
    #             returnedData = await self.moteusController.set_position(
    #                 position=self._moteusRegToDataHashmap[moteus.Register.POSITION], maximum_torque=15, query=True)

    #         if (self._motorMode == self.Mode.CURRENT):
    #             returnedData = await self.moteusController.set_current(
    #                 d_A=self._moteusRegToDataHashmap[moteus.Register.D_CURRENT],
    #                 q_A=self._moteusRegToDataHashmap[moteus.Register.Q_CURRENT],
    #                 query=True
    #             )

    #         # What is VFOC? Do we need it?
    #         # if(self.motorMode == self.Mode.VFOC)

    #         # Publish data to ROS
    #         self.publishDataToRos(returnedData)
    #         asyncio.sleep(0.02)

    # # Goes through all of the publishers and sends the correct data from the
    # # data we got from the motors

    # async def publishDataToRos(self, moteusResult):
    #     for register in self._moteusRegToPubObjHashmap:
    #         publisher = self._moteusRegToPubObjHashmap[register]
    #         data = moteusResult.values(register)

    #         publisher.publish(data)

    # # For each register/data that we want to publish to ROS,
    # # create a specific publisher topic for it.
    # # The format of the publisher is:
    # # <name of the motor>_<the data being sent via moteus.register>_from_can
    # def createPublishers(self, moteusPubList):
    #     for register in moteusPubList:
    #         topicName = self._name + "_" + \
    #             str(register).replace("Register.", "").lower() + "_from_can"
    #         publisher = self.rosNode.create_publisher(
    #             std_msgs.msg.Int32, topicName, 10)

    #         self._moteusRegToPubObjHashmap[register] = publisher

    # # Create a new moteus object
    # # Start up in position mode and stop any movement of the motors
    # # If we cannot detect a motor on the CANFD/CAN bus, handle the exception
    # # Returns True if connection was successful
    # # Returns False if we had an issue connecting
    # async def setupMoteusController(self):

    #     self.moteusController = moteus.Controller(self._canID)

    #     try:
    #         await self.moteusController.set_stop()
    #         # await self.moteusController.set_position(position=math.nan, query=False)
    #     except RuntimeError as error:
    #         self.rosNode.get_logger().error(error.__str__())
    #         return False

    #     return True

    # # Creates a map between the moteus registers and their respective data
    # def createMoteusToDataHashmap(self, moteusSubList):
    #     for moteusRegister in moteusSubList:
    #         self._moteusRegToDataHashmap[moteusRegister] = 0

    # # Create a subscriber for each of the registers to listen to

    # # Go through each register that we want to subscribe to, position, current, etc
    # # and create a subscriber for it. Each subscriber will then modify the
    # # _moteusRegToDataHashmap with the the data recieved
    # # The format of the subscriber is:
    # # <name of the motor>_<the data being recieved via moteus.register>_from_robot_interface
    # def createSubscribers(self, moteusSubList, rosNode):
    #     for register in moteusSubList:
    #         topicName = self._name + "_" + \
    #             str(register).replace("Register.", "").lower() + \
    #             "_from_robot_interface"
    #         rosNode.subscription = rosNode.create_subscription(
    #             std_msgs.msg.Int32,
    #             topicName,
    #             (lambda self, moteus_motor=self,
    #                 reg=register: moteus_motor.updateDataHashmap(reg, self)),
    #             10
    #         )

    #         rosNode.get_logger().info(str(topicName))

    # # Update the data when we got it from the subscriber
    # def updateDataHashmap(self, register, data):
    #     self.rosNode.get_logger().info("Change data for register: " + str(register))
    #     self._moteusRegToDataHashmap[register] = data

    # # This describes which mode the motor will run in
    # # Technically, the moteus code will change the mode for us
    # # when we call set_position or set_current. But to prevent
    # # accidental use of the incorrect moteus controller mode,
    # # use this so that the correct functions are called.
    # class Mode:
    #     POSITION = 0
    #     CURRENT = 1
    #     VFOC = 2
