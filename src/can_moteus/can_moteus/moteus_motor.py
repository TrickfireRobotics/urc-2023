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

        #Deal with moteus controller
        isConnected = asyncio.run(self.setupMoteusController())

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        if (isConnected):
            #asyncio.run(self.startLoop())
            a = 1
        else:
            rosNode.get_logger().error("FAILED TO CONNECT TO CONTROLLER: NAME: " +
                                       name + " CAN ID: " + str(self._canID))


        
        

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



    # Create a new moteus object
    # Start up in position mode and stop any movement of the motors
    # If we cannot detect a motor on the CANFD/CAN bus, handle the exception
    # Returns True if connection was successful
    # Returns False if we had an issue connecting
    async def setupMoteusController(self):

        self.moteusController = moteus.Controller(self._canID)

        try:
            await self.moteusController.set_stop()
        except RuntimeError as error:
            self._rosNode.get_logger().error(error.__str__())
            return False

        return True

    


