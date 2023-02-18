import moteus
import std_msgs.msg
import asyncio
from multiprocessing import Queue, Process, Pipe
from std_msgs.msg import Float32

class Mode:
    POSITION = 0 # Asks for int8/int16/int32. Measured in revolutions
    CURRENT = 1 # Asks for int8/int16/int32. Measured in Amps
    VOLTAGE = 2 # Asks for int8/int16/int32. Measured in Volts

class _PositionData:
    position = 0 # In revoutions

class _CurrentData:
    dCurrent = 0 # In Amps
    qCurrent = 0 # In Amps

# make_vfoc()
class _VoltageData:
    theta = 0 # Rate of change of electrical phase
    voltate = 0 # In Amps

class MoteusMotor: 
    def __init__(self, canID, name, motorMode, moteusPubList, rosNode):
        rosNode.get_logger().info("Creating motor with name: " + name)
        self._canID = canID
        self._name = name
        self._modeData = self.createModeData(motorMode)
        self._moteusRegToPubObjHashmap = dict()
        self._rosNode = rosNode
        self._motorMode = motorMode
        self._moteusController = None

        #Create subscribers and publishers
        self.createPublishers(moteusPubList)
        self.createSubscribers()

        #Deal with moteus controller
        #isConnected = asyncio.run(self.setupMoteusController())

        # loop = asyncio.new_event_loop()
        # asyncio.set_event_loop(loop)




        self._rosNode.get_logger().info("Creating process")
        self._parent_conn, child_conn = Pipe() # Pipes to communicate between ROS process and moteus process

        self.dataQueue = Queue()

        rosNode.create_timer(0.02, self.readMultiprocessingQueue)

        moteusAsyncLoopObject =  _MoteusAsyncLoop(self._motorMode, self._moteusRegToPubObjHashmap, self._moteusController,  self._rosNode, self._canID)
        moteusAsyncProcess = Process(target=moteusAsyncLoopObject.startAsyncMoteusLoop, args=(child_conn, self.dataQueue,))
        moteusAsyncProcess.start()

        #self._parent_conn.send(self._moteusRegToPubObjHashmap)

        self._rosNode.get_logger().info("after process.start")
        

    def readMultiprocessingQueue(self):
        if not self.dataQueue.empty():
            dataRecieved = self.dataQueue.get()
            register = dataRecieved[0]
            data = dataRecieved[1]

            self._rosNode.get_logger().info( "data from queue" + str(dataRecieved))

            publisher = self._moteusRegToPubObjHashmap[register]
            msg = Float32()
            msg.data = data

            publisher.publish(msg)



        

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
                std_msgs.msg.Float32, topicName, 10)

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
                std_msgs.msg.Float32,
                topicName,
                callbackFunction,
                10
            )
        
        
    def _positionCallback(self, data):
        self._rosNode.get_logger().info("position callback with data: " + str(data.data))
        self._modeData.position = data.data
        positonData = _PositionData()
        positonData.position = data.data
        self._parent_conn.send(positonData)

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

        self._moteusController = moteus.Controller(self._canID)

        try:
            self._rosNode.get_logger().info("Just before set stop")
            await self._moteusController.set_stop()
        except RuntimeError as error:
            self._rosNode.get_logger().error(error.__str__())
            return False

        return True


class _MoteusAsyncLoop:
    
    def __init__(self,motorMode, moteusRegToPubObjHashmap, moteusObject, rosNode, canID):
        rosNode.get_logger().info("Inside __init__")
        self._motorMode = motorMode
        self._modeData = None
        self._moteusRegToPubObjHashmap = moteusRegToPubObjHashmap
        self._rosNode = rosNode
        self._moteusObject = moteusObject
        self._moteusController = None
        self._canID = canID

        if(motorMode == Mode.POSITION):
            self._modeData = _PositionData()
        elif(motorMode == Mode.CURRENT):
            self._modeData = _CurrentData()
        elif(motorMode == Mode.VOLTAGE):
            self._modeData = _VoltageData()


        # self.testPub = None
        # self.testPub = self._rosNode.create_publisher(std_msgs.msg.Float32,"test",10)

        msg = Float32()
        msg.data = 123.123

        self.testPub = self._rosNode.create_publisher(std_msgs.msg.Float32,"test",10)
        self.testPub.publish(msg)

        

    def startAsyncMoteusLoop(self, pipeEnd, childQueue):
        self._rosNode.get_logger().info("startAsyncMoteusLoop called")

        

        self._moteusController = moteus.Controller(self._canID)
        asyncio.run(self._loop(pipeEnd, childQueue))


    def publishdata(self, resultFromMoteus, childQueue):
        
        for register in self._moteusRegToPubObjHashmap:
            #self._rosNode.get_logger().info(str(type(resultFromMoteus.values[register])))

            # publisher = self._moteusRegToPubObjHashmap[register]
            # dataToSend = Float32()
            # dataToSend.data = resultFromMoteus.values[register]

            # publisher = self._moteusRegToPubObjHashmap[moteus.Register.POSITION]

            # dataToSend = Float32()
            # dataToSend.data = resultFromMoteus.values[moteus.Register.POSITION]

            # publisher.publish(dataToSend)
            # self.testPub.publish(dataToSend)

            array = [register,resultFromMoteus.values[register]]

            childQueue.put(array)


            #self._rosNode.get_logger().info("Publishing data: " + str(resultFromMoteus.values[register]))

    def printData(self, resultFromMoteus):
        self._rosNode.get_logger().info("Position from moteus: " + str(resultFromMoteus.values[moteus.Register.POSITION]))


    async def _loop(self, pipeEnd, childQueue):
        self._rosNode.get_logger().info("inside loop")


        isSuccessfulConnection = False
        try:
            self._rosNode.get_logger().info("Just before set stop")
            await self._moteusController.set_stop()
            self._rosNode.get_logger().info("Just after set stop")
            isSuccessfulConnection = True
        except RuntimeError as error:
            self._rosNode.get_logger().error(error.__str__())

        #Go into the loop
        while True:
        
            # If there is data to be read in the pipe, update the data thats going to be sent to the moteus controller
            if (pipeEnd.poll()):
                self._modeData = pipeEnd.recv()
            
            resultFromMoteus = None

            if (self._motorMode == Mode.POSITION):
                #self._rosNode.get_logger().info("sending data with position value: " + str(self._modeData.position))
                resultFromMoteus = await self._moteusController.set_position(position=self._modeData.position,query=True)
            elif (self._motorMode == Mode.CURRENT):
                resultFromMoteus = await self._moteusController.set_current(d_A=self._modeData.dCurrent,q_A=self._modeData.qCurrent,query=True)
            elif (self._motorMode == Mode.VOLTAGE):
                resultFromMoteus = await self._moteusController.set_vfoc(theta=self._modeData.theta,voltage=self._modeData.voltate,query=True)


            self.publishdata(resultFromMoteus, childQueue)
            #self.printData(resultFromMoteus)
            

            await asyncio.sleep(0.02)
            




