import moteus
import std_msgs.msg
import asyncio
from multiprocessing import Process, Pipe
from std_msgs.msg import Int32

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
        isConnected = asyncio.run(self.setupMoteusController())

        # loop = asyncio.new_event_loop()
        # asyncio.set_event_loop(loop)

        if (isConnected):
            self._rosNode.get_logger().info("Creating process")
            self._parent_conn, child_conn = Pipe() # Pipes to communicate between ROS process and moteus process

            moteusAsyncLoopObject =  _MoteusAsyncLoop(self._motorMode, self._moteusRegToPubObjHashmap, self._moteusController,  self._rosNode)
            moteusAsyncProcess = Process(target=moteusAsyncLoopObject.startAsyncMoteusLoop, args=(child_conn,))
            moteusAsyncProcess.start()
        else:
            rosNode.get_logger().error("FAILED TO CONNECT TO CONTROLLER: NAME: " +
                                       name + " CAN ID: " + str(self._canID))








    def processAsyncLoopTest(self,child_conn):
    
        # while(True):
        self._rosNode.get_logger().info("insid loop")

        # asyncio.run(self.awaitableTask(child_conn))
            


       # asyncio.sleep(5)
        # self._rosNode.get_logger().info("after sleep loop")
        

    # async def awaitableTask(self,child_conn):
    #     self._rosNode.get_logger().info("I am inside an awaitable function in a multiprocess")



    #     publisher = self._moteusRegToPubObjHashmap[moteus.Register.POSITION]

    #     msg = Int32()
    #     msg.data = 10

    #     publisher.publish(msg)

    #     while True:
    #         self._rosNode.get_logger().info( "Data being sent inside loop: " + str(child_conn.recv().position))
    #         await asyncio.sleep(0.3)

        
        

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
            await self._moteusController.set_stop()
        except RuntimeError as error:
            self._rosNode.get_logger().error(error.__str__())
            return False

        return True


class _MoteusAsyncLoop:
    
    def __init__(self,motorMode, moteusRegToPubObjHashmap, moteusObject, rosNode):
        rosNode.get_logger().info("Inside __init__")
        self._motorMode = motorMode
        self._modeData = None
        self._moteusRegToPubObjHashmap = moteusRegToPubObjHashmap
        self._rosNode = rosNode
        self._moteusObject = moteusObject

        if(motorMode == Mode.POSITION):
            self._modeData = _PositionData()
        elif(motorMode == Mode.CURRENT):
            self._modeData = _CurrentData()
        elif(motorMode == Mode.VOLTAGE):
            self._modeData = _VoltageData()

        

    def startAsyncMoteusLoop(self, pipeEnd):
        self._rosNode.get_logger().info("startAsyncMoteusLoop called")
        asyncio.run(self._loop(pipeEnd))


    async def _loop(self, pipeEnd):
        self._rosNode.get_logger().info("inside loop")

        #Go into the loop
        while True:
        
            # If there is data to be read in the pipe, update the data thats going to be sent to the moteus controller
            if (pipeEnd.poll()):
                self._modeData = pipeEnd.recv()
            
            resultFromMoteus = None

            if (self._motorMode == Mode.POSITION):
                resultFromMoteus = await self._moteusObject.set_position(position=self._modeData.position,query=True)
            elif (self._motorMode == Mode.CURRENT):
                resultFromMoteus = await self._moteusObject.set_current(d_A=self._modeData.dCurrent,q_A=self._modeData.qCurrent,query=True)
            elif (self._motorMode == Mode.VOLTAGE):
                resultFromMoteus = await self._moteusObject.set_vfoc(theta=self._modeData.theta,voltage=self._modeData.voltate,query=True)


            self.publishData(resultFromMoteus)

            await asyncio.sleep(0.02)
            


    def publishdata(self, resultFromMoteus):
        
        for register in self._moteusRegToPubObjHashmap:
            publisher = self._moteusRegToPubObjHashmap[register]

            dataToSend = Int32()
            dataToSend.data = resultFromMoteus.values[register]

            publisher.publish(dataToSend)



        # publisher = self._moteusRegToPubObjHashmap[moteus.Register.POSITION]

        # msg = Int32()
        # msg.data = 10

        # publisher.publish(msg)

        # previousMessage = _PositionData()

        # while True:
        #     dataGot = None
        #     if (pipeEnd.poll()):#There is data
        #         dataGot = pipeEnd.recv().position
        #         previousMessage.position = dataGot
        #     else:
        #         dataGot = previousMessage.position




        #     self._rosNode.get_logger().info( "Data being sent inside loop: " + str(dataGot))
            
        #     await asyncio.sleep(0.5)




