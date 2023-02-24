import moteus
import std_msgs.msg
import asyncio
from multiprocessing import Queue, Process, Pipe
from std_msgs.msg import Float32
import math


class Mode:
    POSITION = 0  # Asks for float8/float16/float32. Measured in revolutions
    VELOCITY = 1  # Asks for float8/float16/float32. Measured in revolutions per second

class MoteusMotor:
    """Connects to a moteus controller. Publishes desired register's values from controller"""

    def __init__(self, canID, name, motorMode, moteusPubList, queueToMoteus, toPublisherQueue, rosNode):
        """    
        Parameters
        ----------
        canID : int 
            Can ID of the moteus controller you are trying to connect to
        name : str
            The name of the controller. This string is used in the topic 
            names of both subscribers and publishers
        motorMode : Mode
            The mode of this controller. This effects what kind of data 
            the object expects and what data the object will send to the 
            moteus controller
        moteusPubList : array of moteus.Register
            Determines what data from the motor will be published to ROS
        """

        rosNode.get_logger().info("Creating motor with name: " + name)
        self._canID = canID
        self._name = name
        self._moteusRegToPubObjHashmap = {}
        self._rosNode = rosNode
        self._motorMode = motorMode
        self._queueToMoteus = queueToMoteus
        self._toPublisherQueue = toPublisherQueue
        self._moteusPubList = moteusPubList

        # Create subscribers and publishers
        self._createPublishers(moteusPubList)
        self._createSubscribers()

        rosNode.create_timer(0.02, self._readFromMoteusQueue)


    def _readFromMoteusQueue(self):
        """Recieve data from multiprocess and publish it to the corresponding register topic"""

        if (self._toPublisherQueue != None) and (not self._toPublisherQueue.empty()):
            dataRecieved = self._toPublisherQueue.get()
            register = dataRecieved[0]  # is a moteus.Register enum
            data = dataRecieved[1]  # is a floating point number

            self._rosNode.get_logger().info("data from queue" + str(dataRecieved) + " motor name and ID" + self._name + " " + str(self._canID))  # TODO Remove this logger            

            publisher = self._moteusRegToPubObjHashmap[register]
            msg = Float32()
            msg.data = data

            publisher.publish(msg)


    def _createPublishers(self, moteusPubList):
        """Create a publisher for data for each moteus register


        For each register that we want to publish to ROS, create
        a specific publisher topic for it.

        The format of the publisher is:
        <name of the motor>_<the data being sent via moteus.register>_from_can
        Example:
        /frontleftdrivemotor_velocity_from_can
        or
        /turntable_position_from_can


        Parameters
        ----------
        moteusPubList : array of moteus.Register
            Determines what data from the motor will be published to ROS
        """

        for register in moteusPubList:
            topicName = self._name + "_" + \
                str(register).replace("Register.", "").lower() + "_from_can"
            publisher = self._rosNode.create_publisher(
                std_msgs.msg.Float32, topicName, 10)

            self._moteusRegToPubObjHashmap[register] = publisher

    def _createSubscribers(self):
        """Create subscriber for the corresponding mode

        The format of the subscriber is:
        <name of the motor>_<the data being recieved via moteus.register>interface
        Example:
        /frontleftdrivemotor_velocity_from_robot_interface
        or
        /turntable_position_from_robot_interface
        """

        dataName = ""
        callbackFunction = None

        if (self._motorMode == Mode.POSITION):
            dataName = "position"
            callbackFunction = self._positionCallback
        elif (self._motorMode == Mode.VELOCITY):
            dataName = "velocity"
            callbackFunction = self._velocityCallback

        topicName = self._name + "_" + dataName + "_from_robot_interface"

        self._rosNode.subscription = self._rosNode.create_subscription(
            std_msgs.msg.Float32,
            topicName,
            callbackFunction,
            10
        )

    def _positionCallback(self, msg):
        """Send the data to the multiprocess for position
        msg : Float32
            This is sent from from the subscriber.
        """

        self._rosNode.get_logger().info("position callback with data: " +
                                        str(msg.data))  # TODO remove this logger


        if self._queueToMoteus != None:
            # [0] can ID
            # [1] data value
            toMultiprocess = [self._canID,self._modeData.position]
            self._queueToMoteus.put(toMultiprocess)

        # self._ros_pipe_conn.send(self._modeData)

    def _velocityCallback(self, msg):
        """Send the data to the multiprocess for velocity
        msg : Float32
            This is sent from from the subscriber.
        """

        self._rosNode.get_logger().info("velocity callback with data: " + str(msg.data))

        if self._queueToMoteus != None:
            # [0] can ID
            # [1] data value
            toMultiprocess = [self._canID,self._modeData.velocity]
            self._queueToMoteus.put(toMultiprocess)


        # self._ros_pipe_conn.send(self._modeData)


# class _MoteusAsyncLoop:
#     """An object used to create a moteus motor multiprocess"""

#     def __init__(self, motorMode, registersToPublish, rosNode, canID):
#         """
#         motorMode : Mode
#             The mode of this controller. This effects what kind of data 
#             the object expects and what data the object will send to the 
#             moteus controller
#         registersToPublish : array of moteus.Register
#             Determines what data to be queries from moteus.Result
#         rosNode : node
#             This node is not the same as the ROS node process. It is cloned.
#             As such, you cannot use this node for any ROS tooling - except
#             for get_logger().info() and get_logger().error(), which works
#         canID : int
#             Can ID of the moteus controller you are trying to connect to
#         """

#         rosNode.get_logger().info("Inside __init__")
#         self._motorMode = motorMode
#         self._modeData = None
#         self._registersToPublish = registersToPublish
#         self._rosNode = rosNode
#         self._moteusController = None
#         self._canID = canID

#         if (motorMode == Mode.POSITION):
#             self._modeData = _PositionData()
#         elif (motorMode == Mode.VELOCITY):
#             self._modeData = _VelocityData()

#     def startAsyncMoteusLoop(self, pipeEnd, rosQueue):
#         """Set up before going into loop for the process"""

#         self._rosNode.get_logger().info(
#             "startAsyncMoteusLoop called")  # TODO remove this logger

#         asyncio.run(self._loop(pipeEnd, rosQueue))

#     async def _connect(self):
#         """
#         Attempts to create a connection to the moteus controller; sends the reset command

#         Returns
#         -------
#         boolean
#             True if connection was successful, False if not
#         """

#         self._moteusController = moteus.Controller(self._canID)

#         try:
#             # Reset the controller
#             await self._moteusController.set_stop()
#         except RuntimeError as error:
#             self._rosNode.get_logger().error(
#                 "FAILED TO CONNECT TO MOTEUS CONTROLLER WITH CAN ID " + str(self._canID))
#             self._rosNode.get_logger().error(error.__str__())
#             return False

#         return True

#     async def _loop(self, pipeEnd, rosQueue):
#         """The main loop of the process

#         Parameter
#         ---------
#         pipeEnd : Pipe
#             This the second part of the pipe that was made in the ROS process.
#             It will be used to only recieve updated data from the ROS subscribers

#         rosQueue : Queue
#             This is used to put data from the moteus controller so that the ROS
#             node can publish the data
#         """

#         isSuccessfulConnection = await self._connect()

#         # Go into the loop
#         while isSuccessfulConnection:

#             # If there is data to be read in the pipe, update the data thats
#             # going to be sent to the moteus controller
#             # Else send what we recieved last time
#             if (pipeEnd.poll()):
#                 self._modeData = pipeEnd.recv()

#             resultFromMoteus = None

#             if (self._motorMode == Mode.POSITION):
#                 resultFromMoteus = await self._moteusController.set_position(position=self._modeData.position, query=True)
#             elif (self._motorMode == Mode.VELOCITY):
#                 # moteus controllers will only go a velocity only if it
#                 # has reached its given position OR we give it math.nan
#                 resultFromMoteus = await self._moteusController.set_position(position=math.nan, velocity=self._modeData.velocity, query=True)

#             self.publishdata(resultFromMoteus, rosQueue)

#             await asyncio.sleep(0.02)

#     def publishdata(self, resultFromMoteus, rosQueue):
#         """For each register, send data through the queue to the main ROS process

#         Parameters
#         ----------
#         resultFromMoteus : moteus.Result
#             This contains all the data from the moteus
#         rosQueue : Queue
#             This is used to put data from the moteus controller so that the ROS
#             node can publish the data
#         """

#         for register in self._registersToPublish:
#             # [0] moteus.Register
#             # [1] floating point number (the data)
#             message = [register, resultFromMoteus.values[register]]
#             rosQueue.put(message)
