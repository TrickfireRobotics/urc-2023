import moteus
import std_msgs.msg
import asyncio
from multiprocessing import Queue, Process, Pipe
from std_msgs.msg import Float32
import math


class Mode:
    POSITION = 0  # Asks for float. Measured in revolutions
    VELOCITY = 1  # Asks float. Measured in revolutions per second


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
        moteusPubList : moteus.Register[]
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
            <name of the motor>_<the data being recieved via moteus.register>_from_robot_interface
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

        topicName = self._name + "_" + dataName + "_from_interface"


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

        if self._queueToMoteus != None:
            # [0] can ID
            # [1] data value
            toMultiprocess = [self._canID,msg.data]
            self._queueToMoteus.put(toMultiprocess)

        # self._ros_pipe_conn.send(self._modeData)

    def _velocityCallback(self, msg):
        """Send the data to the multiprocess for velocity
        msg : Float32
            This is sent from from the subscriber.
        """

        self._rosNode.get_logger().info("I AM CALLED")

        if self._queueToMoteus != None:
            # [0] can ID
            # [1] data value
            toMultiprocess = [self._canID,msg.data]
            self._queueToMoteus.put(toMultiprocess)
