import moteus
import asyncio
from multiprocessing import Queue, Process
import math
from . import moteus_motor


class _MotorData:
    canID = -1
    mode = None
    toPublisherQueue = None # [0] Register [1] data
    moteusPubList = None
    data = 0
    moteusController = None

class MoteusMultiprocess:
    """Connects to moteus controllers and sends data to each controller in order
    
        ------
        This will create a multiprocess queue for each moteus motor to send
        data that it should publish. There will be one queue that all the motors
        will push to.
    """

    def __init__(self, rosNode):
        """
        Parameters
        ----------
        rosNode : Node
            This is used, and should only be used, as a way to print logging
            information to the console. 
        """

        self._queueToMoteus = Queue()
        self._canIDToMotorData = {}
        self._rosNode = rosNode
        


    def addMotor(self, canID, name, motorMode, moteusPubList):
        """
        Parameters
        ----------
        canID : int
            Can ID of the moteus controller you are trying to connect to
        name : str
            The name of the controller. This string is used in the topic 
            names of both subscribers and publishers
        motorMode : moteus_controller.Mode
            The mode of this controller. This effects what kind of data 
            the object expects and what data the object will send to the 
            moteus controller
        moteusPubList : moteus.Register[]
            Determines what data from the motor will be published to ROS
        
        """
        toPublisherQueue = Queue()

        self._rosNode.get_logger().info("ADDING MOTOR")

        # Create motor
        motor = moteus_motor.MoteusMotor(
            canID,
            name,
            motorMode,
            moteusPubList,
            self._queueToMoteus,
            toPublisherQueue,
            self._rosNode
        )

        # Create motor data
        motorData = _MotorData()
        motorData.canID = canID
        motorData.mode = motorMode
        motorData.toPublisherQueue = toPublisherQueue
        motorData.moteusPubList = moteusPubList
    
        # Add this to the map
        self._canIDToMotorData[canID] = motorData


    # Multiprocess realm

    def start(self):
        """
            Starts the multiprocess. Call this after you added all the motors
        """
        moteusAsyncProcess = Process(target=self._startLoop, args=(self._queueToMoteus,))
        moteusAsyncProcess.start()


    def _startLoop(self,queueToMoteus):
        """
            This is now inside the multiprocess. Do any setup here
            that does not depend on asyncio

            Parameters
            ----------
            queueToMoteus : Queue
                This queue is what the multiprocess will read from to update
                its internal data for each motor
        """
        asyncio.run(self._loop(queueToMoteus))


    async def _loop(self,queueToMoteus):
        """
            Connect to the motors and start the loop. For each
            motor, call the correct moteus method and send 
            the correct data

            Parameters
            ----------
            queueToMoteus : Queue
                This queue is what the multiprocess will read from to update
                its internal data for each motor
        """

        self._rosNode.get_logger().info("HELLO WORLD")
        await self._connectToMoteusControllers()
        self._rosNode.get_logger().info("GOODBYE WORLD :(")

        while True:
            self._readqueueToMoteus(queueToMoteus)

            for canID in self._canIDToMotorData:
                motorData = self._canIDToMotorData[canID]

                if (motorData.mode == moteus_motor.Mode.POSITION):
                    #self._rosNode.get_logger().info(str(motorData.data))
                    resultFromMoteus = await motorData.moteusController.set_position(position=motorData.data, velocity=0, query=True)
                    

                elif (motorData.mode == moteus_motor.Mode.VELOCITY):
                    # moteus controllers will only go a velocity only if it
                    # has reached its given position OR we give it math.nan
                    resultFromMoteus = await motorData.moteusController.set_position(position=math.nan, velocity=motorData.data, query=True)

                self._sendDataToMotor(canID, resultFromMoteus)



            await asyncio.sleep(0.02)



    def _sendDataToMotor(self, canID, moteusResult):
        """
            Send the requested data for the specified motor via its canID

            Parameters
            ----------
            canID : int
                The canID for the motor
            moteusResult : moteus.Result
                The data returned from the moteus motor
        """

        # Get the motor from the can ID
        motorData = self._canIDToMotorData[canID]

        # Goe through all the registers we want to publish
        for register in motorData.moteusPubList:
            dataToMotor = [register, moteusResult.values[register]]

            motorData.toPublisherQueue.put(dataToMotor)


    def _readqueueToMoteus(self,queueToMoteus):
        """
            Reads one value from the queue to the moteus multiprocess

            Process
            -------
            queueToMoteus : Queue
                The queue that every motor will send their data to

        """


        if not queueToMoteus.empty():
            dataRecieved = queueToMoteus.get()
            canID = dataRecieved[0]  # is the canID of the motor
            data = dataRecieved[1]  # is a floating point data

            motorData = self._canIDToMotorData[canID]
            motorData.data = data


    async def _connectToMoteusControllers(self):
        """
            Attempts to connect to every moteus controller.
            If an attempt fails, it is removed from the dictionary

            
        """

        for canID in list(self._canIDToMotorData):
            motorData = self._canIDToMotorData[canID]

            moteusMotorController = moteus.Controller(motorData.canID)

            try:
                # Reset the controller
                self._rosNode.get_logger().info("HELLO WORLD SET STOP")
                await moteusMotorController.set_stop()
                self._rosNode.get_logger().info("GOODBYE WORLD SET STOP :(")
                motorData.moteusController = moteusMotorController

            except RuntimeError as error:
                self._rosNode.get_logger().error(
                    "FAILED TO CONNECT TO MOTEUS CONTROLLER WITH CAN ID " + str(motorData.canID))
                self._rosNode.get_logger().error(error.__str__())

                del self._canIDToMotorData[canID]
