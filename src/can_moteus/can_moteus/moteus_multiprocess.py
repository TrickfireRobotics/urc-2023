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


    def __init__(self, rosNode):
        self._queueToMoteus = Queue()
        self._canIDToMotorData = {}
        self._rosNode = rosNode
        


    def addMotor(self, canID, name, motorMode, moteusPubList):

        toPublisherQueue = Queue()

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
        moteusAsyncProcess = Process(target=self._startLoop, args=(self._queueToMoteus,))
        moteusAsyncProcess.start()


    def _startLoop(self,queueToMoteus):

        asyncio.run(self._loop(queueToMoteus))


    async def _loop(self,queueToMoteus):
        await self._connectToMoteusControllers()

        while True:
            self._readqueueToMoteus(queueToMoteus)

            for canID in self._canIDToMotorData:
                motorData = self._canIDToMotorData[canID]

                if (self._motorMode == motorData.mode.POSITION):
                    resultFromMoteus = await self._moteusController.set_position(position=motorData.data, query=True)
                elif (self._motorMode == motorData.mode.VELOCITY):
                    # moteus controllers will only go a velocity only if it
                    # has reached its given position OR we give it math.nan
                    resultFromMoteus = await self._moteusController.set_position(position=math.nan, velocity=motorData.data, query=True)

                self.publishdata(canID, resultFromMoteus)



            await asyncio.sleep(0.02)



    def _sendDataToMotor(self, canID, moteusResult):
        motorData = self._canIDToMotorData[canID]

        for register in motorData.moteusPubList:
            dataToMotor = [register, moteusResult.values[register]]

            motorData.toPublisherQueue.put(dataToMotor)


    def _readqueueToMoteus(self,queueToMoteus):

        if not queueToMoteus.empty():
            dataRecieved = queueToMoteus.get()
            canID = dataRecieved[0]  # is the canID of the motor
            data = dataRecieved[1]  # is a floating point data

            motorData = self._canIDToMotorData[canID]
            motorData.data = data

            #self._rosNode.get_logger().info("Number from queue: " + str(data) + " from CANID: " + str(canID))





    async def _connectToMoteusControllers(self):

        for canID in list(self._canIDToMotorData):
            motorData = self._canIDToMotorData[canID]

            moteusMotorController = moteus.Controller(motorData.canID)

            try:
                # Reset the controller
                await moteusMotorController.set_stop()
                motorData.moteusController = moteusMotorController

            except RuntimeError as error:
                self._rosNode.get_logger().error(
                    "FAILED TO CONNECT TO MOTEUS CONTROLLER WITH CAN ID " + str(motorData.canID))
                self._rosNode.get_logger().error(error.__str__())

                del self._canIDToMotorData[canID]













