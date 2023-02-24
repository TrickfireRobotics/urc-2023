import moteus
import asyncio
from multiprocessing import Queue, Process
import math
from . import moteus_motor


class _MotorData:
    canID = -1
    mode = None
    publisherQueue = None # [0] Register [1] data
    data = 0
    moteusController = None

class MoteusMultiprocess:


    def __init__(self, rosNode):
        self._queueToMoteus = Queue()
        self._canIDToMotorData = {}
        self._rosNode = rosNode
        


    def addMotor(self, motor):

        motorData = _MotorData()
        motorData.canID = motor._canID
        motorData.mode = motor._motorMode
        motorData.publisherQueue = Queue()

        motor._queueToMoteus = self._queueToMoteus
        # Add this to the map
        self._canIDToMotorData[motorData.canID] = motorData


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

            await asyncio.sleep(0.02)



    def _readqueueToMoteus(self,queueToMoteus):

        if not queueToMoteus.empty():
            dataRecieved = queueToMoteus.get()
            canID = dataRecieved[0]  # is the canID of the motor
            data = dataRecieved[1]  # is a floating point data

            motorData = self._canIDToMotorData[canID]
            motorData.data = data

            self._rosNode.get_logger().info("Number from queue: " + str(data) + " from CANID: " + str(canID))





    async def _connectToMoteusControllers(self):

        for canID in self._canIDToMotorData:
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













