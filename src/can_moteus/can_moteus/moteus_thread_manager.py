from . import moteus_motor
import threading
import asyncio
from rclpy.node import Node
import moteus

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from utility.color_text import ColorCodes

class MoteusThreadManager():

    def __init__(self, rosNode: Node):
        self._nameToMoteusMotor = {}
        self._nameToMoteusController = {} # Used by the thread
        self._rosNode = rosNode
        self._moteusThread = None
        self._shouldMoteusThreadLoop = True



    def addMotor(self, canID, motorName):
        #Create motor
        motor = moteus_motor.MoteusMotor(canID, motorName, self._rosNode)

        self._nameToMoteusMotor[motorName] = motor

    def start(self):
        self._moteusThread = threading.Thread(target = self.threadEntry, name = "moteus_thread", daemon = True)
        self._moteusThread.start()

    def terminateAllThreads(self):
        self._shouldMoteusThreadLoop = False
        self._moteusThread.join()
    
    def threadEntry(self):
        self._rosNode.get_logger().info("Moteus Thread Launched")
        asyncio.run(self.startLoop())
        
    async def tryToShutdownMotor(self, motorName):
        
        self._rosNode.get_logger().info("Trying to turn off motor \"" + str(motorName) + "\" (CANID " + self._nameToMoteusMotor[motorName].canID + ")")
        
        try:
            await asyncio.wait_for(self._nameToMoteusController[motorName].set_stop(), timeout = 1)
            self._rosNode.get_logger().info(ColorCodes.GREEN_OK + "Stopped motor")
        except asyncio.TimeoutError:
            self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "FAILED TO \"set_stop\" MOTOR " + str(self._nameToMoteusMotor[motorName].canID) + ColorCodes.ENDC)
        except RuntimeError as error:
            self._rosNode.get_logger().info(ColorCodes.FAIL_RED + error.__str__() + ColorCodes.ENDC)


    async def startLoop(self):
        await self.connectToMoteusControllers()

        
        while self._shouldMoteusThreadLoop:
            for name, controller in self._nameToMoteusController.items():
                moteusMotor = self._nameToMoteusMotor[name]
                
                
                # Check for faults
                resultFromMoteus = await controller.query()
                if resultFromMoteus.values[moteus.Register.FAULT] != 0:
                    self._rosNode.get_logger().info("FAULT CODE: " + str(resultFromMoteus.values[moteus.Register.FAULT]) + " FOR " + name + "(CANID: " + str(moteusMotor.canID) + ")")
                    self.tryToShutdownMotor(name)
                    
                if moteusMotor.setStop is True:
                    await controller.set_stop()
                else:
                    resultFromMoteus = await controller.set_position(
                        position = moteusMotor.position,
                        velocity = moteusMotor.velocity,
                        feedforward_torque = moteusMotor.feedforward_torque,
                        kp_scale = moteusMotor.kp_scale,
                        kd_scale = moteusMotor.kd_scale,
                        maximum_torque = moteusMotor.max_torque,
                        watchdog_timeout = moteusMotor.watchdog_timeout,
                        velocity_limit = moteusMotor.velocity_limit,
                        accel_limit = moteusMotor.accel_limit,
                        fixed_voltage_override = moteusMotor.fixed_voltage_override,
                        #ilimit_scale = moteusMotor.ilimit_scale,
                        query = True
                    )
                    
                    moteusMotor.publishData(resultFromMoteus)
                
            await asyncio.sleep(0.02)
            
        for name, controller in self._nameToMoteusController.items():
            await controller.set_stop()
        
    async def connectToMoteusControllers(self):
        for key, moteusMotor in self._nameToMoteusMotor.items():
            controller = moteus.Controller(moteusMotor.canID)
            
            try:
                # Reset the controller
                self._rosNode.get_logger().info("Connecting to motor with name: " + str(moteusMotor.name))
                await asyncio.wait_for(controller.query(), timeout = 1)
                self._nameToMoteusController[key] = controller
                await controller.set_stop()
                self._rosNode.get_logger().info(ColorCodes.GREEN_OK + "Motor connected: " + str(moteusMotor.name) + " (CANID " + str(moteusMotor.canID) + ")" + ColorCodes.ENDC)
                
            except asyncio.TimeoutError:
                self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "FAILED TO CONNECT TO MOTEUS CONTROLLER WITH CANID " + str(moteusMotor.canID) + ColorCodes.ENDC)
            except RuntimeError as error:
                self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "ERROR WHEN set_stop() IS CALLED. MOST LIKELY CANNOT FIND CANBUS" + ColorCodes.ENDC)
                self._rosNode.get_logger().info(ColorCodes.FAIL_RED + error.__str__() + ColorCodes.ENDC)

