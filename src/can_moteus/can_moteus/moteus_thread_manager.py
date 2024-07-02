from . import moteus_motor
import threading
import asyncio
from rclpy.node import Node
import moteus

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from utility.color_text import ColorCodes

class MoteusThreadManager():
    """
        This creates a new thread called "moteus_thread" to run all
        of the asyncio methods required by the Moteus library
    """

    def __init__(self, rosNode: Node):
        self._nameToMoteusMotor = {}
        self._nameToMoteusController = {} # Used by the thread
        self._rosNode = rosNode
        self._moteusThread = None
        self._shouldMoteusThreadLoop = True # No thread safety, but it works lololol
        self._shouldReconnect = True # No thread safety, but it works lololol
        
        self._CONNECTION_TIMEOUT_IN_SECONDS = 0.1 # 100ms
        self._GENERAL_TIMEOUT = 0.05 #50ms



    def addMotor(self, canID, motorName):
        """
            Adds a motor to the list to attempt to connect to
        """
        #Create motor
        motor = moteus_motor.MoteusMotor(canID, motorName, self._rosNode)
        self._nameToMoteusMotor[motorName] = motor

    def start(self):
        """
            Starts a new thread. New motors cannot be added after this is called
        """
        
        self._moteusThread = threading.Thread(target = self.threadEntry, name = "moteus_thread", daemon = True)
        self._moteusThread.start()
        
    def reconnectMotors(self):
        self._shouldReconnect = True

    def terminateAllThreads(self):
        """
            Gracefully shuts down the motors by calling set_stop().
            Terminates the thread.
            Does not clean up this class
        """
        self._shouldMoteusThreadLoop = False
        self._moteusThread.join()   
        
    
    def threadEntry(self):
        """
            The entry of the thread that launches the asyncio loop
        """
        self._rosNode.get_logger().info(ColorCodes.BLUE_OK + "Moteus Thread Launched" + ColorCodes.ENDC)
        asyncio.run(self.startLoop())
        
    async def tryToShutdownMotor(self, motorName):
        
        self._rosNode.get_logger().info(ColorCodes.WARNING_YELLOW + "Unexpectedly trying to turn off motor \"" + str(motorName) + "\" (CANID " + str(self._nameToMoteusMotor[motorName].canID) + ")" + ColorCodes.ENDC)
        
        try:
            await asyncio.wait_for(self._nameToMoteusController[motorName].set_stop(), timeout = self._GENERAL_TIMEOUT)
            self._rosNode.get_logger().info(ColorCodes.GREEN_OK + "Stopped motor \"" + motorName + "\" (CANID " + str(self._nameToMoteusMotor[motorName].canID) + ")" + ColorCodes.ENDC)
        except asyncio.TimeoutError:
            self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "FAILED TO \"set_stop\" MOTOR. TIMED OUT \"" + motorName + "\"(CANID " + str(self._nameToMoteusMotor[motorName].canID) + ")" + ColorCodes.ENDC)
            del self._nameToMoteusController[motorName]
            del self._nameToMoteusMotor[motorName]
        except RuntimeError as error:
            self._rosNode.get_logger().info(ColorCodes.FAIL_RED + error.__str__() + ColorCodes.ENDC)
            del self._nameToMoteusController[motorName]
            del self._nameToMoteusMotor[motorName]


    async def startLoop(self):
        """
            The main loop of the whole system.
            Reads/sends data to/from the Moteus controllers.
            
            This can handle the following edge cases
            ----------
            1) Motor faults
                A) set_stop() the motor or
                B) Remove the motor from the list of motors
            2) CAN Bus disconnection
                A) Remove the motor from the list of motors
            3) Reconnect to Moteus Controllers
            
            
        """
        # Connect the motor for the first time
        await self.connectToMoteusControllers()

        
        while self._shouldMoteusThreadLoop:
            if self._shouldReconnect:
                await self.connectToMoteusControllers()
            
            # Go through each Moteus Controller to send data
            for name, controller in self._nameToMoteusController.copy().items():
                moteusMotor = self._nameToMoteusMotor[name]
                
                
                try:
                    # Check for faults
                    resultFromMoteus = await asyncio.wait_for(controller.query(), self._GENERAL_TIMEOUT)
                    
                    if resultFromMoteus.values[moteus.Register.FAULT] != 0:
                        self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "FAULT CODE: " + str(resultFromMoteus.values[moteus.Register.FAULT]) + " FOR\"" + name + "\"(CANID " + str(moteusMotor.canID) + ")" + ColorCodes.ENDC)
                        await self.tryToShutdownMotor(name)
                        continue
                    
                    if moteusMotor.setStop is True:
                        
                        await asyncio.wait_for(controller.set_stop(), self._GENERAL_TIMEOUT)
                        
                    else:
                            
                            resultFromMoteus = await asyncio.wait_for(controller.set_position(
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
                                query = True
                            ), self._GENERAL_TIMEOUT)
                            
                            moteusMotor.publishData(resultFromMoteus)
                            
                            
                        
                except asyncio.TimeoutError:
                    self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "FAILED TO SEND/READ DATA TO MOTEUS MOTOR: \"" + name + "\"(CANID " + str(moteusMotor.canID) + ") CAN-FD bus discconected?" + ColorCodes.ENDC)
                    del self._nameToMoteusController[name]
                    del self._nameToMoteusMotor[name]
                    
                    
            await asyncio.sleep(0.02)
            
        # When we exit the while loop, via ctrl-c, we set_stop() all the motors
        # Watch out for the arm
        for name, controller in self._nameToMoteusController.items():
            await controller.set_stop()
        
        
    async def connectToMoteusControllers(self):
        """
            Connect to the Moteus motors.
            There is a timeout until we give up trying to connect
        """
        self._nameToMoteusController = {}
        self._shouldReconnect = False
        
        for key, moteusMotor in self._nameToMoteusMotor.items():
            controller = moteus.Controller(moteusMotor.canID)
            
            
            try:
                # Reset the controller
                self._rosNode.get_logger().info("Connecting to motor with name: " + str(moteusMotor.name))
                await asyncio.wait_for(controller.query(), timeout = self._CONNECTION_TIMEOUT_IN_SECONDS) # idk why, but wait_for(set_stop()) always works,
                self._nameToMoteusController[key] = controller
                await controller.set_stop()
                self._rosNode.get_logger().info(ColorCodes.GREEN_OK + "Moteus motor controller connected: \"" + str(moteusMotor.name) + "\"(CANID " + str(moteusMotor.canID) + ")" + ColorCodes.ENDC)
                
            except asyncio.TimeoutError:
                self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "FAILED TO CONNECT TO MOTEUS CONTROLLER WITH CANID " + str(moteusMotor.canID) + ColorCodes.ENDC)
            except RuntimeError as error:
                self._rosNode.get_logger().info(ColorCodes.FAIL_RED + "ERROR WHEN set_stop() IS CALLED. MOST LIKELY CANNOT FIND CANBUS" + ColorCodes.ENDC)
                self._rosNode.get_logger().info(ColorCodes.FAIL_RED + error.with_traceback() + ColorCodes.ENDC)
                

