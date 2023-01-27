from XInput import *
import asyncio
import math
import moteus
import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Process
from multiprocessing import Queue

class inverse_kinematics:
    def __init__(self):
    
        self.position_Q = Queue()
              
        self.orientation_Q = Queue()
        
        self.ikQ = Queue()
        
        self.ik = [0,0,0,0,0,0,0]
     
    def produce_ik(self):
        #initilize the process ik variabiles
        ik_chain = ikpy.chain.Chain.from_urdf_file("arm_urdf.urdf",active_links_mask=[False, True, True, True, True, True, True])
        self.ik = ik_chain.inverse_kinematics([0.38,0,0.58])
        while(1):
            if not self.position_Q.empty():
                position = self.position_Q.get()
                orientation = self.orientation_Q.get()
                # reach the correct position first
                position_ik = ik_chain.inverse_kinematics(position, initial_position=self.ik)
        
                # then reach the correct orientation
                # reference - https://github.com/Phylliade/ikpy/blob/master/tutorials/Orientation.ipynb
                # see Orientation and Position section
                self.ik = ik_chain.inverse_kinematics(position, orientation, orientation_mode="all", initial_position=position_ik)
                self.ikQ.put(self.ik)
        


class ControllerTest:
    def __init__(self):
    
        self.ik_chain = ikpy.chain.Chain.from_urdf_file("arm_urdf.urdf",active_links_mask=[False, True, True, True, True, True, True])
        self.ik_process = inverse_kinematics()
        self.ik = [0,0,0,0,0,0,0]

        self.l_thumb_stick_pos = (0,0)
        self.r_thumb_stick_pos = (0,0)
        self.l_trigger_pos = 0
        self.r_trigger_pos = 0
        self.l_bumper = 0
        self.r_bumper = 0
        
        self.enabled = False
        
        self.xpos = 0.38
        self.ypos = 0
        self.zpos = 0.58
        self.alpha = 0
        self.beta = 0
        self.gamma = 0
        
        self.target_position = [0,0,0]
        self.target_orientation = np.eye(3)
        
        self.ik_result_position = [0.38,0,0.58]
        self.ik_target_motor_position = [0,0,0,0,0,0]
        self.motor_report_position = [0,0,0,0,0,0]
        
        self.fig, self.ax = plot_utils.init_3d_figure()
        self.ax.legend()
        set_deadzone(DEADZONE_TRIGGER,10) #setup a deadzone for the thumbsticks to avoid stick driftS
        
        #Connect to two moteus drivers 
        self.c1 = moteus.Controller(1) # 1 is extend retract antenna
        self.c2 = moteus.Controller(2)  # 2 is rotate antenna 


   #reference - https://github.com/Zuzu-Typ/XInput-Python
    async def controller_update(self):
        while True:
            # update the controller data
            events = get_events()
            for event in events:
                # thumb stick has been moved
                if event.type == EVENT_STICK_MOVED:
                    if event.stick == LEFT:
                        # save the position of the left stick into the global variable 
                        self.l_thumb_stick_pos = (int(round(event.x,0)), int(round(event.y,0)))
                    elif event.stick == RIGHT:
                        # save the position of the right stick into the global variable 
                        self.r_thumb_stick_pos = (int(round(event.x,0)), int(round(event.y,0)))
                elif event.type == EVENT_BUTTON_PRESSED:
                    if event.button == "B":
                        self.enabled = False
                        await self.c1.set_stop()
                        await self.c2.set_stop()
                    elif event.button == "A":
                        self.enabled = True
                    elif event.button == "LEFT_SHOULDER":
                        self.l_bumper = 1
                    elif event.button == "RIGHT_SHOULDER":
                        self.r_bumper = 1
                elif event.type == EVENT_BUTTON_RELEASED:
                    if event.button == "LEFT_SHOULDER":
                        self.l_bumper = 0
                    elif event.button == "RIGHT_SHOULDER":
                        self.r_bumper = 0
                elif event.type == EVENT_TRIGGER_MOVED:
                    if event.trigger == LEFT:
                        self.l_trigger_pos = event.value
                    elif event.trigger == RIGHT:
                        self.r_trigger_pos = event.value
            
            #Accumulate the position values from the controller
            self.xpos += 0.01 * self.l_thumb_stick_pos[1]
            self.ypos += 0.01 * self.l_thumb_stick_pos[0]
            self.zpos += 0.01 * self.r_trigger_pos
            self.zpos -= 0.01 * self.l_trigger_pos
            
            self.target_position = [-self.xpos, self.ypos, self.zpos]
            
            #Accumulate the angle values from the controller
            self.alpha += 0.1 * self.r_thumb_stick_pos[0]
            self.beta += 0.1 * self.r_thumb_stick_pos[1]
            self.gamma = 0
            
            # build the "rotational matrix" in the absolute referential see - https://en.wikipedia.org/wiki/Rotation_matrix 
            # specifically the general rotation matrix with improper Euler angles alpha(x), beta(y), and gamma(z)
            # and - https://github.com/Phylliade/ikpy/blob/master/tutorials/Orientation.ipynb specifically Orientation on a full referential
            aa = math.cos(self.beta)*math.cos(self.gamma)
            ab = math.sin(self.alpha)*math.sin(self.beta)*math.cos(self.gamma)-math.cos(self.alpha)*math.sin(self.gamma)
            ac = math.cos(self.alpha)*math.sin(self.beta)*math.cos(self.gamma)+math.sin(self.alpha)*math.sin(self.gamma)
            ba = math.cos(self.beta)*math.sin(self.gamma)
            bb = math.sin(self.alpha)*math.sin(self.beta)*math.sin(self.gamma)+math.cos(self.alpha)*math.cos(self.gamma)
            bc = math.cos(self.alpha)*math.sin(self.beta)*math.sin(self.gamma)-math.sin(self.alpha)*math.cos(self.gamma)
            ca = -math.sin(self.beta)
            cb = math.sin(self.alpha)*math.cos(self.beta)
            cc = math.cos(self.alpha)*math.cos(self.beta)
            
            self.target_orientation = [[aa, ab, ac],
                                      [ba, bb, bc],
                                      [ca, cb, cc]]
            await asyncio.sleep(0.02)
    
    async def consume_ik(self):
        while True:

            print("Commanded Position: %s" % [ '%.2f' % elem for elem in self.target_position ])
            #print("Commanded Orientation: %s" % [ '%.2f' % elem for elem in self.target_orientation ])
            
            self.ik_process.orientation_Q.put(self.target_orientation)
            self.ik_process.position_Q.put(self.target_position)

            #wait for a new ik        
            self.ik = self.ik_process.ikQ.get()

            self.ik_result_position = self.ik_chain.forward_kinematics(self.ik)[:3, 3]
            
            print("Calculated FK(cartesian) position: %s" % [ '%.2f' % elem for elem in self.ik_result_position])
            
            # motor map [hand rotation, turntable, shoulder, elbow, forarm rotation, wrist]
            print("motor map =                   [turntable, shoulder, elbow, forarm rot, wrist, hand rot]")
            print("Calculated IK(motor) position : %s" % [ '%.2f' % elem for elem in self.ik ][1:])
            
            await asyncio.sleep(0.02)
        
    async def command_motors(self):        
        while True:
            # send the commanded position to the driver, with a given maximum_torque
            # query is set to True to recieve back telemetry and save that into state1 and state2
            if self.enabled:
                state1 = await self.c1.set_position(position=self.ik[2] * 8, maximum_torque=15, query=True)
                state2 = await self.c2.set_position(position=self.ik[1] * 8, maximum_torque=15, query=True)

                # Read the actual position back from the recieved state variable
                self.motor_report_position[2] = state1.values[moteus.Register.POSITION]
                self.motor_report_position[1] = state2.values[moteus.Register.POSITION]    
        

            # Print blank line so we can separate one iteration from the
            # next
            print("motor map =                   [turntable, shoulder, elbow, forarm rot, wrist, hand rot]")
            print("Reported motor position : %s" % [ '%.2f' % elem for elem in self.motor_report_position ][1:])

            print()
            await asyncio.sleep(0.01)

    async def display_arm(self):
        while True:
            #some code
            self.ax.clear()
            self.ik_chain.plot(self.ik, self.ax, target=self.target_position)
            plt.xlim(-0.5, 0.5)
            plt.ylim(-0.5, 0.5)
            self.ax.set_zlim(0, 0.6)
            self.ax.legend()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            await asyncio.sleep(0.05)
	
	
    async def main(self):
        # In case the controller had faulted previously, at the start of
        # this script we send the stop command in order to clear it.
        await self.c1.set_stop()
        await self.c2.set_stop()
        
        # wait after stop command before sending the next command
        #await asyncio.sleep(0.5)

        # reset the zero position
        #await self.c1.set_position(position=math.nan, query=False)
        #await self.c2.set_position(position=math.nan, query=False)
        
        # wait before starting the main loop 
        #await asyncio.sleep(0.02)
        
        #await self.c1.set_stop()
        #pythoawait self.c2.set_stop()
		
		#setup the ik figure 
        self.fig.set_figheight(9)  
        self.fig.set_figwidth(13)  
        self.ik_chain.plot(self.ik, self.ax, target=self.target_position)
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 0.6)
        plt.ion()
        plt.show()
        
        # wait before starting the main loop 
        await asyncio.sleep(0.02)
        
        ik_producer = Process(target=self.ik_process.produce_ik)
        ik_producer.start()

        controller_task = asyncio.create_task(self.controller_update())

        ik_task = asyncio.create_task(self.consume_ik())
   
        display_task = asyncio.create_task(self.display_arm())
        
        motor_task = asyncio.create_task(self.command_motors())
        
        await controller_task
                    



if __name__ == '__main__':
    obj = ControllerTest()
    asyncio.run(obj.main())