from XInput import *
import asyncio
import math
import moteus
import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

class ControllerTest:
    def __init__(self):
        self.currentPosition1 = 0
        self.currentPosition2 = 0
        self.currentPosition3 = 0
        self.currentPosition4 = 0.58
        self.actualPosition1 = 0
        self.actualPosition2 = 0
        self.target_position = [0,0,0.58]
        self.computed_position = [0,0,0.58]
        self.target_orientation =  [-1,0,0]
        self.l_thumb_stick_pos = (0,0)
        self.r_thumb_stick_pos = (0,0)
        self.enabled = False
        self.ik_chain = ikpy.chain.Chain.from_urdf_file("arm_urdf.urdf",active_links_mask=[False, True, True, True, True, True, True])
        self.ik = self.ik_chain.inverse_kinematics(self.target_position,self.target_orientation, orientation_mode="Z")
        self.fig, self.ax = plot_utils.init_3d_figure()
        set_deadzone(DEADZONE_TRIGGER,10) #setup a deadzone for the thumbsticks to avoid stick drift
        
       #Connect to two moteus drivers 
       #self.c1 = moteus.Controller(1) # 1 is extend retract antenna
       #self.c2 = moteus.Controller(2)  # 2 is rotate antenna 


    def bound(self,value, maximum, minimum):
        '''a function that forces 'value' to be greater than 'maximum' and less than 'minimum'
            returns: value
        '''
        if value > maximum:
            value = maximum
        if value < minimum:
            value = minimum
        return value  

   
    async def controller_update(self):
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
                    if event.button == "A":
                        self.enabled = False
                        #await self.c1.set_stoppython
                        #await self.c2.set_stop()
                    elif event.button == "B":
                        self.enabled = True

    async def update_ik(self):
		#some code
        # accumulate the thumbstick value into the current position
        self.currentPosition1 += 0.01 * self.l_thumb_stick_pos[1]
        self.currentPosition2 += 0.001 * self.r_thumb_stick_pos[0]
        self.currentPosition3 += 0.01 * self.l_thumb_stick_pos[0]
        self.currentPosition4 += 0.01 * self.r_thumb_stick_pos[1]
        

        # keep the commanded position close to the actual rotor position so the motor stops in a reasonable time 
        # when the stick is relesased
        #self.currentPosition1 = self.bound(self.currentPosition1, self.computed_position[0] + 1, self.computed_position[0] - 1)
        #self.currentPosition2 = self.bound(self.currentPosition2, self.computed_position[1] + 1, self.computed_position[1] - 1)
            
        # Bound the commanded position witin the motor limits set in the onboard config 
        # TODO: this could be read from the moteus driver
        #self.currentPosition1 = self.bound(self.currentPosition1, 20, -20)
        #self.currentPosition2 = self.bound(self.currentPosition2, 20, -20)

        self.target_position = [-self.currentPosition1, self.currentPosition3,self.currentPosition4]
        self.target_orientation =[self.currentPosition2,0,0]
        # print out the commanded position into the console
        #print(self.currentPosition1)
        #print(self.currentPosition2)
        old_position= self.ik.copy()
        self.ik = self.ik_chain.inverse_kinematics(self.target_position, self.target_orientation, orientation_mode="Z", initial_position=old_position)
        #self.computed_position = self.ik_chain.forward_kinematics(self.ik)[:3, 3]
        #print("Computed position (readable) : %s" % [ '%.2f' % elem for elem in computed_position[:3, 3] ])
        
        
    async def command_motors(self):        
            # send the commanded position to the driver, with a given maximum_torque
            # query is set to True to recieve back telemetry and save that into state1 and state2
            if self.enabled:
                state1 = await self.c1.set_position(position=self.currentPosition1, maximum_torque=15, query=True)
                state2 = await self.c2.set_position(position=self.currentPosition2, maximum_torque=15, query=True)

                # Read the actual position back from the recieved state variable
                self.actualPosition1 = state1.values[moteus.Register.POSITION]
                self.actualPosition2 = state2.values[moteus.Register.POSITION]    
        

            # Print blank line so we can separate one iteration from the
            # next
            print()

    async def display_arm(self):
		#some code
        self.ax.clear()
        self.ik_chain.plot(self.ik, self.ax, target=self.target_position)
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 0.6)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
	
	
    async def main(self):
        # In case the controller had faulted previously, at the start of
        # this script we send the stop command in order to clear it.
        #await self.c1.set_stop()
        #await self.c2.set_stop()
        
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

        # infinite loop
        while True:
			
            await self.controller_update()
			
            await self.update_ik()
            
            await self.display_arm()
           
			
            #async with asyncio.TaskGroup() as tg:
				#task1 = tg.create_task(command_motors())
                #task2 = tg.create_task(self.display_arm())
  
                
            # Wait 20ms between iterations.  By default, when commanded
            # over CAN, there is a watchdog which requires commands to be
            # sent at least every 100ms or the co/ntroller will enter a
            # latched fault state.
            await asyncio.sleep(0.02)


if __name__ == '__main__':
    obj = ControllerTest()
    asyncio.run(obj.main())