import XInput
import asyncio
import math
import moteus
import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Process
from multiprocessing import Queue

startPosition = [0, 0, 1.5, -3.14, 0, 0, 0]
maxTorque = 5


class display:
    def __init__(self):

        self.fig, self.ax = plot_utils.init_3d_figure()
        self.ax.legend()

        self.disp_ik_Q = Queue()
        self.disp_target_Q = Queue()
        self.reset_flag = 0

    def display_ik(self):
        ik_chain = ikpy.chain.Chain.from_urdf_file(
            "arm_urdf.urdf", active_links_mask=[
                False, True, True, True, True, True, True])
        ik = startPosition
        target_position = ik_chain.forward_kinematics(ik)[:3, 3]

        self.fig.set_figheight(9)
        self.fig.set_figwidth(13)
        ik_chain.plot(ik, self.ax, target=target_position)
        plt.xlim(-0.5, 0.5)
        plt.ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 0.6)
        plt.ion()
        plt.show()

        while (1):
            if self.reset_flag == 1:
                ik_chain = ikpy.chain.Chain.from_urdf_file(
                    "arm_urdf.urdf", active_links_mask=[
                        False, True, True, True, True, True, True])
                ik = startPosition
                self.disp_ik_Q = Queue()
                self.disp_target_Q = Queue()
                target_position = ik_chain.forward_kinematics(ik)[:3, 3]
                self.reset_flag = 0

            ik = self.disp_ik_Q.get()
            target_position = self.disp_target_Q.get()

            self.ax.clear()

            ik_chain.plot(ik, self.ax, target=target_position)

            plt.xlim(-0.5, 0.5)
            plt.ylim(-0.5, 0.5)
            self.ax.set_zlim(0, 0.6)
            self.ax.legend()

            self.fig.canvas.draw()

            self.fig.canvas.flush_events()

    def reset(self):
        self.reset_flag = 1


class inverse_kinematics:
    def __init__(self):

        self.position_Q = Queue()

        self.orientation_Q = Queue()

        self.ikQ = Queue()

        self.ik = startPosition

        self.reset_flag = 0

    def produce_ik(self):
        # initilize the process ik variabiles
        ik_chain = ikpy.chain.Chain.from_urdf_file(
            "arm_urdf.urdf", active_links_mask=[
                False, True, True, True, True, True, True])
        while 1:
            if self.reset_flag == 1:
                ik_chain = ikpy.chain.Chain.from_urdf_file(
                    "arm_urdf.urdf", active_links_mask=[
                        False, True, True, True, True, True, True])
                self.ik = startPosition
                self.reset_flag = 0
                self.position_Q = Queue()
                self.orientation_Q = Queue()
                self.ikQ = Queue()

            if not self.position_Q.empty():
                position = self.position_Q.get()
                orientation = self.orientation_Q.get()
                # reach the correct position first
                #position_ik = ik_chain.inverse_kinematics(
                #    position, initial_position=self.ik)

                # then reach the correct orientation
                # reference -
                # https://github.com/Phylliade/ikpy/blob/master/tutorials/Orientation.ipynb
                # see Orientation and Position section
                self.ik = ik_chain.inverse_kinematics(
                    position, orientation, orientation_mode="all",
                    initial_position=self.ik)
                self.ikQ.put(self.ik)

    def reset(self):
        self.reset_flag = 1


class ControllerTest:
    def __init__(self):
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(
            "arm_urdf.urdf", active_links_mask=[
                False, True, True, True, True, True, True])
        self.ik_process = inverse_kinematics()
        self.display_process = display()
        self.ik = startPosition
        self.l_thumb_stick_pos = (0, 0)
        self.r_thumb_stick_pos = (0, 0)
        self.l_trigger_pos = 0
        self.r_trigger_pos = 0
        self.l_bumper = 0
        self.r_bumper = 0

        self.enabled = False

        self.ik_result_position = self.ik_chain.forward_kinematics(self.ik)
        self.target_position = self.ik_result_position[:3, 3]
        self.target_orientation = self.ik_result_position[:3, :3]
        self.ik_target_motor_position = self.ik
        self.motor_report_position = self.ik
        self.offset = [0, 0, 0, 0, 0, 0]
        self.selected_axis = 0
        # 1 = ik mode, -1 = per axis mode
        self.control_mode = 1

        self.xpos = self.target_position[0]
        self.ypos = self.target_position[1]
        self.zpos = self.target_position[2]
        self.alpha = -1.6
        self.beta = 0
        self.gamma = 0

        # setup a deadzone for the thumbsticks to avoid stick driftS
        XInput.set_deadzone(XInput.DEADZONE_TRIGGER, 10)

        

        self.test = True
        # Connect to two moteus drivers
        if self.test is False:
            #self.c1 = moteus.Controller(1)  # 1 is Turntable
            #self.c2 = moteus.Controller(2)  # 2 is Shoulder
            #self.c3 = moteus.Controller(3)  # 3 is Elbow
            self.c2 = moteus.Controller(6)  # 3 is forarm
            #self.c5 = moteus.Controller(5)  # 3 is Elbow
            #self.c6 = moteus.Controller(6)  # 3 is Elbow

    def reset(self):
        self.ik_chain = ikpy.chain.Chain.from_urdf_file(
            "arm_urdf.urdf", active_links_mask=[
                False, True, True, True, True, True, True])
        self.ik = startPosition
        self.l_thumb_stick_pos = (0, 0)
        self.r_thumb_stick_pos = (0, 0)
        self.l_trigger_pos = 0
        self.r_trigger_pos = 0
        self.l_bumper = 0
        self.r_bumper = 0
        self.ik_result_position = self.ik_chain.forward_kinematics(self.ik)
        self.target_position = self.ik_result_position[:3, 3]
        self.target_orientation = self.ik_result_position[:3, :3]
        self.xpos = self.target_position[0]
        self.ypos = self.target_position[1]
        self.zpos = self.target_position[2]
        self.alpha = -1.6
        self.beta = 0
        self.gamma = 0

    # reference - https://github.com/Zuzu-Typ/XInput-Python
    async def controller_update(self):
        while (1):
            # update the controller data
            events = XInput.get_events()
            for event in events:
                # thumb stick has been moved
                if event.type == XInput.EVENT_STICK_MOVED:
                    if event.stick == XInput.LEFT:
                        # save the position of the left stick
                        self.l_thumb_stick_pos = (int(round(event.x, 0)),
                                                  int(round(event.y, 0)))
                    elif event.stick == XInput.RIGHT:
                        # save the position of the right stick
                        self.r_thumb_stick_pos = (int(round(event.x, 0)),
                                                  int(round(event.y, 0)))
                elif event.type == XInput.EVENT_BUTTON_PRESSED:
                    if event.button == "B":
                        self.enabled = False
                        if self.test is False:
                            #await self.c1.set_stop()
                            #await self.c2.set_stop()
                            #await self.c3.set_stop()
                            await self.c2.set_stop()
                            #await self.c5.set_stop()
                            #await self.c6.set_stop()
                    elif event.button == "A":
                        self.enabled = True
                    elif event.button == "LEFT_SHOULDER":
                        self.l_bumper = 1
                    elif event.button == "RIGHT_SHOULDER":
                        self.r_bumper = 1
                    # elif event.button == "Y":
                        # self.control_mode = self.control_mode * -1
                    elif event.button == "X":
                        self.reset()
                        self.ik_process.reset()
                        self.display_process.reset()
                        continue
                    elif event.button == "DPAD_UP":
                        self.selected_axis += 1
                        if self.selected_axis > 6:
                            self.selected_axis = 0
                    elif event.button == "DPAD_DOWN":
                        self.selected_axis -= 1
                        if self.selected_axis < 0:
                            self.selected_axis = 6
                    #elif event.button == "DPAD_LEFT":
                        #self.offset[self.selected_axis] -= 0.1
                    #elif event.button == "DPAD_RIGHT":
                        #self.offset[self.selected_axis] += 0.1
                elif event.type == XInput.EVENT_BUTTON_RELEASED:
                    if event.button == "LEFT_SHOULDER":
                        self.l_bumper = 0
                    elif event.button == "RIGHT_SHOULDER":
                        self.r_bumper = 0
                elif event.type == XInput.EVENT_TRIGGER_MOVED:
                    if event.trigger == XInput.LEFT:
                        self.l_trigger_pos = event.value
                    elif event.trigger == XInput.RIGHT:
                        self.r_trigger_pos = event.value

            # Accumulate the position values from the controller
            self.xpos -= 0.005 * self.l_thumb_stick_pos[0]
            self.ypos += 0.005 * self.l_thumb_stick_pos[1]
            self.zpos += 0.005 * self.r_trigger_pos
            self.zpos -= 0.005 * self.l_trigger_pos

            self.target_position = [-self.xpos, self.ypos, self.zpos]

            # Accumulate the angle values from the controller
            self.alpha += 0.05 * self.r_thumb_stick_pos[1]
            self.beta += 0.05 * self.r_thumb_stick_pos[0]
            self.gamma = 0
            
            print("Alpha %f" % self.alpha)
            print("Beta %f" % self.beta)
            print("Gamma %f" % self.gamma)
            
            # build the "rotational matrix" in the absolute referential
            # see - https://en.wikipedia.org/wiki/Rotation_matrix
            # specifically the general rotation matrix with improper
            # Euler angles alpha(x), beta(y), and gamma(z)
            # https://github.com/Phylliade/ikpy/blob/master/tutorials/Orientation.ipynb
            # specifically Orientation on a full referential
            aa = math.cos(self.beta)*math.cos(self.gamma)
            ab = math.sin(self.alpha)*math.sin(self.beta)*math.cos(self.gamma)
            -math.cos(self.alpha)*math.sin(self.gamma)
            ac = math.cos(self.alpha)*math.sin(self.beta)*math.cos(self.gamma)
            +math.sin(self.alpha)*math.sin(self.gamma)
            ba = math.cos(self.beta)*math.sin(self.gamma)
            bb = math.sin(self.alpha)*math.sin(self.beta)*math.sin(self.gamma)
            +math.cos(self.alpha)*math.cos(self.gamma)
            bc = math.cos(self.alpha)*math.sin(self.beta)*math.sin(self.gamma)
            -math.sin(self.alpha)*math.cos(self.gamma)
            ca = -math.sin(self.beta)
            cb = math.sin(self.alpha)*math.cos(self.beta)
            cc = math.cos(self.alpha)*math.cos(self.beta)

            self.target_orientation = [[aa, ab, ac],
                                       [ba, bb, bc],
                                       [ca, cb, cc]]

            await asyncio.sleep(0.05)

    async def consume_ik(self):
        first = True
        while (1):
            while self.control_mode == 1:
                self.ik_process.orientation_Q.put(self.target_orientation)
                self.ik_process.position_Q.put(self.target_position)

                # wait for a new ik
                self.ik = self.ik_process.ikQ.get()
                #print(self.ik)

                self.ik_result_position =\
                    self.ik_chain.forward_kinematics(self.ik)[:3, 3]
                self.ik_result_orientation =\
                    self.ik_chain.forward_kinematics(self.ik)[:3, :3]

                await asyncio.sleep(0.05)


    async def command_motors(self):
        while (1):

            # send the commanded position to the driver
            # query is set to True to recieve back telemetry
            if self.enabled:

                # Ik returns output position in radians
                # moteus expects position in number of motor rotor rotations
                # motor command = ik result / 2pi * gear ratio
                # turntable example = ik result /2pi * 32.7 = ik result * 5.204
                # use a negative to get the direction correct
                if self.test != 1:
                    #state1 = await self.c1.set_position(
                    #    position=(self.ik[1] * 5.204)
                    #    + self.offset[0] - (startPosition[1] * 5.204),
                    #    maximum_torque=maxTorque, query=True)
                    #state2 = await self.c2.set_position(
                    #    position=(self.ik[2] * 3.979)
                    #    + self.offset[1] - (startPosition[2] * 3.979),
                    #    maximum_torque=maxTorque, query=True)
                    #state3 = await self.c3.set_position(
                    #    position=(self.ik[3] * 5.204)
                    #    + self.offset[2] - (startPosition[3] * 5.204),
                    #    maximum_torque=maxTorque, query=True)

                    state4 = await self.c2.set_position(
                        position=((self.ik[2] * 3.979)
                        + self.offset[1] - (startPosition[2] * 3.979)),
                        maximum_torque=maxTorque, query=True)

                    #state5 = await self.c5.set_position(
                    #    position=(self.ik[5] * 3.820)
                    #    + self.offset[4] - (startPosition[5] * 3.820),
                    #    maximum_torque=maxTorque, query=True)
                    #state6 = await self.c6.set_position(
                    #    position=(self.ik[6] / 6.283)
                    #    + self.offset[5] - (startPosition[6] / 6.283),
                    #    maximum_torque=maxTorque, query=True)

                # Read the actual position back from the recieved state
                    #self.motor_report_position[0] =\
                    #    state2.values[moteus.Register.POSITION]
                    #self.motor_report_position[1] =\
                    #    state1.values[moteus.Register.POSITION]
                    #self.motor_report_position[2] =\
                    #    state3.values[moteus.Register.POSITION]
                    self.motor_report_position[1] =\
                        state4.values[moteus.Register.POSITION]
                    #self.motor_report_position[4] =\
                    #    state5.values[moteus.Register.POSITION]
                    #self.motor_report_position[5] =\
                    #    state6.values[moteus.Register.POSITION]
                    # print motor 4's position
                    #print("Motor 4 position: %f" % self.motor_report_position[1])
            await asyncio.sleep(0.01)

    async def display_arm(self):
        while (1):
            self.display_process.disp_ik_Q.put(self.ik)
            self.display_process.disp_target_Q.put(self.target_position)

            await asyncio.sleep(0.25)

    async def main(self):
        # In case the controller had faulted previously, at the start of
        # this script we send the stop command in order to clear it.
        if self.test != 1:
            #await self.c1.set_stop()
            #await self.c2.set_stop()
            #await self.c3.set_stop()
            await self.c2.set_stop()
            #await self.c5.set_stop()
            #await self.c6.set_stop()

            # start all motors at NAN

            state = await self.c2.set_position(
                math.nan, maximum_torque=maxTorque, query=True)
            
            self.motor_report_position[1] =\
                            state.values[moteus.Register.POSITION]

            self.offset[1] = self.motor_report_position[1]
            
            state = await self.c2.set_position(
                position=self.offset[1], maximum_torque=maxTorque, query=True)
            
            self.motor_report_position[1] =\
                            state.values[moteus.Register.POSITION]
                            
            await self.c2.set_stop()

            # print the motor position
            print("Motor 4 initial position: %f" % self.offset[1])

            

        # wait after stop command before sending the next command
        await asyncio.sleep(0.5)

        ik_producer = Process(target=self.ik_process.produce_ik)
        ik_producer.start()

        ik_display = Process(target=self.display_process.display_ik)
        ik_display.start()

        controller_task = asyncio.create_task(self.controller_update())

        ik_task = asyncio.create_task(self.consume_ik())

        display_task = asyncio.create_task(self.display_arm())

        motor_task = asyncio.create_task(self.command_motors())

        await controller_task

        await ik_task

        await display_task

        await motor_task


if __name__ == '__main__':
    obj = ControllerTest()
    asyncio.run(obj.main())
