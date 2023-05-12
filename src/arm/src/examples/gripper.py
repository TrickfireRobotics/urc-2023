import serial 
import asyncio
from XInput import *

class Gripper:
    def __init__(self):
        self.extention = 0
        self.rotation = 0
        self.ser = serial.Serial("COM6", 115200)
        set_deadzone(DEADZONE_TRIGGER,10) #setup a deadzone for the thumbsticks to avoid stick driftS



    async def writeToMicrocontroller(self):
        print("Starting to write to microcontroller")
        while True:
            command = b"c " +  str(int(self.rotation)).encode() + b" " + str(int(self.extention)).encode() + b"\n"
            print(command.decode("utf-8"))
            self.ser.write(command)
            resp = self.ser.readline()
            print(resp.decode("utf-8"))
            resp = resp.split(b"=")
            resp = resp[1]
            resp = resp.strip()
            resp = int(resp)
            self.extention = resp
            resp = self.ser.readline()
            print(resp.decode("utf-8"))
            resp = resp.split(b"=")
            resp = resp[1]
            resp = resp.strip()
            resp = int(resp)
            self.rotation = resp
            await asyncio.sleep(0.01)

    async def readController(self):
        print("Starting to read controller")
        currentStick = (0,0)
        while True:
            # update the controller data
            events = get_events()
            for event in events:
            # thumb stick has been moved
                if event.type == EVENT_STICK_MOVED:
                    if event.stick == RIGHT:
                        currentStick = (event.x, event.y)
            self.extention += round(currentStick[1] * 10,0)
            self.rotation += round(currentStick[0] * 10,0)
            await asyncio.sleep(0.01)

    async def main(self):
        await asyncio.gather(
            self.readController(),
            self.writeToMicrocontroller()
            
        )

        




if __name__ == '__main__':
    program = Gripper()
    asyncio.run(Gripper.main(program))