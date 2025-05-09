from i2c_bus import I2CBus

class ArduinoNanoEvery:
    def __init__(self, bus: I2CBus, address: int = 0x10):
        self.bus = bus
        self.addr = address

    def send_command(self, cmd_id: int, payload: bytes = b''):
        # Prefix messages with a command byte, e.g. 0x01 = "ping"
        msg = bytes([cmd_id]) + payload
        self.bus.write(self.addr, msg)