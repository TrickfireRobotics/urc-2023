from i2cpy import I2C


class I2CBus:
    def __init__(self, bus_id=None, driver=None, freq=400000):
        # If bus_id is None, default Ch341 driver is used
        self.i2c = I2C(bus_id, driver=driver, freq=freq)
        self.i2c.init()

    def scan(self, addr, data: bytes):
        self.i2c.writeto(addr, data)

    def read(self, addr, length: int) -> bytes:
        return self.i2c.readfrom(addr, length)
