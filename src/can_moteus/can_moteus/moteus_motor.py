

class MoteusMotor():

    def __init__(self, canID, name):
        self.canID = canID
        self.name = name

        # The settings we can send to the moteus controller
        # using the make_position() method
        self.position = -1
        self.velocity = -1
        self.feedfoward_torque = -1
        self.kp_scale = -1
        self.kd_scale = -1
        self.max_torque = -1
        self.watchdog_timeout = -1
        self.velocity_limit = -1
        self.accel_limit = -1
        self.fixed_votlage_override = -1
        self.ilimit_scale = -1
