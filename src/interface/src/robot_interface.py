# General interface for robot funcitonality

class RobotInterface:
    #
    # Constructors/Destructors
    #
    
    # RobotInterface();
    # ~RobotInterface(){};
    # RobotInterface(const RobotInterface&) = delete;
    # RobotInterface& operator=(const RobotInterface&) = delete;
    # RobotInterface(RobotInterface&&) = delete;
    # RobotInterface& operator=(RobotInterface&&) = delete;


    #
    # Movement
    #

    # General movement including all wheels
    def moveForward(self, amount):
        print()
    def moveBackward(self, amount):
        print()
    def turnLeft(self, amount):
        print()
    def turnRight(self, amount):
        print()

    # Left front wheel
    def leftFrontWheelForward(self, amount):
        print()
    def leftFrontWheelBackward(self, amount):
        print()

    # Right front wheel
    def rightFrontWheelForward(self, amount):
        print()
    def rightFrontWheelBackward(self, amount):
        print()

    # Left middle wheel
    def leftMiddleWheelForward(self, amount):
        print()
    def leftMiddleWheelBackward(self, amount):
        print()

    # Right middle wheel
    def rightMiddleWheelForward(self, amount):
        print()
    def rightMiddleWheelBackward(self, amount):
        print()

    # Left back wheel
    def leftBackWheelForward(self, amount):
        print()
    def leftBackWheelBackward(self, amount):
        print()

    # Right back wheel
    def rightBackWheelForward(self, amount):
        print()
    def rightBackWheelBackward(self, amount):
        print()

    #
    # Arm movement/rotation
    #

    # Base turntable
    def baseRotationLeft(self, amount):
        print()
    def baseRotationRight(self, amount):
        print()

    # Arm 1 is closest to base and arm 3 is furthest from base

    # Arm 1 movement
    def arm1Extend(self, amount):
        print()
    def arm1Retract(self, amount):
        print()

    # Arm 2 movement
    def arm2Extend(self, amount):
        print()
    def arm2Retract(self, amount):
        print()

    # Arm 3 movement
    def arm3Extend(self, amount):
        print()
    def arm3Retract(self, amount):
        print()

    # Atenna movement
    def antennaExtend(self):
        print()
    def antennaRetract(self):
        print()
    def antennaTurnLeft(self, amount):
        print()
    def antennaTurnRight(self, amount):
        print()
