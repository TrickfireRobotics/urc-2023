#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

/**
 * General interface for robot funcitonality
 * (Functions with actions return bool status code)
*/
class RobotInterface
{
    public:
        /*
        * Constructors/Destructors
        */
        RobotInterface();
        ~RobotInterface(){};
        RobotInterface(const RobotInterface&) = delete;
        RobotInterface& operator=(const RobotInterface&) = delete;
        RobotInterface(RobotInterface&&) = delete;
        RobotInterface& operator=(RobotInterface&&) = delete;

        /*
        * Movement
        */

        // General movement including all wheels
        bool moveForward(const double amount);
        bool moveBackward(const double amount);
        bool turnLeft(const double amount);
        bool turnRight(const double amount);

        // Left front wheel
        bool leftFrontWheelForward(const double amount);
        bool leftFrontWheelBack(const double amount);

        // Right front wheel
        bool rightFrontWheelForward(const double amount);
        bool rightFrontWheelBack(const double amount);

        // Left middle wheel
        bool leftMiddleWheelForward(const double amount);
        bool leftMiddleWheelBack(const double amount);

        // Right middle wheel
        bool rightMiddleWheelForward(const double amount);
        bool rightMiddleWheelBack(const double amount);

        // Left back wheel
        bool leftBackWheelForward(const double amount);
        bool leftBackWheelBack(const double amount);

        // Right back wheel
        bool rightBackWheelForward(const double amount);
        bool rightBackWheelBack(const double amount);

        /*
        * Arm movement/rotation
        */

        // Base turntable
        bool baseRotation(const double amount);

        // Base joint
        // ADD SOMETHING HERE FOR JOINT

        // Arm 1 rotation
        bool arm1Extend(const double amount);
        bool arm1Retract(const double amount);

        // Arm 2 rotation
        bool arm2Extend(const double amount);
        bool arm2Retract(const double amount);

        // Arm 3 rotation
        bool arm3Extend(const double amount);
        bool arm3Retract(const double amount);
};

#endif