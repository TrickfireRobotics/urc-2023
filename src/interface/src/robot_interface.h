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
        void moveForward(const double amount);
        void moveBackward(const double amount);
        void turnLeft(const double amount);
        void turnRight(const double amount);

        // Left front wheel
        void leftFrontWheelForward(const double amount);
        void leftFrontWheelBackward(const double amount);

        // Right front wheel
        void rightFrontWheelForward(const double amount);
        void rightFrontWheelBackward(const double amount);

        // Left middle wheel
        void leftMiddleWheelForward(const double amount);
        void leftMiddleWheelBackward(const double amount);

        // Right middle wheel
        void rightMiddleWheelForward(const double amount);
        void rightMiddleWheelBackward(const double amount);

        // Left back wheel
        void leftBackWheelForward(const double amount);
        void leftBackWheelBackward(const double amount);

        // Right back wheel
        void rightBackWheelForward(const double amount);
        void rightBackWheelBackward(const double amount);

        /*
        * Arm movement/rotation
        */

        // Base turntable
        void baseRotationLeft(const double amount);
        void baseRotationRight(const double amount);

        // Arm 1 is closest to base and arm 3 is furthest from base

        // Arm 1 movement
        void arm1Extend(const double amount);
        void arm1Retract(const double amount);

        // Arm 2 movement
        void arm2Extend(const double amount);
        void arm2Retract(const double amount);

        // Arm 3 movement
        void arm3Extend(const double amount);
        void arm3Retract(const double amount);
};

#endif