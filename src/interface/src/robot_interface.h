#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include <unordered_map>

using namespace std::chrono_literals;

/**
 * General interface for robot funcitonality
 * (Functions with actions return bool status code)
*/
class RobotInterface : public rclcpp::Node
{
    public:
        /*
        * Constructors/Destructors
        */

        Node selfNode;

        RobotInterface::RobotInterface(Node rosNode);
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
        void leftFrontWheel(const double amount);

        // Right front wheel
        void rightFrontWheel(const double amount);

        // Left middle wheel
        void leftMiddleWheel(const double amount);

        // Right middle wheel
        void rightMiddleWheel(const double amount);

        // Left back wheel
        void leftBackWheel(const double amount);

        // Right back wheel
        void rightBackWheel(const double amount);

        /*
        * Arm movement/rotation
        */

        // Base turntable
        void baseRotation(const double amount);

        // Arm 1 is closest to base and arm 3 is furthest from base

        // Arm 1 movement
        void arm1(const double amount);

        // Arm 2 movement
        void arm2(const double amount);

        // Arm 3 movement
        void arm3(const double amount);

        // Antenna movement
        void antennaExtend();
        void antennaRetract();
        void antennaTurntable(const double amount);

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        unordered_map<string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> robotPublishers;   
};

#endif
