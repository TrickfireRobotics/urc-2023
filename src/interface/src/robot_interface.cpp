#include <cstdio>
#include <iostream>
#include <unordered_map>
#include "robot_interface.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <publisher.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std;

/*
 * Constructors/Destructors
 */
RobotInterface::RobotInterface(Node rosNode) {
    self.selfNode = rosNode;
    cout << "GOT TO INIT" << endl;
    publisher_ = this->create_publisher<float>("leftFrontWheel_Velocity_from_interface", 10);
    robotPublishers['leftFrontWheel'] = publisher_
    publisher_ = this->create_publisher<float>("rightFrontWheel_Velocity_from_interface", 10);
    robotPublishers['rightFrontWheel'] = publisher_
    publisher_ = this->create_publisher<float>("leftMiddleWheel_Velocity_from_interface", 10);
    robotPublishers['leftMiddleWheel'] = publisher_
    publisher_ = this->create_publisher<float>("rightMiddleWheel_Velocity_from_interface", 10);
    robotPublishers['rightMiddleWheel'] = publisher_
    publisher_ = this->create_publisher<float>("leftBackWheel_Velocity_from_interface", 10);
    robotPublishers['leftBackWheel'] = publisher_
    publisher_ = this->create_publisher<float>("rightBackWheel_Velocity_from_interface", 10);
    robotPublishers['rightBackWheel'] = publisher_
    publisher_ = this->create_publisher<float>("baseRotation_Position_from_interface", 10);
    robotPublishers['baseRotation'] = publisher_
    publisher_ = this->create_publisher<float>("arm1_Position_from_interface", 10);
    robotPublishers['arm1'] = publisher_
    publisher_ = this->create_publisher<float>("arm2_Position_from_interface", 10);
    robotPublishers['arm2'] = publisher_
    publisher_ = this->create_publisher<float>("arm3_Position_from_interface", 10);
    robotPublishers['arm3'] = publisher_
    publisher_ = this->create_publisher<float>("antenna_Position_from_interface", 10);
    robotPublishers['antenna'] = publisher_
    publisher_ = this->create_publisher<float>("antennaTurntable_Position_from_interface", 10);
    robotPublishers['antennaTurntable'] = publisher_
}

RobotInterface::~RobotInterface() {

}

RobotInterface::RobotInterface(const RobotInterface &) = delete;
RobotInterface &RobotInterface::operator=(const RobotInterface &) = delete;
RobotInterface::RobotInterface(RobotInterface &&) = delete;
RobotInterface &RobotInterface::operator=(RobotInterface &&) = delete;


/*
 * Movement
 */

// General movement including all wheels
void RobotInterface::moveForward(const double amount) {
    
}

void RobotInterface::moveBackward(const double amount) {

}

void RobotInterface::turnLeft(const double amount) {

}

void RobotInterface::turnRight(const double amount) {

}

// Left front wheel
void RobotInterface::leftFrontWheel(const double amount) {
    cout << "Testing leftFrontWheel" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount

    publisher_ = robotPublishers['leftFrontWheel']
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing leftFrontWheel: " + message.data << endl;
}

// Right front wheel
void RobotInterface::rightFrontWheel(const double amount) {

}

// Left middle wheel
void RobotInterface::leftMiddleWheel(const double amount) {

}

// Right middle wheel
void RobotInterface::rightMiddleWheel(const double amount) {

}

// Left back wheel
void RobotInterface::leftBackWheel(const double amount) {

}

// Right back wheel
void RobotInterface::rightBackWheel(const double amount) {

}

/*
 * Arm movement/rotation
 */

// Base turntable
void RobotInterface::baseRotation(const double amount) {

}

// Arm 1 is closest to base and arm 3 is furthest from base

// Arm 1 movement
void RobotInterface::arm1(const double amount) {

}

// Arm 2 movement
void RobotInterface::arm2(const double amount) {

}

// Arm 3 movement
void RobotInterface::arm3(const double amount) {

}

// Antenna movement
void RobotInterface::antennaExtend() {

}

void RobotInterface::antennaRetract() {

}

void RobotInterface::antennaTurntable(const double amount) {

}
