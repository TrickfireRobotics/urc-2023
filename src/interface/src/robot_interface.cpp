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
    robotPublishers['leftFrontWheel'] = publisher_;
    publisher_ = this->create_publisher<float>("rightFrontWheel_Velocity_from_interface", 10);
    robotPublishers['rightFrontWheel'] = publisher_;
    publisher_ = this->create_publisher<float>("leftMiddleWheel_Velocity_from_interface", 10);
    robotPublishers['leftMiddleWheel'] = publisher_;
    publisher_ = this->create_publisher<float>("rightMiddleWheel_Velocity_from_interface", 10);
    robotPublishers['rightMiddleWheel'] = publisher_;
    publisher_ = this->create_publisher<float>("leftBackWheel_Velocity_from_interface", 10);
    robotPublishers['leftBackWheel'] = publisher_;
    publisher_ = this->create_publisher<float>("rightBackWheel_Velocity_from_interface", 10);
    robotPublishers['rightBackWheel'] = publisher_;
    publisher_ = this->create_publisher<float>("baseRotation_Position_from_interface", 10);
    robotPublishers['baseRotation'] = publisher_;
    publisher_ = this->create_publisher<float>("arm1_Position_from_interface", 10);
    robotPublishers['arm1'] = publisher_;
    publisher_ = this->create_publisher<float>("arm2_Position_from_interface", 10);
    robotPublishers['arm2'] = publisher_;
    publisher_ = this->create_publisher<float>("arm3_Position_from_interface", 10);
    robotPublishers['arm3'] = publisher_;
    publisher_ = this->create_publisher<float>("antenna_Position_from_interface", 10);
    robotPublishers['antenna'] = publisher_;
    publisher_ = this->create_publisher<float>("antennaTurntable_Position_from_interface", 10);
    robotPublishers['antennaTurntable'] = publisher_;
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
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['leftFrontWheel']
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing leftFrontWheel: " + message.data << endl;
}

// Right front wheel
void RobotInterface::rightFrontWheel(const double amount) {
    cout << "Testing rightFrontWheel" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['rightFrontWheel'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing rightFrontWheel: " + message.data << endl;
}

// Left middle wheel
void RobotInterface::leftMiddleWheel(const double amount) {
    cout << "Testing leftMiddleWheel" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['leftMiddleWheel'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing leftMiddleWheel: " + message.data << endl;
}

// Right middle wheel
void RobotInterface::rightMiddleWheel(const double amount) {
    cout << "Testing rightMiddleWheel" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['rightMiddleWheel'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing rightMiddleWheel: " + message.data << endl;
}

// Left back wheel
void RobotInterface::leftBackWheel(const double amount) {
    cout << "Testing leftBackWheel" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['leftBackWheel'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing leftBackWheel: " + message.data << endl;
}

// Right back wheel
void RobotInterface::rightBackWheel(const double amount) {
    cout << "Testing rightBackWheel" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['rightBackWheel'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing rightBackWheel: " + message.data << endl;
}

/*
 * Arm movement/rotation
 */

// Base turntable
void RobotInterface::baseRotation(const double amount) {
    cout << "Testing baseRotation" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['baseRotation'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing baseRotation: " + message.data << endl;
}

// Arm 1 is closest to base and arm 3 is furthest from base

// Arm 1 movement
void RobotInterface::arm1(const double amount) {
    cout << "Testing arm1" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm1'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm1: " + message.data << endl;
}

// Arm 2 movement
void RobotInterface::arm2(const double amount) {
    cout << "Testing arm2" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm2'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm2: " + message.data << endl;
}

// Arm 3 movement
void RobotInterface::arm3(const double amount) {
    cout << "Testing arm3" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm3'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm3: " + message.data << endl;
}

// Antenna movement
void RobotInterface::antennaExtend() {
    cout << "Testing antenna" << endl;

    // Output is in "revolutions"
    doublerevolutionsOutput = 1; // we don't know the actual position it should be. this is a placeholder

    publisher_ = robotPublishers['antenna'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing antenna: " + message.data << endl;
}

void RobotInterface::antennaRetract() {
    cout << "Testing antenna" << endl;

    // Output is in "revolutions"
    doublerevolutionsOutput = 0; // we don't know the actual position it should be. this is a placeholder

    publisher_ = robotPublishers['antenna'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing antenna: " + message.data << endl;
}

void RobotInterface::antennaTurntable(const double amount) {
    cout << "Testing antennaTurntable" << endl;

    // Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360

    publisher_ = robotPublishers['antennaTurntable'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing antennaTurntable: " + message.data << endl;
}
