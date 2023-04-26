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
    
    // Drive base
    publisher_ = this->create_publisher<float>("front_left_wheel_motor_velocity_from_interface", 10);
    robotPublishers['front_left_wheel_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("front_right_wheel_motor_velocity_from_interface", 10);
    robotPublishers['front_right_wheel_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("mid_left_wheel_motor_velocity_from_interface", 10);
    robotPublishers['mid_left_wheel_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("mid_right_wheel_motor_velocity_from_interface", 10);
    robotPublishers['mid_right_wheel_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("rear_left_wheel_motor_velocity_from_interface", 10);
    robotPublishers['rear_left_wheel_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("rear_right_wheel_motor_velocity_from_interface", 10);
    robotPublishers['rear_right_wheel_motor'] = publisher_;

    // Arm
    publisher_ = this->create_publisher<float>("arm_turntable_motor_position_from_interface", 10);
    robotPublishers['arm_turntable_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("arm_shoulder_motor_position_from_interface", 10);
    robotPublishers['arm_shoulder_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("arm_elbow_motor_position_from_interface", 10);
    robotPublishers['arm_elbow_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("arm_forearm_motor_position_from_interface", 10);
    robotPublishers['arm_forearm_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("arm_wrist_motor_position_from_interface", 10);
    robotPublishers['arm_wrist_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("arm_hand_motor_position_from_interface", 10);
    robotPublishers['arm_hand_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("arm_fingers_motor_position_from_interface", 10);
    robotPublishers['arm_fingers_motor'] = publisher_;

    // Antenna
    publisher_ = this->create_publisher<float>("antenna_motor_position_from_interface", 10);
    robotPublishers['antenna_motor'] = publisher_;
    publisher_ = this->create_publisher<float>("antenna_turntable_motor_position_from_interface", 10);
    robotPublishers['antenna_turntable_motor'] = publisher_;
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
void RobotInterface::front_left_wheel_motor(const double amount) {
    cout << "Testing front_left_wheel_motor" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['front_left_wheel_motor']
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing front_left_wheel_motor: " + message.data << endl;
}

// Right front wheel
void RobotInterface::front_right_wheel_motor(const double amount) {
    cout << "Testing front_right_wheel_motor" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['front_right_wheel_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing front_right_wheel_motor: " + message.data << endl;
}

// Left middle wheel
void RobotInterface::mid_left_wheel_motor(const double amount) {
    cout << "Testing mid_left_wheel_motor" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['mid_left_wheel_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing mid_left_wheel_motor: " + message.data << endl;
}

// Right middle wheel
void RobotInterface::mid_right_wheel_motor(const double amount) {
    cout << "Testing mid_right_wheel_motor" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['mid_right_wheel_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing mid_right_wheel_motor: " + message.data << endl;
}

// Left back wheel
void RobotInterface::rear_left_wheel_motor(const double amount) {
    cout << "Testing rear_left_wheel_motor" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['rear_left_wheel_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing rear_left_wheel_motor: " + message.data << endl;
}

// Right back wheel
void RobotInterface::rear_right_wheel_motor(const double amount) {
    cout << "Testing rear_right_wheel_motor" << endl;

    //Converting from input "meters per second" to output "revolutions per second"
    double revolutionsOutput = 1.76838724851 * amount;

    publisher_ = robotPublishers['rear_right_wheel_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing rear_right_wheel_motor: " + message.data << endl;
}

/*
 * Arm movement/rotation
 */

// Arm turntable
void RobotInterface::arm_turntable_motor(const double amount) {
    cout << "Testing arm_turntable_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_turntable_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_turntable_motor: " + message.data << endl;
}

// Arm 1 is closest to base and arm 3 is furthest from base

// Arm 1 movement
void RobotInterface::arm_shoulder_motor(const double amount) {
    cout << "Testing arm_shoulder_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_shoulder_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_shoulder_motor: " + message.data << endl;
}

// Arm 2 movement
void RobotInterface::arm_elbow_motor(const double amount) {
    cout << "Testing arm_elbow_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_elbow_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_elbow_motor: " + message.data << endl;
}

// Arm 3 movement
void RobotInterface::arm_forearm_motor(const double amount) {
    cout << "Testing arm_forearm_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_forearm_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_forearm_motor: " + message.data << endl;
}

// Arm wrist movement
void RobotInterface::arm_wrist_motor(const double amount) {
    cout << "Testing arm_wrist_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_wrist_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_wrist_motor: " + message.data << endl;
}

// Arm hand movement
void RobotInterface::arm_hand_motor(const double amount) {
    cout << "Testing arm_hand_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_hand_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_hand_motor: " + message.data << endl;
}

// Arm fingers movement
void RobotInterface::arm_fingers_motor(const double amount) {
    cout << "Testing arm_fingers_motor" << endl;

    //Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360;

    publisher_ = robotPublishers['arm_fingers_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing arm_fingers_motor: " + message.data << endl;
}

// antenna_motor movement
void RobotInterface::antenna_motor_extend() {
    cout << "Testing antenna_motor" << endl;

    // Output is in "revolutions"
    doublerevolutionsOutput = 1; // we don't know the actual position it should be. this is a placeholder

    publisher_ = robotPublishers['antenna_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing antenna_motor: " + message.data << endl;
}

void RobotInterface::antenna_motor_retract() {
    cout << "Testing antenna_motor" << endl;

    // Output is in "revolutions"
    doublerevolutionsOutput = 0; // we don't know the actual position it should be. this is a placeholder

    publisher_ = robotPublishers['antenna_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing antenna_motor: " + message.data << endl;
}

void RobotInterface::antenna_turntable_motor(const double amount) {
    cout << "Testing antenna_turntable_motor" << endl;

    // Converting from input "degrees" to output "revolutions"
    double revolutionsOutput = amount / 360

    publisher_ = robotPublishers['antenna_turntable_motor'];
    auto message = std_msgs::msg::String();
    message.data = std::to_string(revolutionsOutput);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    cout << "Publishing antenna_turntable_motor: " + message.data << endl;
}
