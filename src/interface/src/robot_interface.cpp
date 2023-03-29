#include <cstdio>
#include <iostream>
#include <unordered_map>
#include "robot_interface.h"
using namespace std;

/*
 * Constructors/Destructors
 */
RobotInterface::RobotInterface() {

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
    cout << "Testing moveForward" << endl;
}

void RobotInterface::moveBackward(const double amount) {

}

void RobotInterface::turnLeft(const double amount) {

}

void RobotInterface::turnRight(const double amount) {

}

// Left front wheel
void RobotInterface::leftFrontWheel(const double amount) {

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
