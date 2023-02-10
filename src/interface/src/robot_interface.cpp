<<<<<<< HEAD
#include <cstdio>
#include <iostream>
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

}

void RobotInterface::moveBackward(const double amount) {

}

void RobotInterface::turnLeft(const double amount) {

}

void RobotInterface::turnRight(const double amount) {

}

// Left front wheel
void RobotInterface::leftFrontWheelForward(const double amount) {

}

void RobotInterface::leftFrontWheelBackward(const double amount) {

}

// Right front wheel
void RobotInterface::rightFrontWheelForward(const double amount) {

}

void RobotInterface::rightFrontWheelBackward(const double amount) {

}

// Left middle wheel
void RobotInterface::leftMiddleWheelForward(const double amount) {

}

void RobotInterface::leftMiddleWheelBackward(const double amount) {

}

// Right middle wheel
void RobotInterface::rightMiddleWheelForward(const double amount) {

}

void RobotInterface::rightMiddleWheelBackward(const double amount) {

}

// Left back wheel
void RobotInterface::leftBackWheelForward(const double amount) {

}

void RobotInterface::leftBackWheelBackward(const double amount) {

}

// Right back wheel
void RobotInterface::rightBackWheelForward(const double amount) {

}

void RobotInterface::rightBackWheelBackward(const double amount) {

}

/*
 * Arm movement/rotation
 */

// Base turntable
void RobotInterface::baseRotationLeft(const double amount) {

}

void RobotInterface::baseRotationRight(const double amount) {

}

// Arm 1 is closest to base and arm 3 is furthest from base

// Arm 1 movement
void RobotInterface::arm1Extend(const double amount) {

}

void RobotInterface::arm1Retract(const double amount) {

}

// Arm 2 movement
void RobotInterface::arm2Extend(const double amount) {

}

void RobotInterface::arm2Retract(const double amount) {

}

// Arm 3 movement
void RobotInterface::arm3Extend(const double amount) {

}

void RobotInterface::arm3Retract(const double amount) {

}

// Antenna movement
void RobotInterface::antennaExtend() {

}

void RobotInterface::antennaRetract() {

}

void RobotInterface::antennaTurnLeft(const double amount) {

}

void RobotInterface::antennaTurnRight(const double amount) {
    
}
||||||| parent of 8f38ccc (Created robot_interface.cpp file)
=======
#include <cstdio>
#include <iostream>
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

}

void RobotInterface::moveBackward(const double amount) {

}

void RobotInterface::turnLeft(const double amount) {

}

void RobotInterface::turnRight(const double amount) {

}

// Left front wheel
void RobotInterface::leftFrontWheelForward(const double amount) {

}

void RobotInterface::leftFrontWheelBackward(const double amount) {

}

// Right front wheel
void RobotInterface::rightFrontWheelForward(const double amount) {

}

void RobotInterface::rightFrontWheelBackward(const double amount) {

}

// Left middle wheel
void RobotInterface::leftMiddleWheelForward(const double amount) {

}

void RobotInterface::leftMiddleWheelBackward(const double amount) {

}

// Right middle wheel
void RobotInterface::rightMiddleWheelForward(const double amount) {

}

void RobotInterface::rightMiddleWheelBackward(const double amount) {

}

// Left back wheel
void RobotInterface::leftBackWheelForward(const double amount) {

}

void RobotInterface::leftBackWheelBackward(const double amount) {

}

// Right back wheel
void RobotInterface::rightBackWheelForward(const double amount) {

}

void RobotInterface::rightBackWheelBackward(const double amount) {

}

/*
 * Arm movement/rotation
 */

// Base turntable
void RobotInterface::baseRotationLeft(const double amount) {

}

void RobotInterface::baseRotationRight(const double amount) {

}

// Arm 1 is closest to base and arm 3 is furthest from base

// Arm 1 movement
void RobotInterface::arm1Extend(const double amount) {

}

void RobotInterface::arm1Retract(const double amount) {

}

// Arm 2 movement
void RobotInterface::arm2Extend(const double amount) {

}

void RobotInterface::arm2Retract(const double amount) {

}

// Arm 3 movement
void RobotInterface::arm3Extend(const double amount) {

}

void RobotInterface::arm3Retract(const double amount) {

}
>>>>>>> 8f38ccc (Created robot_interface.cpp file)
