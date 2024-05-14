#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

#include "../header/I2CExpander.hpp"

/**
 * 
 * 
 * 
 * 
 * 
 * 
 *             Step Size Resolution
 * --------------------------------------------------
 * |  MS1  |  MS2  |  MS3  |  Microstep Resolution  |
 * |-------|-------|-------|------------------------|
 * |  LOW  |  LOW  |  LOW  |        Full Step       |
 * |  HIGH |  LOW  |  LOW  |        Half Step       |
 * |  LOW  |  HIGH |  LOW  |       Quarter Step     |
 * |  HIGH |  HIGH |  LOW  |       Eighth Step      |
 * |  HIGH |  HIGH |  HIGH |      Sixteenth Step    |
 * --------------------------------------------------
 * 
 * 
 * 
*/
class StepperMotor {

// HIGH = CW, LOW = CCW
enum Direction {CLOCKWISE = 1, COUNTER_CLOCKWISE = 0};


private:
    int id;
    int dir;
    int step;
    int reset; // Sets the zero of the current position
    int ms3; // Used for micro stepping
    int ms2; // Used for micro stepping
    int ms1; // Used for micro stepping
    int enable;



public:

    StepperMotor(int id);

    // Getters
    int getID();
    

    // 
    void stepMotor();
    void setZero();
    void setMicrostepSetting(int ms1, int ms2, int ms3);
    void sleep();
    void wakeup();
    void setDirection(Direction targetDir);



};

#endif STEPPER_MOTOR_HPP