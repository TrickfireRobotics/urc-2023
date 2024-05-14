#include "../header/stepperMotor.hpp"


StepperMotor::StepperMotor(int id) : id(id), dir(StepperMotor::Direction::COUNTER_CLOCKWISE), step(1), ms1(0), ms2(0), ms3(0), enable(0){




}



void StepperMotor::stepMotor(){

}


void StepperMotor::setZero(){

}


void StepperMotor::setMicrostepSetting(int ms1, int ms2, int ms3){
    StepperMotor::ms1 = ms1;
    StepperMotor::ms2 = ms2;
    StepperMotor::ms3 = ms3;
}



