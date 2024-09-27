#include <cstdlib>
#include <iostream>

#include <myactuator_rmd/myactuator_rmd.hpp>


int main() {
  myactuator_rmd::CanDriver driver {"can0"};
  myactuator_rmd::ActuatorInterface actuator {driver, 1};

  std::cout << actuator.getVersionDate() << std::endl;
  std::cout << actuator.sendPositionAbsoluteSetpoint(180.0, 500.0) << std::endl;
  actuator.shutdownMotor();
  return EXIT_SUCCESS;
}