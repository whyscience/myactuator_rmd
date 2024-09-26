#include <cstdlib>
#include <iostream>
#include <unistd.h>

#include <myactuator_rmd/myactuator_rmd.hpp>


int main() {
    myactuator_rmd::CanDriver driver {"can0"};
    std::cout << "init ActuatorInterface" << std::endl;
    myactuator_rmd::ActuatorInterface actuator {driver, 1};

    //sudo ip link set can0 up type can bitrate 1000000
    std::cout << "getVersionDate " << std::endl;
    std::cout << actuator.getVersionDate() << std::endl;
    std::cout << "sendPositionAbsoluteSetpoint: " << actuator.sendPositionAbsoluteSetpoint(180.0, 500.0) << std::endl;
    sleep(3);
    std::cout << "sendVelocitySetpoint: " << actuator.sendVelocitySetpoint(100.0) << std::endl;
    sleep(3);
    actuator.shutdownMotor();
    return EXIT_SUCCESS;
}