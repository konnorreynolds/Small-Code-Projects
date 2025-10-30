//
// Created by konno on 10/26/2025.
//

#ifndef ELEGOO_DEV_KIT_BOARD_DIFFERENTIALDRIVEWRAPPER_H
#define ELEGOO_DEV_KIT_BOARD_DIFFERENTIALDRIVEWRAPPER_H

struct MotorSpeeds {
    double leftSpeed;
    double rightSpeed;
    MotorSpeeds() : leftSpeed(0), rightSpeed(0) {}
};

MotorSpeeds tankDrive(double leftSpeed, double rightSpeed);

#endif //ELEGOO_DEV_KIT_BOARD_DIFFERENTIALDRIVEWRAPPER_H