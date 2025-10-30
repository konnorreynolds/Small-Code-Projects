//
// Created by konno on 10/26/2025.
//

#include "DifferentialDriveWrapper.h"

MotorSpeeds tankDrive(double leftSpeed, double rightSpeed) {
    MotorSpeeds motorSpeeds;
    motorSpeeds.leftSpeed = leftSpeed;
    motorSpeeds.rightSpeed = rightSpeed;
    return motorSpeeds;
}
