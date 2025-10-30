//
// Created by Konnor on 10/26/2025.
// Motor Driver for basic Gate Driver ICs with Int1 and Int2 pins for forward and reverse control.
//

#include "GenericMotorDriver.h"

void dutyCycleControl(Motor motor, double dutyCycle) {
    if (dutyCycle < 0) {
        // If dutyCycle is negative, go in reverse. Set forward channel to 0.
        ledcWrite(motor.reverseChannel, round(-dutyCycle * 255));
        ledcWrite(motor.forwardChannel, 0);
    } else if (dutyCycle > 0) {
        // If dutyCycle is positive, go forward. Set reverse channel to 0.
        ledcWrite(motor.forwardChannel, round(dutyCycle * 255));
        ledcWrite(motor.reverseChannel, 0);
    } else {
        ledcWrite(motor.forwardChannel, 0);
        ledcWrite(motor.reverseChannel, 0);
    }
}