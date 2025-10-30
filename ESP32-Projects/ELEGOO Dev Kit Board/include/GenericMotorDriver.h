//
// Created by konno on 10/26/2025.
//

#ifndef ELEGOO_DEV_KIT_BOARD_MOTOR_DRIVER_H
#define ELEGOO_DEV_KIT_BOARD_MOTOR_DRIVER_H

// All Project Includes.
#include <Arduino.h>

// Struct defining a motor.
struct Motor {
    int forwardPin;
    int reversePin;
    int forwardChannel;  // Add PWM channels
    int reverseChannel;

    // Default initializes, -1 is unusable.
    Motor() : forwardPin(-1), reversePin(-1), forwardChannel(-1), reverseChannel(-1) {}

    // Initializes pins with the ADC for analog control.
    Motor(int fwdPin, int revPin, int fwdCh, int revCh)
        : forwardPin(fwdPin), reversePin(revPin),
          forwardChannel(fwdCh), reverseChannel(revCh) {
        // Setup LEDC channels
        ledcSetup(forwardChannel, 5000, 8);  // 5 kHz, 8-bit 0-255.
        ledcSetup(reverseChannel, 5000, 8);
        ledcAttachPin(forwardPin, forwardChannel);
        ledcAttachPin(reversePin, reverseChannel);
    }
};

void dutyCycleControl(Motor motor, double dutyCycle);

#endif //ELEGOO_DEV_KIT_BOARD_MOTOR_DRIVER_H