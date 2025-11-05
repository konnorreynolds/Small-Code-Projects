// ============================================================================
// Simple Motor Control - No Sensor Feedback
// ============================================================================
// Learn the basics: control a motor without any encoders or sensors.
// Just set the speed and let it run!
//
// What you'll learn:
// - How to create a motor controller
// - How to set motor speed (duty cycle)
// - How to make timed movements
// - How to ramp speed up/down
// ============================================================================

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>

using namespace robotlib;

int main() {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  Simple Motor Control\n";
    std::cout << "  (No Encoders Required!)\n";
    std::cout << "========================================\n\n";

    // ========================================
    // STEP 1: Create the motor
    // ========================================
    std::cout << "STEP 1: Creating motor controller...\n\n";

    Arm motor = Arm();  // Create a motor controller

    // 🔌 HARDWARE SETUP:
    // Connect motor driver (like L298N or motor shield) to:
    // - Arduino Pin 9 (PWM) → Motor speed control
    // - Arduino Pin 8 (DIR) → Motor direction
    // - Motor driver → DC motor
    // - Power supply → Motor driver VCC (6-12V for most motors)

    std::cout << "✓ Motor created!\n";
    std::cout << "  Ready to control\n\n";

    // ========================================
    // STEP 2: Spin the motor forward
    // ========================================
    std::cout << "STEP 2: Spin motor forward at 50% power\n\n";

    motor.setDutyCycle(0.5);  // 0.5 = 50% power forward

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // digitalWrite(8, HIGH);              // Set direction to forward
    // analogWrite(9, 0.5 * 255);          // Set PWM to 128 (50% of 255)
    // The motor spins forward at half speed

    // ⏱️ IN YOUR REAL CODE, ADD:
    // delay(2000);  // Let motor run for 2 seconds

    std::cout << "  Power level: " << (motor.getDutyCycle() * 100.0) << "%\n";
    std::cout << "  Direction: Forward\n";
    std::cout << "  💡 In real hardware, let this run for 2 seconds\n\n";

    // ========================================
    // STEP 3: Spin the motor backward
    // ========================================
    std::cout << "STEP 3: Spin motor backward at 30% power\n\n";

    motor.setDutyCycle(-0.3);  // Negative = reverse direction

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // digitalWrite(8, LOW);               // Set direction to reverse
    // analogWrite(9, 0.3 * 255);          // Set PWM to 77 (30% of 255)
    // The motor spins backward at slower speed

    // ⏱️ IN YOUR REAL CODE, ADD:
    // delay(1000);  // Let motor run for 1 second

    std::cout << "  Power level: " << (motor.getDutyCycle() * -100.0) << "%\n";
    std::cout << "  Direction: Backward\n";
    std::cout << "  💡 In real hardware, let this run for 1 second\n\n";

    // ========================================
    // STEP 4: Stop the motor
    // ========================================
    std::cout << "STEP 4: Stop the motor\n\n";

    motor.stop();  // Stop the motor

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // analogWrite(9, 0);                  // Set PWM to 0
    // The motor stops spinning (coasts to a stop)
    // For instant stop (brake), you'd short both motor terminals

    std::cout << "  Power level: " << (motor.getDutyCycle() * 100.0) << "%\n";
    std::cout << "  Motor is now stopped\n\n";

    // ========================================
    // STEP 5: Smoothly ramp up speed
    // ========================================
    std::cout << "STEP 5: Smoothly ramp up from 0% to 100%\n\n";
    std::cout << "  Speed  | Power\n";
    std::cout << "  -------|-------\n";

    // Gradually increase from 0 to 100% in 25% steps
    for (int speed = 0; speed <= 100; speed += 25) {
        double power = speed / 100.0;  // Convert to 0.0-1.0 range
        motor.setDutyCycle(power);

        // 🔧 WHAT HAPPENS IN REAL HARDWARE:
        // analogWrite(9, power * 255);    // PWM increases: 0, 64, 128, 191, 255
        // The motor gradually speeds up

        // ⏱️ IN YOUR REAL CODE, ADD:
        // delay(500);  // Wait 0.5s between speed changes

        std::cout << "  " << std::setw(5) << speed << "% | "
                  << std::setw(5) << std::fixed << std::setprecision(1)
                  << (motor.getDutyCycle() * 100.0) << "%\n";
    }

    std::cout << "\n  💡 In real hardware, add a small delay between steps\n";
    std::cout << "     This prevents sudden jumps that could damage motor/gears\n\n";

    // ========================================
    // Summary
    // ========================================
    std::cout << "========================================\n";
    std::cout << "✓ You learned:\n";
    std::cout << "  1. Create a motor: Arm motor = Arm()\n";
    std::cout << "  2. Spin forward: motor.setDutyCycle(0.5)\n";
    std::cout << "  3. Spin backward: motor.setDutyCycle(-0.5)\n";
    std::cout << "  4. Stop: motor.stop()\n";
    std::cout << "  5. Ramp speed: Loop and change gradually\n\n";

    std::cout << "========================================\n";
    std::cout << "🔧 COMPLETE ARDUINO IMPLEMENTATION:\n";
    std::cout << "========================================\n\n";
    std::cout << "const int PWM_PIN = 9;    // Motor speed\n";
    std::cout << "const int DIR_PIN = 8;    // Motor direction\n\n";
    std::cout << "void setup() {\n";
    std::cout << "  pinMode(PWM_PIN, OUTPUT);\n";
    std::cout << "  pinMode(DIR_PIN, OUTPUT);\n";
    std::cout << "}\n\n";
    std::cout << "void loop() {\n";
    std::cout << "  // Get duty cycle from RobotLib\n";
    std::cout << "  double duty = motor.getDutyCycle();\n\n";
    std::cout << "  // Set direction based on sign\n";
    std::cout << "  digitalWrite(DIR_PIN, duty >= 0 ? HIGH : LOW);\n\n";
    std::cout << "  // Set speed (PWM is 0-255)\n";
    std::cout << "  analogWrite(PWM_PIN, abs(duty) * 255);\n";
    std::cout << "}\n\n";

    std::cout << "Next step: Try 'simple_servo.cpp' for servo control!\n";
    std::cout << "========================================\n\n";

    return 0;
}
