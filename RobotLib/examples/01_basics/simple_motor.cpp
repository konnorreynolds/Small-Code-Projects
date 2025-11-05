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

    std::cout << "✓ Motor created!\n";
    std::cout << "  Ready to control\n\n";

    // ========================================
    // STEP 2: Spin the motor forward
    // ========================================
    std::cout << "STEP 2: Spin motor forward at 50% power\n\n";

    motor.setDutyCycle(0.5);  // 0.5 = 50% power forward

    std::cout << "  Power level: " << (motor.getDutyCycle() * 100.0) << "%\n";
    std::cout << "  Direction: Forward\n";
    std::cout << "  💡 In real hardware, let this run for 2 seconds\n\n";

    // ========================================
    // STEP 3: Spin the motor backward
    // ========================================
    std::cout << "STEP 3: Spin motor backward at 30% power\n\n";

    motor.setDutyCycle(-0.3);  // Negative = reverse direction

    std::cout << "  Power level: " << (motor.getDutyCycle() * -100.0) << "%\n";
    std::cout << "  Direction: Backward\n";
    std::cout << "  💡 In real hardware, let this run for 1 second\n\n";

    // ========================================
    // STEP 4: Stop the motor
    // ========================================
    std::cout << "STEP 4: Stop the motor\n\n";

    motor.stop();  // Stop the motor

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

        std::cout << "  " << std::setw(5) << speed << "% | "
                  << std::setw(5) << std::fixed << std::setprecision(1)
                  << (motor.getDutyCycle() * 100.0) << "%\n";
    }

    std::cout << "\n  💡 In real hardware, add a small delay between steps\n\n";

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

    std::cout << "Next step: Try 'simple_servo.cpp' for servo control!\n";
    std::cout << "========================================\n\n";

    return 0;
}
