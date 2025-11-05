// ============================================================================
// Simple Motor Control - No Sensor Feedback
// ============================================================================
// Demonstrates basic motor control using the fluent API without any sensor
// feedback. Perfect for open-loop control where you just set motor speeds.
//
// Use cases:
// - Basic motor testing
// - Open-loop speed control
// - Simple timed movements
// - Motor calibration
// ============================================================================

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>

using namespace robotlib;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Simple Motor Control (No Feedback)\n";
    std::cout << "========================================\n\n";

    // Create motor controller - no sensor setup needed!
    Arm motor = Arm()
        .withLimits(deg(-180), deg(180))  // Optional safety limits
        .setSpeed(0.8);                   // 80% max speed

    std::cout << "Motor configured:\n";
    std::cout << "  Speed: " << (motor.getSpeed() * 100.0) << "%\n";
    std::cout << "  Limits: " << motor.getMinAngle().toDegrees()
              << "° to " << motor.getMaxAngle().toDegrees() << "°\n\n";

    // Example 1: Simple position command (open-loop)
    std::cout << "Example 1: Move to 90 degrees (open-loop)\n";
    motor.moveTo(deg(90));

    std::cout << "  Commanded target: " << motor.getTarget().toDegrees() << "°\n";
    std::cout << "  Duty cycle: " << (motor.getDutyCycle() * 100.0) << "%\n";
    std::cout << "  (Motor will run until you stop it - no feedback!)\n\n";

    // Example 2: Direct duty cycle control
    std::cout << "Example 2: Set duty cycle directly\n";
    motor.setDutyCycle(0.5);  // 50% power forward

    std::cout << "  Duty cycle: " << (motor.getDutyCycle() * 100.0) << "%\n";
    std::cout << "  Run for 2 seconds then stop\n\n";

    // Example 3: Timed movement sequence
    std::cout << "Example 3: Timed movement sequence\n";
    std::cout << "  Step 1: Forward 30% for 1.0s\n";
    motor.setDutyCycle(0.3);

    std::cout << "  Step 2: Reverse 30% for 1.0s\n";
    motor.setDutyCycle(-0.3);

    std::cout << "  Step 3: Stop\n";
    motor.stop();
    std::cout << "  Duty cycle: " << motor.getDutyCycle() << "\n\n";

    // Example 4: Speed ramping (manual)
    std::cout << "Example 4: Manual speed ramping\n";
    std::cout << "  Speed | Duty(%)\n";
    std::cout << "  ------|--------\n";

    for (int speed = 0; speed <= 100; speed += 25) {
        double duty = speed / 100.0;
        motor.setDutyCycle(duty);
        std::cout << "  " << std::setw(5) << speed << " | "
                  << std::setw(6) << std::fixed << std::setprecision(1)
                  << (motor.getDutyCycle() * 100.0) << "\n";
    }

    std::cout << "\n========================================\n";
    std::cout << "✓ Simple motor control without feedback\n";
    std::cout << "  • No encoders or sensors needed\n";
    std::cout << "  • Direct duty cycle control\n";
    std::cout << "  • Perfect for open-loop applications\n";
    std::cout << "  • Use timers to control movement duration\n";
    std::cout << "========================================\n";

    return 0;
}
