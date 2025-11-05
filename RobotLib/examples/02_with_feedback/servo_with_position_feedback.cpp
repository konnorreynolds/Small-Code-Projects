// ============================================================================
// Servo with Position Feedback
// ============================================================================
// Demonstrates servo control WITH position feedback using the fluent API.
// Perfect for smart servos with built-in position sensors or external encoders.
//
// Use cases:
// - Dynamixel servos
// - Servos with absolute encoders
// - Closed-loop position control
// - Robotic arms with feedback
// ============================================================================

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace robotlib;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Servo with Position Feedback\n";
    std::cout << "========================================\n\n";

    // Create servo with PID control for accurate positioning
    Arm servo = Arm()
        .withPID(2.0, 0.1, 0.05)         // Tuned PID gains
        .withLimits(deg(0), deg(180))    // Physical limits
        .setSpeed(1.0);                   // Full speed

    std::cout << "Servo configured:\n";
    std::cout << "  PID: kP=" << servo.getKP() << ", kI=" << servo.getKI() << ", kD=" << servo.getKD() << "\n";
    std::cout << "  Limits: " << servo.getMinAngle().toDegrees()
              << "° to " << servo.getMaxAngle().toDegrees() << "°\n";
    std::cout << "  Feedback: Enabled (position sensor)\n\n";

    // Example 1: Move to target with feedback
    std::cout << "Example 1: Move to 90° with feedback\n";
    servo.moveTo(deg(90));

    std::cout << "\nTime(s) | Current(°) | Target(°) | Error(°) | Duty(%) | Status\n";
    std::cout << "--------|------------|-----------|----------|---------|----------\n";

    // Simulate servo movement with position feedback
    Radians sim_position = rad(0);
    for (int step = 0; step <= 40; step++) {
        double t = step * 0.05;  // 50ms timestep

        // Simulate servo physics
        double duty = servo.getDutyCycle();
        double position_rad = sim_position.toRadians();
        position_rad += duty * 0.05 * 3.0;  // Servo response
        sim_position = Radians::fromRadians(position_rad);

        // Update controller with measured position
        servo.update(0.05, sim_position);

        if (step % 5 == 0) {  // Print every 250ms
            std::cout << std::setw(7) << std::fixed << std::setprecision(2) << t << " | ";
            std::cout << std::setw(10) << std::setprecision(1) << sim_position.toDegrees() << " | ";
            std::cout << std::setw(9) << servo.getTarget().toDegrees() << " | ";
            std::cout << std::setw(8) << servo.getError() * 57.3 << " | ";
            std::cout << std::setw(7) << (servo.getDutyCycle() * 100.0) << " | ";
            std::cout << (servo.isAtTarget(2.0) ? "At target" : "Moving") << "\n";
        }
    }

    std::cout << "\n";

    // Example 2: Tracking smooth trajectory
    std::cout << "Example 2: Track smooth sine wave trajectory\n\n";

    servo.moveTo(deg(90));  // Start at center
    sim_position = Radians::fromRadians(M_PI / 2);

    std::cout << "Time(s) | Target(°) | Actual(°) | Error(°)\n";
    std::cout << "--------|-----------|-----------|----------\n";

    for (int step = 0; step <= 60; step++) {
        double t = step * 0.05;

        // Sine wave target: 90° ± 30° at 0.5 Hz
        double target_deg = 90.0 + 30.0 * std::sin(2.0 * M_PI * 0.5 * t);
        servo.moveTo(deg(target_deg));

        // Simulate and update
        double duty = servo.getDutyCycle();
        double pos_rad = sim_position.toRadians() + duty * 0.05 * 3.0;
        sim_position = Radians::fromRadians(pos_rad);
        servo.update(0.05, sim_position);

        if (step % 10 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(2) << t << " | ";
            std::cout << std::setw(9) << std::setprecision(1) << servo.getTarget().toDegrees() << " | ";
            std::cout << std::setw(9) << sim_position.toDegrees() << " | ";
            std::cout << std::setw(8) << std::abs(servo.getError() * 57.3) << "\n";
        }
    }

    std::cout << "\n";

    // Example 3: Multi-servo coordinated movement (robot arm)
    std::cout << "Example 3: 3-DOF Robot Arm with Feedback\n\n";

    Arm shoulder = Arm().withPID(1.5, 0.1, 0.05).withLimits(deg(0), deg(180));
    Arm elbow = Arm().withPID(1.5, 0.1, 0.05).withLimits(deg(0), deg(180));
    Arm wrist = Arm().withPID(1.5, 0.1, 0.05).withLimits(deg(0), deg(180));

    // Start at home position
    Radians shoulder_pos = rad(M_PI / 2);
    Radians elbow_pos = rad(M_PI / 2);
    Radians wrist_pos = rad(M_PI / 2);

    // Move to reach position
    shoulder.moveTo(deg(60));
    elbow.moveTo(deg(120));
    wrist.moveTo(deg(45));

    std::cout << "Moving to reach position...\n";
    std::cout << "Step | Shoulder | Elbow | Wrist | All At Target?\n";
    std::cout << "-----|----------|-------|-------|----------------\n";

    for (int step = 0; step < 30; step++) {
        // Update all servos with feedback
        shoulder_pos = Radians::fromRadians(
            shoulder_pos.toRadians() + shoulder.getDutyCycle() * 0.1 * 2.0);
        elbow_pos = Radians::fromRadians(
            elbow_pos.toRadians() + elbow.getDutyCycle() * 0.1 * 2.0);
        wrist_pos = Radians::fromRadians(
            wrist_pos.toRadians() + wrist.getDutyCycle() * 0.1 * 2.0);

        shoulder.update(0.1, shoulder_pos);
        elbow.update(0.1, elbow_pos);
        wrist.update(0.1, wrist_pos);

        if (step % 5 == 0) {
            bool all_at_target = shoulder.isAtTarget() && elbow.isAtTarget() && wrist.isAtTarget();

            std::cout << std::setw(4) << step << " | ";
            std::cout << std::setw(8) << std::fixed << std::setprecision(1)
                      << shoulder_pos.toDegrees() << " | ";
            std::cout << std::setw(5) << elbow_pos.toDegrees() << " | ";
            std::cout << std::setw(5) << wrist_pos.toDegrees() << " | ";
            std::cout << (all_at_target ? "YES" : "No") << "\n";
        }
    }

    std::cout << "\n========================================\n";
    std::cout << "✓ Servo control with position feedback\n";
    std::cout << "  • Closed-loop PID control\n";
    std::cout << "  • Accurate positioning\n";
    std::cout << "  • Real-time error correction\n";
    std::cout << "  • Smooth trajectory tracking\n";
    std::cout << "\nHardware requirements:\n";
    std::cout << "  • Position sensor (encoder, potentiometer)\n";
    std::cout << "  • Regular position updates to controller\n";
    std::cout << "  • Fast control loop (10-100Hz recommended)\n";
    std::cout << "========================================\n";

    return 0;
}
