// ============================================================================
// Simple Servo Control - No Position Feedback
// ============================================================================
// Demonstrates servo control using the fluent API without position feedback.
// Perfect for hobby servos that don't have position sensors.
//
// Use cases:
// - Standard hobby servos (SG90, MG996R, etc.)
// - Gripper control
// - Camera pan/tilt
// - Simple robot arms
// ============================================================================

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>

using namespace robotlib;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Simple Servo Control (No Feedback)\n";
    std::cout << "========================================\n\n";

    // Create servo controller for typical hobby servo
    Arm gripper = Arm()
        .withLimits(deg(0), deg(180))  // Typical servo range
        .setSpeed(1.0);                 // Full speed

    Arm panServo = Arm()
        .withLimits(deg(-90), deg(90))  // Pan range
        .setSpeed(0.5);                  // Slower movement

    Arm tiltServo = Arm()
        .withLimits(deg(-45), deg(45))   // Tilt range
        .setSpeed(0.5);                   // Slower movement

    std::cout << "Servos configured:\n";
    std::cout << "  Gripper: 0° to 180°\n";
    std::cout << "  Pan: -90° to 90°\n";
    std::cout << "  Tilt: -45° to 45°\n\n";

    // Example 1: Gripper control
    std::cout << "Example 1: Gripper control\n";
    std::cout << "  Opening gripper (0°)...\n";
    gripper.moveTo(deg(0));
    std::cout << "    Target: " << gripper.getTarget().toDegrees() << "°\n";

    std::cout << "  Closing gripper (90°)...\n";
    gripper.moveTo(deg(90));
    std::cout << "    Target: " << gripper.getTarget().toDegrees() << "°\n\n";

    // Example 2: Camera pan/tilt
    std::cout << "Example 2: Camera pan/tilt sequence\n";

    struct Position {
        const char* name;
        double pan;
        double tilt;
    };

    Position positions[] = {
        {"Center",        0,   0},
        {"Look Left",    45,   0},
        {"Look Right",  -45,   0},
        {"Look Up",       0,  30},
        {"Look Down",     0, -30},
    };

    std::cout << "  Position      | Pan(°) | Tilt(°)\n";
    std::cout << "  --------------|--------|--------\n";

    for (const auto& pos : positions) {
        panServo.moveTo(deg(pos.pan));
        tiltServo.moveTo(deg(pos.tilt));

        std::cout << "  " << std::setw(13) << std::left << pos.name << " | "
                  << std::setw(6) << std::right << std::fixed << std::setprecision(0)
                  << panServo.getTarget().toDegrees() << " | "
                  << std::setw(6) << tiltServo.getTarget().toDegrees() << "\n";
    }

    std::cout << "\n";

    // Example 3: Simple robot arm (3 servos)
    std::cout << "Example 3: Simple 3-DOF robot arm\n";

    Arm shoulder = Arm().withLimits(deg(0), deg(180));
    Arm elbow = Arm().withLimits(deg(0), deg(180));
    Arm wrist = Arm().withLimits(deg(0), deg(180));

    // Arm poses
    struct ArmPose {
        const char* name;
        double shoulder_angle;
        double elbow_angle;
        double wrist_angle;
    };

    ArmPose poses[] = {
        {"Rest",        90,  90,  90},
        {"Reach Up",    45,  45, 135},
        {"Reach Forward", 90, 135,  45},
        {"Rest",        90,  90,  90},
    };

    std::cout << "  Pose          | Shoulder | Elbow | Wrist\n";
    std::cout << "  --------------|----------|-------|-------\n";

    for (const auto& pose : poses) {
        shoulder.moveTo(deg(pose.shoulder_angle));
        elbow.moveTo(deg(pose.elbow_angle));
        wrist.moveTo(deg(pose.wrist_angle));

        std::cout << "  " << std::setw(13) << std::left << pose.name << " | "
                  << std::setw(8) << std::right << std::fixed << std::setprecision(0)
                  << shoulder.getTarget().toDegrees() << " | "
                  << std::setw(5) << elbow.getTarget().toDegrees() << " | "
                  << std::setw(5) << wrist.getTarget().toDegrees() << "\n";
    }

    std::cout << "\n========================================\n";
    std::cout << "✓ Simple servo control without feedback\n";
    std::cout << "  • Works with standard hobby servos\n";
    std::cout << "  • No position sensors required\n";
    std::cout << "  • Servo library handles positioning\n";
    std::cout << "  • Perfect for grippers, pan/tilt, arms\n";
    std::cout << "\nHardware connection:\n";
    std::cout << "  • Convert duty cycle to PWM (1000-2000µs)\n";
    std::cout << "  • 0° = 1000µs, 90° = 1500µs, 180° = 2000µs\n";
    std::cout << "========================================\n";

    return 0;
}
