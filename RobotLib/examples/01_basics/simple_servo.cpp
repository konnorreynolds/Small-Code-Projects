// ============================================================================
// Simple Servo Control - No Position Feedback
// ============================================================================
// Learn the basics: control a hobby servo without position sensors.
// Works with standard servos like SG90, MG996R, etc.
//
// What you'll learn:
// - How to create a servo controller
// - How to move to specific angles
// - How to control multiple servos
// - Common servo applications (gripper, pan/tilt)
// ============================================================================

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>

using namespace robotlib;

int main() {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  Simple Servo Control\n";
    std::cout << "  (For Hobby Servos like SG90)\n";
    std::cout << "========================================\n\n";

    // ========================================
    // STEP 1: Create a servo
    // ========================================
    std::cout << "STEP 1: Creating servo controller...\n\n";

    Arm servo = Arm()
        .withLimits(deg(0), deg(180));  // Typical servo: 0° to 180°

    std::cout << "✓ Servo created!\n";
    std::cout << "  Range: " << servo.getMinAngle().toDegrees()
              << "° to " << servo.getMaxAngle().toDegrees() << "°\n\n";

    // ========================================
    // STEP 2: Move to center position
    // ========================================
    std::cout << "STEP 2: Move servo to center (90°)\n\n";

    servo.moveTo(deg(90));  // Move to 90 degrees

    std::cout << "  Target position: " << servo.getTarget().toDegrees() << "°\n";
    std::cout << "  💡 Servo will move to center position\n\n";

    // ========================================
    // STEP 3: Move to minimum position
    // ========================================
    std::cout << "STEP 3: Move servo to minimum (0°)\n\n";

    servo.moveTo(deg(0));  // Move to 0 degrees

    std::cout << "  Target position: " << servo.getTarget().toDegrees() << "°\n";
    std::cout << "  💡 Servo rotates all the way left\n\n";

    // ========================================
    // STEP 4: Move to maximum position
    // ========================================
    std::cout << "STEP 4: Move servo to maximum (180°)\n\n";

    servo.moveTo(deg(180));  // Move to 180 degrees

    std::cout << "  Target position: " << servo.getTarget().toDegrees() << "°\n";
    std::cout << "  💡 Servo rotates all the way right\n\n";

    // ========================================
    // EXAMPLE: Gripper control
    // ========================================
    std::cout << "========================================\n";
    std::cout << "EXAMPLE: Controlling a gripper\n";
    std::cout << "========================================\n\n";

    Arm gripper = Arm().withLimits(deg(0), deg(90));

    std::cout << "Opening gripper...\n";
    gripper.moveTo(deg(0));  // 0° = open
    std::cout << "  Position: " << gripper.getTarget().toDegrees() << "° (open)\n\n";

    std::cout << "Closing gripper...\n";
    gripper.moveTo(deg(90));  // 90° = closed
    std::cout << "  Position: " << gripper.getTarget().toDegrees() << "° (closed)\n\n";

    // ========================================
    // EXAMPLE: Camera pan/tilt
    // ========================================
    std::cout << "========================================\n";
    std::cout << "EXAMPLE: Camera pan/tilt system\n";
    std::cout << "========================================\n\n";

    // Create two servos: one for pan, one for tilt
    Arm panServo = Arm().withLimits(deg(-90), deg(90));   // Left/right
    Arm tiltServo = Arm().withLimits(deg(-45), deg(45));  // Up/down

    std::cout << "Look center:\n";
    panServo.moveTo(deg(0));
    tiltServo.moveTo(deg(0));
    std::cout << "  Pan: " << panServo.getTarget().toDegrees() << "°, "
              << "Tilt: " << tiltServo.getTarget().toDegrees() << "°\n\n";

    std::cout << "Look left:\n";
    panServo.moveTo(deg(45));
    tiltServo.moveTo(deg(0));
    std::cout << "  Pan: " << panServo.getTarget().toDegrees() << "°, "
              << "Tilt: " << tiltServo.getTarget().toDegrees() << "°\n\n";

    std::cout << "Look up:\n";
    panServo.moveTo(deg(0));
    tiltServo.moveTo(deg(30));
    std::cout << "  Pan: " << panServo.getTarget().toDegrees() << "°, "
              << "Tilt: " << tiltServo.getTarget().toDegrees() << "°\n\n";

    // ========================================
    // EXAMPLE: Simple robot arm
    // ========================================
    std::cout << "========================================\n";
    std::cout << "EXAMPLE: 3-servo robot arm\n";
    std::cout << "========================================\n\n";

    // Create three servos for a simple arm
    Arm shoulder = Arm().withLimits(deg(0), deg(180));
    Arm elbow = Arm().withLimits(deg(0), deg(180));
    Arm wrist = Arm().withLimits(deg(0), deg(180));

    std::cout << "Rest position:\n";
    shoulder.moveTo(deg(90));
    elbow.moveTo(deg(90));
    wrist.moveTo(deg(90));
    std::cout << "  Shoulder: " << shoulder.getTarget().toDegrees() << "°\n";
    std::cout << "  Elbow: " << elbow.getTarget().toDegrees() << "°\n";
    std::cout << "  Wrist: " << wrist.getTarget().toDegrees() << "°\n\n";

    std::cout << "Reach forward:\n";
    shoulder.moveTo(deg(90));
    elbow.moveTo(deg(135));
    wrist.moveTo(deg(45));
    std::cout << "  Shoulder: " << shoulder.getTarget().toDegrees() << "°\n";
    std::cout << "  Elbow: " << elbow.getTarget().toDegrees() << "°\n";
    std::cout << "  Wrist: " << wrist.getTarget().toDegrees() << "°\n\n";

    // ========================================
    // Summary
    // ========================================
    std::cout << "========================================\n";
    std::cout << "✓ You learned:\n";
    std::cout << "  1. Create servo: Arm servo = Arm()\n";
    std::cout << "  2. Set limits: .withLimits(deg(0), deg(180))\n";
    std::cout << "  3. Move servo: servo.moveTo(deg(90))\n";
    std::cout << "  4. Get position: servo.getTarget()\n\n";

    std::cout << "💡 Hardware tip:\n";
    std::cout << "  Connect servo to PWM pin\n";
    std::cout << "  0° = 1000µs pulse, 90° = 1500µs, 180° = 2000µs\n\n";

    std::cout << "Next: Try 'simple_differential_drive.cpp'!\n";
    std::cout << "========================================\n\n";

    return 0;
}
