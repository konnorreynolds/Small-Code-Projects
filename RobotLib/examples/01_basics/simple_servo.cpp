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

#include "../../include/RobotLib.h"

using namespace robotlib;
using namespace robotlib::output;

int main() {
    println("");
    println("========================================");
    println("  Simple Servo Control");
    println("  (For Hobby Servos like SG90)");
    println("========================================\n");

    // ========================================
    // STEP 1: Create a servo
    // ========================================
    println("STEP 1: Creating servo controller...\n");

    Arm servo = Arm()
        .withLimits(deg(0), deg(180));  // Typical servo: 0° to 180°

    // 🔌 HARDWARE SETUP:
    // Connect servo (SG90, MG996R, or similar) to:
    // - Arduino Pin 9 (PWM) → Servo signal wire (usually orange/white)
    // - Arduino 5V → Servo power (red wire)
    // - Arduino GND → Servo ground (brown/black wire)
    // Note: Large servos need external power supply!

    println("✓ Servo created!");
    print("  Range: " ,  servo.getMinAngle().toDegrees()
              , "° to ");

    // ========================================
    // STEP 2: Move to center position
    // ========================================
    println("STEP 2: Move servo to center (90°)\n");

    servo.moveTo(deg(90));  // Move to 90 degrees

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // servo.write(90);                    // Arduino Servo library
    // OR: Send 1500µs pulse width
    // The servo rotates to 90° (center/neutral position)

    println("  Target position: ", servo.getTarget().toDegrees(), "°");
    println("  💡 Servo will move to center position\n");

    // ========================================
    // STEP 3: Move to minimum position
    // ========================================
    println("STEP 3: Move servo to minimum (0°)\n");

    servo.moveTo(deg(0));  // Move to 0 degrees

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // servo.write(0);                     // Arduino Servo library
    // OR: Send 1000µs pulse width
    // The servo rotates all the way to the left

    println("  Target position: ", servo.getTarget().toDegrees(), "°");
    println("  💡 Servo rotates all the way left\n");

    // ========================================
    // STEP 4: Move to maximum position
    // ========================================
    println("STEP 4: Move servo to maximum (180°)\n");

    servo.moveTo(deg(180));  // Move to 180 degrees

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // servo.write(180);                   // Arduino Servo library
    // OR: Send 2000µs pulse width
    // The servo rotates all the way to the right

    println("  Target position: ", servo.getTarget().toDegrees(), "°");
    println("  💡 Servo rotates all the way right\n");

    // ========================================
    // EXAMPLE: Gripper control
    // ========================================
    println("========================================");
    println("EXAMPLE: Controlling a gripper");
    println("========================================\n");

    Arm gripper = Arm().withLimits(deg(0), deg(90));

    println("Opening gripper...");
    gripper.moveTo(deg(0));  // 0° = open
    println("  Position: ", gripper.getTarget().toDegrees(), "° (open)\n");

    println("Closing gripper...");
    gripper.moveTo(deg(90));  // 90° = closed
    println("  Position: ", gripper.getTarget().toDegrees(), "° (closed)\n");

    // ========================================
    // EXAMPLE: Camera pan/tilt
    // ========================================
    println("========================================");
    println("EXAMPLE: Camera pan/tilt system");
    println("========================================\n");

    // Create two servos: one for pan, one for tilt
    Arm panServo = Arm().withLimits(deg(-90), deg(90));   // Left/right
    Arm tiltServo = Arm().withLimits(deg(-45), deg(45));  // Up/down

    println("Look center:");
    panServo.moveTo(deg(0));
    tiltServo.moveTo(deg(0));
    print("  Pan: " ,  panServo.getTarget().toDegrees() , "°, ");

    println("Look left:");
    panServo.moveTo(deg(45));
    tiltServo.moveTo(deg(0));
    print("  Pan: " ,  panServo.getTarget().toDegrees() , "°, ");

    println("Look up:");
    panServo.moveTo(deg(0));
    tiltServo.moveTo(deg(30));
    print("  Pan: " ,  panServo.getTarget().toDegrees() , "°, ");

    // ========================================
    // EXAMPLE: Simple robot arm
    // ========================================
    println("========================================");
    println("EXAMPLE: 3-servo robot arm");
    println("========================================\n");

    // Create three servos for a simple arm
    Arm shoulder = Arm().withLimits(deg(0), deg(180));
    Arm elbow = Arm().withLimits(deg(0), deg(180));
    Arm wrist = Arm().withLimits(deg(0), deg(180));

    println("Rest position:");
    shoulder.moveTo(deg(90));
    elbow.moveTo(deg(90));
    wrist.moveTo(deg(90));
    println("  Shoulder: ", shoulder.getTarget().toDegrees(), "°");
    println("  Elbow: ", elbow.getTarget().toDegrees(), "°");
    println("  Wrist: ", wrist.getTarget().toDegrees(), "°\n");

    println("Reach forward:");
    shoulder.moveTo(deg(90));
    elbow.moveTo(deg(135));
    wrist.moveTo(deg(45));
    println("  Shoulder: ", shoulder.getTarget().toDegrees(), "°");
    println("  Elbow: ", elbow.getTarget().toDegrees(), "°");
    println("  Wrist: ", wrist.getTarget().toDegrees(), "°\n");

    // ========================================
    // Summary
    // ========================================
    println("========================================");
    println("✓ You learned:");
    println("  1. Create servo: Arm servo = Arm()");
    println("  2. Set limits: .withLimits(deg(0), deg(180))");
    println("  3. Move servo: servo.moveTo(deg(90))");
    println("  4. Get position: servo.getTarget()\n");

    println("========================================");
    println("🔧 COMPLETE ARDUINO IMPLEMENTATION:");
    println("========================================\n");
    println("#include <Servo.h>\n");
    println("Servo myServo;");
    println("const int SERVO_PIN = 9;\n");
    println("void setup() {");
    println("  myServo.attach(SERVO_PIN);");
    println("}\n");
    println("void loop() {");
    println("  // Get target angle from RobotLib");
    println("  double angle = servo.getTarget().toDegrees();\n");
    println("  // Command the physical servo");
    println("  myServo.write(angle);\n");
    println("  delay(15);  // Small delay for servo to move");
    println("}\n");

    println("💡 Pulse widths:");
    println("  0° = 1000µs | 90° = 1500µs | 180° = 2000µs\n");

    println("Next: Try 'simple_differential_drive.cpp'!");
    println("========================================\n");

    return 0;
}
