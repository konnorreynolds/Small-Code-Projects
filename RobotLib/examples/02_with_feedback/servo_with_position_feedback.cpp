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

#include "../../include/RobotLib.h"
#include <cmath>

using namespace robotlib;
using namespace robotlib::output;

int main() {
    println("========================================");
    println("  Servo with Position Feedback");
    println("========================================\n");

    // Create servo with PID control for accurate positioning
    Arm servo = Arm()
        .withPID(2.0, 0.1, 0.05)         // Tuned PID gains
        .withLimits(deg(0), deg(180))    // Physical limits
        .setSpeed(1.0);                   // Full speed

    println("Servo configured:");
    println("  PID: kP=", servo.getKP(), ", kI=", servo.getKI(), ", kD=", servo.getKD(), "");
    print("  Limits: " ,  servo.getMinAngle().toDegrees()
              , "° to ");
    println("  Feedback: Enabled (position sensor)\n");

    // Example 1: Move to target with feedback
    println("Example 1: Move to 90° with feedback");
    servo.moveTo(deg(90));

    println("\nTime(s) | Current(°) | Target(°) | Error(°) | Duty(%) | Status");
    println("--------|------------|-----------|----------|---------|----------");

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
            print(, , t, " | ");
            print(, sim_position.toDegrees(), " | ");
            print(, servo.getTarget().toDegrees(), " | ");
            print(, servo.getError() * 57.3, " | ");
            print(, (servo.getDutyCycle() * 100.0), " | ");
            println((servo.isAtTarget(2.0) ? "At target" : "Moving"), "");
        }
    }

    println("");

    // Example 2: Tracking smooth trajectory
    println("Example 2: Track smooth sine wave trajectory\n");

    servo.moveTo(deg(90));  // Start at center
    sim_position = Radians::fromRadians(M_PI / 2);

    println("Time(s) | Target(°) | Actual(°) | Error(°)");
    println("--------|-----------|-----------|----------");

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
            print(, , t, " | ");
            print(, servo.getTarget().toDegrees(), " | ");
            print(, sim_position.toDegrees(), " | ");
            println(, std::abs(servo.getError() * 57.3), "");
        }
    }

    println("");

    // Example 3: Multi-servo coordinated movement (robot arm)
    println("Example 3: 3-DOF Robot Arm with Feedback\n");

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

    println("Moving to reach position...");
    println("Step | Shoulder | Elbow | Wrist | All At Target?");
    println("-----|----------|-------|-------|----------------");

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

            print(, step, " | ");
            print(,  
                      , shoulder_pos.toDegrees() , " | ");
            print(, elbow_pos.toDegrees(), " | ");
            print(, wrist_pos.toDegrees(), " | ");
            println((all_at_target ? "YES" : "No"), "");
        }
    }

    println("\n========================================");
    println("✓ Servo control with position feedback");
    println("  • Closed-loop PID control");
    println("  • Accurate positioning");
    println("  • Real-time error correction");
    println("  • Smooth trajectory tracking");
    println("\nHardware requirements:");
    println("  • Position sensor (encoder, potentiometer)");
    println("  • Regular position updates to controller");
    println("  • Fast control loop (10-100Hz recommended)");
    println("========================================");

    return 0;
}
