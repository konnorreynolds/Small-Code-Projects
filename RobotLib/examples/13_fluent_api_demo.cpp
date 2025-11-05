// ============================================================================
// Example 13: Fluent API Demonstration
// ============================================================================
// Demonstrates the easy-to-use fluent/chaining API for RobotLib.
// Every method returns *this for beautiful method chaining.
//
// Topics covered:
// - Fluent API pattern (thing.action().action().action())
// - Arm control with chaining
// - Differential drive with chaining
// - Sensor filtering with chaining
// - PID controller with chaining
// ============================================================================

#include "../include/robotlib_api.h"
#include <iostream>
#include <iomanip>

using namespace robotlib;

void demonstrateArmControl() {
    std::cout << "========================================\n";
    std::cout << "  Fluent Arm Control\n";
    std::cout << "========================================\n\n";

    // Create and configure arm with beautiful chaining
    Arm leftArm = Arm()
        .withPID(1.5, 0.1, 0.05)
        .withFeedforward(0.05, 0.001)
        .withLimits(deg(-90), deg(90))
        .setSpeed(0.8);

    std::cout << "Arm configured with chaining:\n";
    std::cout << "  PID: kP=" << leftArm.getKP() << ", kI=" << leftArm.getKI() << ", kD=" << leftArm.getKD() << "\n";
    std::cout << "  Feedforward: kS=" << leftArm.getKS() << ", kV=" << leftArm.getKV() << "\n";
    std::cout << "  Limits: " << leftArm.getMinAngle().toDegrees() << "° to " << leftArm.getMaxAngle().toDegrees() << "°\n";
    std::cout << "  Speed: " << (leftArm.getSpeed() * 100.0) << "%\n";
    std::cout << "  PID enabled: " << (leftArm.isPIDEnabled() ? "Yes" : "No") << "\n";
    std::cout << "  Feedforward enabled: " << (leftArm.isFeedforwardEnabled() ? "Yes" : "No") << "\n";
    std::cout << "  Has limits: " << (leftArm.hasLimits() ? "Yes" : "No") << "\n\n";

    std::cout << "✓ Configuration stored and retrievable!\n\n";

    // Move to position with chaining
    std::cout << "Moving to 45 degrees...\n";
    leftArm.moveTo(deg(45))
           .setSpeed(1.0);

    // Simulate control loop
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nTime(s) | Position(°) | Target(°) | Duty(%) | Status\n";
    std::cout << "--------|-------------|-----------|---------|--------\n";

    Radians sim_position = rad(0);
    for (int step = 0; step <= 20; step++) {
        double t = step * 0.1;

        // Simulate motor response
        double duty = leftArm.getDutyCycle();
        double position_rad = sim_position.toRadians();
        position_rad += duty * 0.1 * 2.0;  // Simple integration
        sim_position = Radians::fromRadians(position_rad);

        // Update controller with measured position (chaining!)
        leftArm.update(0.1, sim_position);

        if (step % 4 == 0) {
            std::cout << std::setw(7) << t << " | ";
            std::cout << std::setw(11) << sim_position.toDegrees() << " | ";
            std::cout << std::setw(9) << leftArm.getTarget().toDegrees() << " | ";
            std::cout << std::setw(7) << (leftArm.getDutyCycle() * 100.0) << " | ";
            std::cout << (leftArm.isAtTarget() ? "At target" : "Moving") << "\n";
        }
    }

    // Chain multiple movements
    std::cout << "\nChaining multiple movements:\n";
    leftArm.moveTo(deg(90))
           .setSpeed(0.5)
           .update(0.1, deg(50))
           .update(0.1, deg(55));

    std::cout << "  Target: " << leftArm.getTarget().toDegrees() << "°\n";
    std::cout << "  Position: " << leftArm.getPosition().toDegrees() << "°\n";
    std::cout << "  Error: " << leftArm.getError() * 57.3 << "°\n\n";
}

void demonstrateDifferentialDrive() {
    std::cout << "========================================\n";
    std::cout << "  Fluent Differential Drive\n";
    std::cout << "========================================\n\n";

    // Create and configure with chaining
    DifferentialDrive robot = DifferentialDrive()
        .withWheelbase(m(0.5))
        .withWheelDiameter(m(0.1))
        .withMaxSpeed(mps(2.0));

    std::cout << "Robot configured:\n";
    std::cout << "  Wheelbase: " << robot.getWheelbase().toMeters() << "m\n";
    std::cout << "  Wheel diameter: " << robot.getWheelDiameter().toMeters() << "m\n";
    std::cout << "  Max speed: " << robot.getMaxSpeed().toMetersPerSecond() << " m/s\n\n";

    std::cout << "✓ Configuration stored - access anytime with robot.getWheelbase(), etc.\n\n";

    // Drive forward with chaining
    std::cout << "Arcade drive - Forward 50%, Turn left 30%:\n";
    robot.arcade(0.5, -0.3);

    std::cout << "  Left motor: " << (robot.getLeftDuty() * 100.0) << "%\n";
    std::cout << "  Right motor: " << (robot.getRightDuty() * 100.0) << "%\n\n";

    // Chain multiple drive commands
    std::cout << "Chaining: Drive forward -> Turn -> Stop\n";
    robot.drive(mps(1.0), radps(0.0))  // Forward
         .arcade(0.0, 0.5)              // Turn right
         .stop();                        // Stop

    std::cout << "  Final state: Stopped\n";
    std::cout << "  Left duty: " << robot.getLeftDuty() << "\n";
    std::cout << "  Right duty: " << robot.getRightDuty() << "\n\n";

    // Tank drive
    std::cout << "Tank drive - Spin in place:\n";
    robot.tank(0.5, -0.5);  // Left forward, right backward

    std::cout << "  Left motor: " << (robot.getLeftDuty() * 100.0) << "%\n";
    std::cout << "  Right motor: " << (robot.getRightDuty() * 100.0) << "%\n\n";

    // Demonstrate comprehensive state tracking with odometry
    std::cout << "State tracking with odometry:\n";
    robot.resetOdometry(0.0, 0.0, 0.0);  // Start at origin

    // Simulate driving forward with encoder feedback
    for (int i = 0; i < 5; i++) {
        // Simulate wheel velocities from encoders
        robot.updateVelocities(mps(1.0), mps(1.0))  // Both wheels 1 m/s
              .updateOdometry(0.1);  // Update position every 0.1s

        if (i == 4) {
            std::cout << "  Position: (" << robot.getX() << ", " << robot.getY() << ") m\n";
            std::cout << "  Heading: " << robot.getThetaDegrees() << "°\n";
            std::cout << "  Left velocity: " << robot.getLeftVelocity().toMetersPerSecond() << " m/s\n";
            std::cout << "  Right velocity: " << robot.getRightVelocity().toMetersPerSecond() << " m/s\n";
        }
    }

    std::cout << "\n✓ Complete robot state stored and accessible!\n";
    std::cout << "  • Configuration: wheelbase, wheel diameter, max speed\n";
    std::cout << "  • Command state: duty cycles, commanded velocities\n";
    std::cout << "  • Measured state: encoder distances, wheel velocities\n";
    std::cout << "  • Odometry state: X, Y, theta position\n\n";
}

void demonstrateSensorFiltering() {
    std::cout << "========================================\n";
    std::cout << "  Fluent Sensor Filtering\n";
    std::cout << "========================================\n\n";

    // Create sensor with filtering (chained!)
    Sensor distanceSensor = Sensor()
        .withLowPassFilter(0.3)
        .withMovingAverage();

    std::cout << "Sensor configured with:\n";
    std::cout << "  - Low-pass filter (alpha=" << distanceSensor.getLowPassAlpha() << ")\n";
    std::cout << "  - Moving average enabled: " << (distanceSensor.isMovingAverageEnabled() ? "Yes" : "No") << "\n";
    std::cout << "  - Low-pass enabled: " << (distanceSensor.isLowPassEnabled() ? "Yes" : "No") << "\n\n";

    std::cout << "✓ Filter configuration retrievable!\n\n";

    std::cout << "Reading noisy data:\n";
    std::cout << "Sample | Raw   | Filtered\n";
    std::cout << "-------|-------|----------\n";

    // Simulate noisy sensor readings with chaining
    double noisy_readings[] = {100.0, 102.0, 98.0, 101.0, 99.0, 100.5, 101.5, 99.5};

    for (size_t i = 0; i < 8; ++i) {
        // Chain read and query operations
        distanceSensor.read(noisy_readings[i]);

        std::cout << std::setw(6) << (i + 1) << " | ";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << std::setw(5) << distanceSensor.getRawValue() << " | ";
        std::cout << std::setw(8) << distanceSensor.getValue() << "\n";
    }

    std::cout << "\nFiltered value is smoother!\n\n";
}

void demonstratePIDController() {
    std::cout << "========================================\n";
    std::cout << "  Fluent PID Controller\n";
    std::cout << "========================================\n\n";

    // Configure PID with chaining
    PID controller = PID()
        .withGains(1.2, 0.1, 0.05)
        .withOutputLimits(-1.0, 1.0)
        .withIntegralLimit(10.0);

    std::cout << "PID configured:\n";
    std::cout << "  Gains: kP=" << controller.getKP() << ", kI=" << controller.getKI() << ", kD=" << controller.getKD() << "\n";
    std::cout << "  Output: -1.0 to 1.0\n";
    std::cout << "  Integral limit: 10.0\n\n";

    std::cout << "✓ PID gains stored and retrievable!\n\n";

    std::cout << "Controlling to setpoint=100:\n";
    std::cout << "Step | Error | Output\n";
    std::cout << "-----|-------|-------\n";

    double measured = 0.0;
    double setpoint = 100.0;

    for (int step = 0; step < 10; step++) {
        double error = setpoint - measured;

        // Calculate and query output (chaining!)
        controller.calculate(error, 0.1);

        std::cout << std::setw(4) << step << " | ";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << std::setw(5) << controller.getError() << " | ";
        std::cout << std::setw(6) << controller.getOutput() << "\n";

        // Simulate response
        measured += controller.getOutput() * 10.0;
    }

    std::cout << "\nFinal measured value: " << measured << "\n\n";
}

void demonstrateComplexChaining() {
    std::cout << "========================================\n";
    std::cout << "  Complex Method Chaining\n";
    std::cout << "========================================\n\n";

    std::cout << "Creating a complete robot arm subsystem:\n\n";

    // Everything configured in one beautiful chain!
    Arm shoulder = Arm()
        .withPID(2.0, 0.15, 0.08)
        .withFeedforward(0.1, 0.002)
        .withLimits(deg(0), deg(180))
        .setSpeed(0.9)
        .moveTo(deg(90))
        .resetPosition(deg(0));

    std::cout << "Shoulder configured and ready!\n";
    std::cout << "  Current: " << shoulder.getPosition().toDegrees() << "°\n";
    std::cout << "  Target: " << shoulder.getTarget().toDegrees() << "°\n";
    std::cout << "  Error: " << (shoulder.getError() * 57.3) << "°\n\n";

    // Complex control sequence - all chained!
    std::cout << "Executing movement sequence:\n";
    shoulder.moveTo(deg(45))
            .setSpeed(0.5)
            .update(0.1, deg(10))
            .update(0.1, deg(20))
            .setSpeed(1.0)
            .moveTo(deg(135));

    std::cout << "  New target: " << shoulder.getTarget().toDegrees() << "°\n";
    std::cout << "  Speed: 100%\n\n";

    // Create complete robot with chaining
    std::cout << "Creating complete robot:\n";

    DifferentialDrive chassis = DifferentialDrive()
        .withWheelbase(m(0.6))
        .withWheelDiameter(m(0.15))
        .withMaxSpeed(mps(3.0));

    Sensor rangefinder = Sensor()
        .withLowPassFilter(0.2)
        .withMovingAverage();

    std::cout << "  ✓ Chassis configured\n";
    std::cout << "  ✓ Arm configured\n";
    std::cout << "  ✓ Sensor configured\n\n";

    // Control everything together with chaining!
    chassis.arcade(0.8, 0.0);           // Drive forward
    shoulder.moveTo(deg(120));          // Raise arm
    rangefinder.read(150.5);            // Read sensor

    std::cout << "Robot state:\n";
    std::cout << "  Chassis: " << (chassis.getLeftDuty() * 100.0) << "% throttle\n";
    std::cout << "  Arm: " << shoulder.getTarget().toDegrees() << "° target\n";
    std::cout << "  Range: " << rangefinder.getValue() << " cm\n\n";
}

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  RobotLib Fluent API Demo             ║\n";
    std::cout << "║  Beautiful Method Chaining             ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrateArmControl();
    demonstrateDifferentialDrive();
    demonstrateSensorFiltering();
    demonstratePIDController();
    demonstrateComplexChaining();

    std::cout << "========================================\n";
    std::cout << "  Key Advantages of Fluent API\n";
    std::cout << "========================================\n\n";

    std::cout << "✅ Readable: Configuration reads like English\n";
    std::cout << "✅ Compact: Multiple actions in one line\n";
    std::cout << "✅ Type-safe: Still uses RobotLib's unit system\n";
    std::cout << "✅ Discoverable: IDE autocomplete shows all options\n";
    std::cout << "✅ Flexible: Chain as much or as little as you want\n\n";

    std::cout << "Example usage patterns:\n\n";

    std::cout << "1. Configuration chaining:\n";
    std::cout << "   Arm().withPID(1,0,0).withLimits(deg(-90), deg(90));\n\n";

    std::cout << "2. Action chaining:\n";
    std::cout << "   arm.moveTo(deg(90)).setSpeed(0.5).update(dt, pos);\n\n";

    std::cout << "3. Mixed chaining:\n";
    std::cout << "   robot.withWheelbase(m(0.5)).drive(mps(1), radps(0));\n\n";

    std::cout << "4. Query ends chain:\n";
    std::cout << "   double duty = arm.moveTo(deg(45)).getDutyCycle();\n\n";

    return 0;
}

/*
Expected Output:
    - Arm control with PID and chaining
    - Differential drive arcade/tank modes
    - Sensor filtering demonstration
    - PID controller step response
    - Complex multi-system chaining

This example demonstrates:
    - Fluent API pattern (method chaining)
    - Every config/action method returns *this
    - Query methods return values (end chain)
    - Readable, discoverable, type-safe

Perfect for:
    - Beginners wanting simple API
    - Teams wanting consistent code style
    - Rapid prototyping
    - Educational settings
*/
