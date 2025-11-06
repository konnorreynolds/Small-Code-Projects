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

#include "../../include/RobotLib.h"

using namespace robotlib;
using namespace robotlib::output;

void demonstrateArmControl() {
    println("========================================");
    println("  Fluent Arm Control");
    println("========================================\n");

    // Create and configure arm with beautiful chaining
    Arm leftArm = Arm()
        .withPID(1.5, 0.1, 0.05)
        .withFeedforward(0.05, 0.001)
        .withLimits(deg(-90), deg(90))
        .setSpeed(0.8);

    println("Arm configured with chaining:");
    println("  PID: kP=", leftArm.getKP(), ", kI=", leftArm.getKI(), ", kD=", leftArm.getKD(), "");
    println("  Feedforward: kS=", leftArm.getKS(), ", kV=", leftArm.getKV(), "");
    println("  Limits: ", leftArm.getMinAngle().toDegrees(), "° to ", leftArm.getMaxAngle().toDegrees(), "°");
    println("  Speed: ", (leftArm.getSpeed() * 100.0), "%");
    println("  PID enabled: ", (leftArm.isPIDEnabled() ? "Yes" : "No"), "");
    println("  Feedforward enabled: ", (leftArm.isFeedforwardEnabled() ? "Yes" : "No"), "");
    println("  Has limits: ", (leftArm.hasLimits() ? "Yes" : "No"), "\n");

    println("✓ Configuration stored and retrievable!\n");

    // Move to position with chaining
    println("Moving to 45 degrees...");
    leftArm.moveTo(deg(45))
           .setSpeed(1.0);

    // Simulate control loop
    print(, );
    println("\nTime(s) | Position(°) | Target(°) | Duty(%) | Status");
    println("--------|-------------|-----------|---------|--------");

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
            print(, t, " | ");
            print(, sim_position.toDegrees(), " | ");
            print(, leftArm.getTarget().toDegrees(), " | ");
            print(, (leftArm.getDutyCycle() * 100.0), " | ");
            println((leftArm.isAtTarget() ? "At target" : "Moving"), "");
        }
    }

    // Chain multiple movements
    println("\nChaining multiple movements:");
    leftArm.moveTo(deg(90))
           .setSpeed(0.5)
           .update(0.1, deg(50))
           .update(0.1, deg(55));

    println("  Target: ", leftArm.getTarget().toDegrees(), "°");
    println("  Position: ", leftArm.getPosition().toDegrees(), "°");
    println("  Error: ", leftArm.getError() * 57.3, "°\n");
}

void demonstrateDifferentialDrive() {
    println("========================================");
    println("  Fluent Differential Drive");
    println("========================================\n");

    // Create and configure with chaining
    DifferentialDrive robot = DifferentialDrive()
        .withWheelbase(m(0.5))
        .withWheelDiameter(m(0.1))
        .withMaxSpeed(mps(2.0));

    println("Robot configured:");
    println("  Wheelbase: ", robot.getWheelbase().toMeters(), "m");
    println("  Wheel diameter: ", robot.getWheelDiameter().toMeters(), "m");
    println("  Max speed: ", robot.getMaxSpeed().toMetersPerSecond(), " m/s\n");

    println("✓ Configuration stored - access anytime with robot.getWheelbase(), etc.\n");

    // Drive forward with chaining
    println("Arcade drive - Forward 50%, Turn left 30%:");
    robot.arcade(0.5, -0.3);

    println("  Left motor: ", (robot.getLeftDuty() * 100.0), "%");
    println("  Right motor: ", (robot.getRightDuty() * 100.0), "%\n");

    // Chain multiple drive commands
    println("Chaining: Drive forward -> Turn -> Stop");
    robot.drive(mps(1.0), radps(0.0))  // Forward
         .arcade(0.0, 0.5)              // Turn right
         .stop();                        // Stop

    println("  Final state: Stopped");
    println("  Left duty: ", robot.getLeftDuty(), "");
    println("  Right duty: ", robot.getRightDuty(), "\n");

    // Tank drive
    println("Tank drive - Spin in place:");
    robot.tank(0.5, -0.5);  // Left forward, right backward

    println("  Left motor: ", (robot.getLeftDuty() * 100.0), "%");
    println("  Right motor: ", (robot.getRightDuty() * 100.0), "%\n");

    // Demonstrate comprehensive state tracking with odometry
    println("State tracking with odometry:");
    robot.resetOdometry(0.0, 0.0, 0.0);  // Start at origin

    // Simulate driving forward with encoder feedback
    for (int i = 0; i < 5; i++) {
        // Simulate wheel velocities from encoders
        robot.updateVelocities(mps(1.0), mps(1.0))  // Both wheels 1 m/s
              .updateOdometry(0.1);  // Update position every 0.1s

        if (i == 4) {
            println("  Position: (", robot.getX(), ", ", robot.getY(), ") m");
            println("  Heading: ", robot.getThetaDegrees(), "°");
            println("  Left velocity: ", robot.getLeftVelocity().toMetersPerSecond(), " m/s");
            println("  Right velocity: ", robot.getRightVelocity().toMetersPerSecond(), " m/s");
        }
    }

    println("\n✓ Complete robot state stored and accessible!");
    println("  • Configuration: wheelbase, wheel diameter, max speed");
    println("  • Command state: duty cycles, commanded velocities");
    println("  • Measured state: encoder distances, wheel velocities");
    println("  • Odometry state: X, Y, theta position\n");
}

void demonstrateSensorFiltering() {
    println("========================================");
    println("  Fluent Sensor Filtering");
    println("========================================\n");

    // Create sensor with filtering (chained!)
    Sensor distanceSensor = Sensor()
        .withLowPassFilter(0.3)
        .withMovingAverage();

    println("Sensor configured with:");
    println("  - Low-pass filter (alpha=", distanceSensor.getLowPassAlpha(), ")");
    println("  - Moving average enabled: ", (distanceSensor.isMovingAverageEnabled() ? "Yes" : "No"), "");
    println("  - Low-pass enabled: ", (distanceSensor.isLowPassEnabled() ? "Yes" : "No"), "\n");

    println("✓ Filter configuration retrievable!\n");

    println("Reading noisy data:");
    println("Sample | Raw   | Filtered");
    println("-------|-------|----------");

    // Simulate noisy sensor readings with chaining
    double noisy_readings[] = {100.0, 102.0, 98.0, 101.0, 99.0, 100.5, 101.5, 99.5};

    for (size_t i = 0; i < 8; ++i) {
        // Chain read and query operations
        distanceSensor.read(noisy_readings[i]);

        print(, (i + 1), " | ");
        print(, );
        print(, distanceSensor.getRawValue(), " | ");
        println(, distanceSensor.getValue(), "");
    }

    println("\nFiltered value is smoother!\n");
}

void demonstratePIDController() {
    println("========================================");
    println("  Fluent PID Controller");
    println("========================================\n");

    // Configure PID with chaining
    PID controller = PID()
        .withGains(1.2, 0.1, 0.05)
        .withOutputLimits(-1.0, 1.0)
        .withIntegralLimit(10.0);

    println("PID configured:");
    println("  Gains: kP=", controller.getKP(), ", kI=", controller.getKI(), ", kD=", controller.getKD(), "");
    println("  Output: -1.0 to 1.0");
    println("  Integral limit: 10.0\n");

    println("✓ PID gains stored and retrievable!\n");

    println("Controlling to setpoint=100:");
    println("Step | Error | Output");
    println("-----|-------|-------");

    double measured = 0.0;
    double setpoint = 100.0;

    for (int step = 0; step < 10; step++) {
        double error = setpoint - measured;

        // Calculate and query output (chaining!)
        controller.calculate(error, 0.1);

        print(, step, " | ");
        print(, );
        print(, controller.getError(), " | ");
        println(, controller.getOutput(), "");

        // Simulate response
        measured += controller.getOutput() * 10.0;
    }

    println("\nFinal measured value: ", measured, "\n");
}

void demonstrateComplexChaining() {
    println("========================================");
    println("  Complex Method Chaining");
    println("========================================\n");

    println("Creating a complete robot arm subsystem:\n");

    // Everything configured in one beautiful chain!
    Arm shoulder = Arm()
        .withPID(2.0, 0.15, 0.08)
        .withFeedforward(0.1, 0.002)
        .withLimits(deg(0), deg(180))
        .setSpeed(0.9)
        .moveTo(deg(90))
        .resetPosition(deg(0));

    println("Shoulder configured and ready!");
    println("  Current: ", shoulder.getPosition().toDegrees(), "°");
    println("  Target: ", shoulder.getTarget().toDegrees(), "°");
    println("  Error: ", (shoulder.getError() * 57.3), "°\n");

    // Complex control sequence - all chained!
    println("Executing movement sequence:");
    shoulder.moveTo(deg(45))
            .setSpeed(0.5)
            .update(0.1, deg(10))
            .update(0.1, deg(20))
            .setSpeed(1.0)
            .moveTo(deg(135));

    println("  New target: ", shoulder.getTarget().toDegrees(), "°");
    println("  Speed: 100%\n");

    // Create complete robot with chaining
    println("Creating complete robot:");

    DifferentialDrive chassis = DifferentialDrive()
        .withWheelbase(m(0.6))
        .withWheelDiameter(m(0.15))
        .withMaxSpeed(mps(3.0));

    Sensor rangefinder = Sensor()
        .withLowPassFilter(0.2)
        .withMovingAverage();

    println("  ✓ Chassis configured");
    println("  ✓ Arm configured");
    println("  ✓ Sensor configured\n");

    // Control everything together with chaining!
    chassis.arcade(0.8, 0.0);           // Drive forward
    shoulder.moveTo(deg(120));          // Raise arm
    rangefinder.read(150.5);            // Read sensor

    println("Robot state:");
    println("  Chassis: ", (chassis.getLeftDuty() * 100.0), "% throttle");
    println("  Arm: ", shoulder.getTarget().toDegrees(), "° target");
    println("  Range: ", rangefinder.getValue(), " cm\n");
}

int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  RobotLib Fluent API Demo             ║");
    println("║  Beautiful Method Chaining             ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrateArmControl();
    demonstrateDifferentialDrive();
    demonstrateSensorFiltering();
    demonstratePIDController();
    demonstrateComplexChaining();

    println("========================================");
    println("  Key Advantages of Fluent API");
    println("========================================\n");

    println("✅ Readable: Configuration reads like English");
    println("✅ Compact: Multiple actions in one line");
    println("✅ Type-safe: Still uses RobotLib's unit system");
    println("✅ Discoverable: IDE autocomplete shows all options");
    println("✅ Flexible: Chain as much or as little as you want\n");

    println("Example usage patterns:\n");

    println("1. Configuration chaining:");
    println("   Arm().withPID(1,0,0).withLimits(deg(-90), deg(90));\n");

    println("2. Action chaining:");
    println("   arm.moveTo(deg(90)).setSpeed(0.5).update(dt, pos);\n");

    println("3. Mixed chaining:");
    println("   robot.withWheelbase(m(0.5)).drive(mps(1), radps(0));\n");

    println("4. Query ends chain:");
    println("   double duty = arm.moveTo(deg(45)).getDutyCycle();\n");

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
