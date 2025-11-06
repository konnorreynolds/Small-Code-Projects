// ============================================================================
// Differential Drive with Odometry
// ============================================================================
// Demonstrates differential drive WITH odometry tracking using the fluent API.
// Shows how to track robot position using wheel encoders.
//
// Use cases:
// - Autonomous navigation
// - Position tracking
// - Path following
// - Dead reckoning
// ============================================================================

#include "../../include/RobotLib.h"
#include <cmath>

using namespace robotlib;
using namespace robotlib::output;

int main() {
    println("========================================");
    println("  Differential Drive with Odometry");
    println("========================================\n");

    // Configure robot with full state tracking
    DifferentialDrive robot = DifferentialDrive()
        .withWheelbase(m(0.4))
        .withWheelDiameter(m(0.1))
        .withMaxSpeed(mps(1.0));

    println("Robot configured:");
    println("  Wheelbase: ", robot.getWheelbase().toMeters(), " m");
    println("  Wheel diameter: ", robot.getWheelDiameter().toMeters(), " m");
    println("  Odometry: Enabled\n");

    // Example 1: Drive straight and track position
    println("Example 1: Drive straight forward 2 meters");
    robot.resetOdometry(0.0, 0.0, 0.0);  // Start at origin

    println("\nTime(s) | Cmd(m/s) | X(m) | Y(m) | Theta(°)");
    println("--------|----------|------|------|----------");

    robot.drive(mps(0.5), radps(0.0));  // Drive forward at 0.5 m/s

    for (int step = 0; step <= 40; step++) {
        double t = step * 0.1;

        // Simulate encoder measurements
        robot.updateVelocities(mps(0.5), mps(0.5))
              .updateOdometry(0.1);

        if (step % 5 == 0) {
            print(, , t, " | ");
            print(,  robot.getLinearVelocity().toMetersPerSecond() , " | ");
            print(, robot.getX(), " | ");
            print(, robot.getY(), " | ");
            println(, robot.getThetaDegrees(), "");
        }
    }

    println("\nFinal position: (", robot.getX(), ", ", robot.getY(), ") m");
    print("Expected: (2.0, 0.0) m - "
              ,  (std::abs(robot.getX() - 2.0) < 0.1 ? "✓" : "✗") , "\n\n");

    // Example 2: Drive in a square
    println("Example 2: Drive in a 1m x 1m square\n");
    robot.resetOdometry(0.0, 0.0, 0.0);

    struct SquareSegment {
        const char* action;
        double duration;
        double linear;
        double angular;
    };

    SquareSegment segments[] = {
        {"Forward 1m",     2.0, 0.5,  0.0},
        {"Turn left 90°",  1.0, 0.0,  1.57},
        {"Forward 1m",     2.0, 0.5,  0.0},
        {"Turn left 90°",  1.0, 0.0,  1.57},
        {"Forward 1m",     2.0, 0.5,  0.0},
        {"Turn left 90°",  1.0, 0.0,  1.57},
        {"Forward 1m",     2.0, 0.5,  0.0},
        {"Turn left 90°",  1.0, 0.0,  1.57},
    };

    println("Segment        | X(m) | Y(m) | Heading(°)");
    println("---------------|------|------|------------");
    print("Start          | " , robot.getX() , " | ");

    for (const auto& seg : segments) {
        robot.drive(mps(seg.linear), radps(seg.angular));

        int steps = (int)(seg.duration / 0.1);
        for (int i = 0; i < steps; i++) {
            // Simulate differential velocities for turning
            double left_vel = seg.linear - seg.angular * robot.getWheelbase().toMeters() / 2.0;
            double right_vel = seg.linear + seg.angular * robot.getWheelbase().toMeters() / 2.0;

            robot.updateVelocities(mps(left_vel), mps(right_vel))
                  .updateOdometry(0.1);
        }

        print(,  seg.action , " | ");
    }

    print("\nClosure error: (" ,  std::abs(robot.getX()) , ", ");
    println("Heading error: ", std::abs(robot.getThetaDegrees()), "°\n");

    // Example 3: Drive in a circle
    println("Example 3: Drive in a circle (radius 1m)\n");
    robot.resetOdometry(0.0, 0.0, 0.0);

    double radius = 1.0;  // meters
    double linear_vel = 0.5;  // m/s
    double angular_vel = linear_vel / radius;  // rad/s for circular motion

    robot.drive(mps(linear_vel), radps(angular_vel));

    println("Time(s) | X(m) | Y(m) | Heading(°)");
    println("--------|------|------|------------");

    for (int step = 0; step <= 80; step++) {
        double t = step * 0.1;

        // Calculate differential wheel speeds for circular path
        double left_vel = linear_vel - angular_vel * robot.getWheelbase().toMeters() / 2.0;
        double right_vel = linear_vel + angular_vel * robot.getWheelbase().toMeters() / 2.0;

        robot.updateVelocities(mps(left_vel), mps(right_vel))
              .updateOdometry(0.1);

        if (step % 10 == 0) {
            print(, , t, " | ");
            print(, robot.getX(), " | ");
            print(, robot.getY(), " | ");
            println(, robot.getThetaDegrees(), "");
        }
    }

    println("");

    // Example 4: Retrieve complete robot state
    println("Example 4: Complete Robot State\n");

    println("Configuration:");
    println("  Wheelbase: ", robot.getWheelbase().toMeters(), " m");
    println("  Wheel diameter: ", robot.getWheelDiameter().toMeters(), " m");
    println("  Max speed: ", robot.getMaxSpeed().toMetersPerSecond(), " m/s\n");

    println("Command State:");
    println("  Left duty: ", (robot.getLeftDuty() * 100.0), "%");
    println("  Right duty: ", (robot.getRightDuty() * 100.0), "%");
    println("  Linear velocity: ", robot.getLinearVelocity().toMetersPerSecond(), " m/s");
    println("  Angular velocity: ", robot.getAngularVelocity().toRadiansPerSecond(), " rad/s\n");

    println("Measured State:");
    println("  Left velocity: ", robot.getLeftVelocity().toMetersPerSecond(), " m/s");
    println("  Right velocity: ", robot.getRightVelocity().toMetersPerSecond(), " m/s\n");

    println("Odometry State:");
    println("  X position: ", robot.getX(), " m");
    println("  Y position: ", robot.getY(), " m");
    println("  Heading: ", robot.getThetaDegrees(), "°");

    println("\n========================================");
    println("✓ Drive control with odometry tracking");
    println("  • Closed-loop position tracking");
    println("  • Real-time state estimation");
    println("  • Encoder-based odometry");
    println("  • Complete state accessible");
    println("\nState organization:");
    println("  • Configuration: Hardware parameters");
    println("  • Command: What we told it to do");
    println("  • Measured: Sensor feedback");
    println("  • Odometry: Calculated position");
    println("========================================");

    return 0;
}
