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

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace robotlib;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Differential Drive with Odometry\n";
    std::cout << "========================================\n\n";

    // Configure robot with full state tracking
    DifferentialDrive robot = DifferentialDrive()
        .withWheelbase(m(0.4))
        .withWheelDiameter(m(0.1))
        .withMaxSpeed(mps(1.0));

    std::cout << "Robot configured:\n";
    std::cout << "  Wheelbase: " << robot.getWheelbase().toMeters() << " m\n";
    std::cout << "  Wheel diameter: " << robot.getWheelDiameter().toMeters() << " m\n";
    std::cout << "  Odometry: Enabled\n\n";

    // Example 1: Drive straight and track position
    std::cout << "Example 1: Drive straight forward 2 meters\n";
    robot.resetOdometry(0.0, 0.0, 0.0);  // Start at origin

    std::cout << "\nTime(s) | Cmd(m/s) | X(m) | Y(m) | Theta(°)\n";
    std::cout << "--------|----------|------|------|----------\n";

    robot.drive(mps(0.5), radps(0.0));  // Drive forward at 0.5 m/s

    for (int step = 0; step <= 40; step++) {
        double t = step * 0.1;

        // Simulate encoder measurements
        robot.updateVelocities(mps(0.5), mps(0.5))
              .updateOdometry(0.1);

        if (step % 5 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | ";
            std::cout << std::setw(8) << std::setprecision(2)
                      << robot.getLinearVelocity().toMetersPerSecond() << " | ";
            std::cout << std::setw(4) << std::setprecision(2) << robot.getX() << " | ";
            std::cout << std::setw(4) << std::setprecision(2) << robot.getY() << " | ";
            std::cout << std::setw(8) << std::setprecision(1) << robot.getThetaDegrees() << "\n";
        }
    }

    std::cout << "\nFinal position: (" << robot.getX() << ", " << robot.getY() << ") m\n";
    std::cout << "Expected: (2.0, 0.0) m - "
              << (std::abs(robot.getX() - 2.0) < 0.1 ? "✓" : "✗") << "\n\n";

    // Example 2: Drive in a square
    std::cout << "Example 2: Drive in a 1m x 1m square\n\n";
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

    std::cout << "Segment        | X(m) | Y(m) | Heading(°)\n";
    std::cout << "---------------|------|------|------------\n";
    std::cout << "Start          | " << std::setw(4) << std::fixed << std::setprecision(2)
              << robot.getX() << " | " << robot.getY() << " | "
              << std::setw(10) << std::setprecision(1) << robot.getThetaDegrees() << "\n";

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

        std::cout << std::setw(14) << std::left << seg.action << " | "
                  << std::setw(4) << std::right << std::fixed << std::setprecision(2)
                  << robot.getX() << " | "
                  << std::setw(4) << robot.getY() << " | "
                  << std::setw(10) << std::setprecision(1) << robot.getThetaDegrees() << "\n";
    }

    std::cout << "\nClosure error: (" << std::abs(robot.getX()) << ", "
              << std::abs(robot.getY()) << ") m\n";
    std::cout << "Heading error: " << std::abs(robot.getThetaDegrees()) << "°\n\n";

    // Example 3: Drive in a circle
    std::cout << "Example 3: Drive in a circle (radius 1m)\n\n";
    robot.resetOdometry(0.0, 0.0, 0.0);

    double radius = 1.0;  // meters
    double linear_vel = 0.5;  // m/s
    double angular_vel = linear_vel / radius;  // rad/s for circular motion

    robot.drive(mps(linear_vel), radps(angular_vel));

    std::cout << "Time(s) | X(m) | Y(m) | Heading(°)\n";
    std::cout << "--------|------|------|------------\n";

    for (int step = 0; step <= 80; step++) {
        double t = step * 0.1;

        // Calculate differential wheel speeds for circular path
        double left_vel = linear_vel - angular_vel * robot.getWheelbase().toMeters() / 2.0;
        double right_vel = linear_vel + angular_vel * robot.getWheelbase().toMeters() / 2.0;

        robot.updateVelocities(mps(left_vel), mps(right_vel))
              .updateOdometry(0.1);

        if (step % 10 == 0) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(1) << t << " | ";
            std::cout << std::setw(4) << std::setprecision(2) << robot.getX() << " | ";
            std::cout << std::setw(4) << robot.getY() << " | ";
            std::cout << std::setw(10) << std::setprecision(1) << robot.getThetaDegrees() << "\n";
        }
    }

    std::cout << "\n";

    // Example 4: Retrieve complete robot state
    std::cout << "Example 4: Complete Robot State\n\n";

    std::cout << "Configuration:\n";
    std::cout << "  Wheelbase: " << robot.getWheelbase().toMeters() << " m\n";
    std::cout << "  Wheel diameter: " << robot.getWheelDiameter().toMeters() << " m\n";
    std::cout << "  Max speed: " << robot.getMaxSpeed().toMetersPerSecond() << " m/s\n\n";

    std::cout << "Command State:\n";
    std::cout << "  Left duty: " << (robot.getLeftDuty() * 100.0) << "%\n";
    std::cout << "  Right duty: " << (robot.getRightDuty() * 100.0) << "%\n";
    std::cout << "  Linear velocity: " << robot.getLinearVelocity().toMetersPerSecond() << " m/s\n";
    std::cout << "  Angular velocity: " << robot.getAngularVelocity().toRadiansPerSecond() << " rad/s\n\n";

    std::cout << "Measured State:\n";
    std::cout << "  Left velocity: " << robot.getLeftVelocity().toMetersPerSecond() << " m/s\n";
    std::cout << "  Right velocity: " << robot.getRightVelocity().toMetersPerSecond() << " m/s\n\n";

    std::cout << "Odometry State:\n";
    std::cout << "  X position: " << robot.getX() << " m\n";
    std::cout << "  Y position: " << robot.getY() << " m\n";
    std::cout << "  Heading: " << robot.getThetaDegrees() << "°\n";

    std::cout << "\n========================================\n";
    std::cout << "✓ Drive control with odometry tracking\n";
    std::cout << "  • Closed-loop position tracking\n";
    std::cout << "  • Real-time state estimation\n";
    std::cout << "  • Encoder-based odometry\n";
    std::cout << "  • Complete state accessible\n";
    std::cout << "\nState organization:\n";
    std::cout << "  • Configuration: Hardware parameters\n";
    std::cout << "  • Command: What we told it to do\n";
    std::cout << "  • Measured: Sensor feedback\n";
    std::cout << "  • Odometry: Calculated position\n";
    std::cout << "========================================\n";

    return 0;
}
