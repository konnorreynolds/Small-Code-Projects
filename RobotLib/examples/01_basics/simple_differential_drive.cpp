// ============================================================================
// Simple Differential Drive - No Odometry
// ============================================================================
// Demonstrates basic differential drive control without odometry or encoders.
// Perfect for simple robots where you just want to drive around.
//
// Use cases:
// - Basic robot control
// - RC car style driving
// - Simple timed movements
// - Motor testing
// ============================================================================

#include "../../include/robotlib_api.h"
#include <iostream>
#include <iomanip>

using namespace robotlib;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Simple Differential Drive\n";
    std::cout << "  (No Odometry or Encoders)\n";
    std::cout << "========================================\n\n";

    // Configure robot - just the physical parameters
    DifferentialDrive robot = DifferentialDrive()
        .withWheelbase(m(0.3))          // 30cm between wheels
        .withWheelDiameter(m(0.1))      // 10cm diameter wheels
        .withMaxSpeed(mps(1.0));        // 1 m/s max speed

    std::cout << "Robot configured:\n";
    std::cout << "  Wheelbase: " << robot.getWheelbase().toMeters() << " m\n";
    std::cout << "  Wheel diameter: " << robot.getWheelDiameter().toMeters() << " m\n";
    std::cout << "  Max speed: " << robot.getMaxSpeed().toMetersPerSecond() << " m/s\n";
    std::cout << "  (No encoders or odometry configured)\n\n";

    // Example 1: Arcade drive (like a video game)
    std::cout << "Example 1: Arcade Drive Control\n";
    std::cout << "  Command       | Forward | Turn  | Left(%) | Right(%)\n";
    std::cout << "  --------------|---------|-------|---------|----------\n";

    struct DriveCommand {
        const char* name;
        double forward;
        double turn;
    };

    DriveCommand commands[] = {
        {"Forward",      1.0,  0.0},
        {"Forward Left", 0.8,  0.3},
        {"Forward Right", 0.8, -0.3},
        {"Spin Left",    0.0,  1.0},
        {"Spin Right",   0.0, -1.0},
        {"Reverse",     -0.5,  0.0},
        {"Stop",         0.0,  0.0},
    };

    for (const auto& cmd : commands) {
        robot.arcade(cmd.forward, cmd.turn);

        std::cout << "  " << std::setw(13) << std::left << cmd.name << " | "
                  << std::setw(7) << std::right << std::fixed << std::setprecision(1)
                  << cmd.forward << " | "
                  << std::setw(5) << cmd.turn << " | "
                  << std::setw(7) << (robot.getLeftDuty() * 100.0) << " | "
                  << std::setw(8) << (robot.getRightDuty() * 100.0) << "\n";
    }

    std::cout << "\n";

    // Example 2: Tank drive (independent wheel control)
    std::cout << "Example 2: Tank Drive Control\n";
    std::cout << "  Command       | Left  | Right\n";
    std::cout << "  --------------|-------|-------\n";

    struct TankCommand {
        const char* name;
        double left;
        double right;
    };

    TankCommand tankCommands[] = {
        {"Forward",       1.0,  1.0},
        {"Gentle Right",  1.0,  0.5},
        {"Gentle Left",   0.5,  1.0},
        {"Spin Right",    0.5, -0.5},
        {"Spin Left",    -0.5,  0.5},
        {"Reverse",      -0.8, -0.8},
    };

    for (const auto& cmd : tankCommands) {
        robot.tank(cmd.left, cmd.right);

        std::cout << "  " << std::setw(13) << std::left << cmd.name << " | "
                  << std::setw(5) << std::right << std::fixed << std::setprecision(1)
                  << cmd.left << " | "
                  << std::setw(5) << cmd.right << "\n";
    }

    std::cout << "\n";

    // Example 3: Timed movement sequence (open-loop)
    std::cout << "Example 3: Timed Movement Sequence\n";
    std::cout << "  (Run each command for specified time)\n\n";

    struct TimedMove {
        const char* action;
        double forward;
        double turn;
        double duration;
    };

    TimedMove sequence[] = {
        {"Drive forward",  0.5,  0.0, 2.0},
        {"Turn right",     0.0, -0.5, 1.0},
        {"Drive forward",  0.5,  0.0, 2.0},
        {"Turn right",     0.0, -0.5, 1.0},
        {"Drive forward",  0.5,  0.0, 2.0},
        {"Stop",           0.0,  0.0, 0.0},
    };

    std::cout << "  Step | Action         | Duration(s)\n";
    std::cout << "  -----|----------------|------------\n";

    int step = 1;
    for (const auto& move : sequence) {
        robot.arcade(move.forward, move.turn);

        std::cout << "  " << std::setw(4) << step++ << " | "
                  << std::setw(14) << std::left << move.action << " | "
                  << std::setw(10) << std::right << std::fixed << std::setprecision(1)
                  << move.duration << "\n";
    }

    std::cout << "\n";

    // Example 4: Direct velocity control
    std::cout << "Example 4: Direct Velocity Control\n";
    robot.drive(mps(0.5), radps(0.3));

    std::cout << "  Commanded linear: " << robot.getLinearVelocity().toMetersPerSecond() << " m/s\n";
    std::cout << "  Commanded angular: " << robot.getAngularVelocity().toRadiansPerSecond() << " rad/s\n";
    std::cout << "  Left duty: " << (robot.getLeftDuty() * 100.0) << "%\n";
    std::cout << "  Right duty: " << (robot.getRightDuty() * 100.0) << "%\n";

    std::cout << "\n========================================\n";
    std::cout << "✓ Simple drive control without feedback\n";
    std::cout << "  • No encoders or sensors needed\n";
    std::cout << "  • Open-loop control only\n";
    std::cout << "  • Use timers for movement duration\n";
    std::cout << "  • Perfect for basic robots\n";
    std::cout << "\nTypical usage pattern:\n";
    std::cout << "  1. Configure robot parameters\n";
    std::cout << "  2. Send drive commands\n";
    std::cout << "  3. Get duty cycles for motors\n";
    std::cout << "  4. Use hardware timers for duration\n";
    std::cout << "========================================\n";

    return 0;
}
