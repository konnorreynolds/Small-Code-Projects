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

#include "../../include/RobotLib.h"

using namespace robotlib;
using namespace robotlib::output;

int main() {
    println("========================================");
    println("  Simple Differential Drive");
    println("  (No Odometry or Encoders)");
    println("========================================\n");

    // Configure robot - just the physical parameters
    DifferentialDrive robot = DifferentialDrive()
        .withWheelbase(m(0.3))          // 30cm between wheels
        .withWheelDiameter(m(0.1))      // 10cm diameter wheels
        .withMaxSpeed(mps(1.0));        // 1 m/s max speed

    println("Robot configured:");
    println("  Wheelbase: ", robot.getWheelbase().toMeters(), " m");
    println("  Wheel diameter: ", robot.getWheelDiameter().toMeters(), " m");
    println("  Max speed: ", robot.getMaxSpeed().toMetersPerSecond(), " m/s");
    println("  (No encoders or odometry configured)\n");

    // Example 1: Arcade drive (like a video game)
    println("Example 1: Arcade Drive Control");
    println("  Command       | Forward | Turn  | Left(%) | Right(%)");
    println("  --------------|---------|-------|---------|----------");

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

        print("  " ,   cmd.name , " | ");
    }

    println("");

    // Example 2: Tank drive (independent wheel control)
    println("Example 2: Tank Drive Control");
    println("  Command       | Left  | Right");
    println("  --------------|-------|-------");

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

        print("  " ,   cmd.name , " | ");
    }

    println("");

    // Example 3: Timed movement sequence (open-loop)
    println("Example 3: Timed Movement Sequence");
    println("  (Run each command for specified time)\n");

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

    println("  Step | Action         | Duration(s)");
    println("  -----|----------------|------------");

    int step = 1;
    for (const auto& move : sequence) {
        robot.arcade(move.forward, move.turn);

        print("  " , step++ , " | ");
    }

    println("");

    // Example 4: Direct velocity control
    println("Example 4: Direct Velocity Control");
    robot.drive(mps(0.5), radps(0.3));

    println("  Commanded linear: ", robot.getLinearVelocity().toMetersPerSecond(), " m/s");
    println("  Commanded angular: ", robot.getAngularVelocity().toRadiansPerSecond(), " rad/s");
    println("  Left duty: ", (robot.getLeftDuty() * 100.0), "%");
    println("  Right duty: ", (robot.getRightDuty() * 100.0), "%");

    println("\n========================================");
    println("✓ Simple drive control without feedback");
    println("  • No encoders or sensors needed");
    println("  • Open-loop control only");
    println("  • Use timers for movement duration");
    println("  • Perfect for basic robots");
    println("\nTypical usage pattern:");
    println("  1. Configure robot parameters");
    println("  2. Send drive commands");
    println("  3. Get duty cycles for motors");
    println("  4. Use hardware timers for duration");
    println("========================================");

    return 0;
}
