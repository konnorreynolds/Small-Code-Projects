// ============================================================================
// Basic Robot Simulation Example
// ============================================================================
// Demonstrates:
// - Creating a simulated robot
// - Visualizing in real-time
// - Simple motor control
// - Obstacle avoidance
//
// Controls:
// - ESC: Exit
// - T: Toggle path tracing
// - C: Clear path
//
// Compile:
//   g++ -std=c++11 -I../include 01_basic_simulation.cpp -lSDL2 -o basic_sim
//
// Run:
//   ./basic_sim
// ============================================================================

#include <RobotLib.h>
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"

using namespace robotlib;
using namespace robotlib::simulation;
using namespace robotlib::visualization;
using namespace robotlib::output;

int main() {
    println("=== RobotLib Simulation ===");
    println("Controls:");
    println("  ESC - Exit");
    println("  T   - Toggle path tracing");
    println("  C   - Clear path");
    println();

    // Create robot simulator
    DifferentialDriveSimulator robot(
        0.15,  // 15cm wheelbase
        0.10,  // 10cm robot radius
        1000,  // 1000 encoder counts per meter
        4.0,   // 4m world width
        3.0    // 3m world height
    );

    // Set starting position
    robot.setPosition(0.5, 0.5, 0.0);

    // Add some obstacles
    robot.addObstacle(Rectangle(2.0, 1.5, 0.5, 0.5));
    robot.addObstacle(Rectangle(1.0, 2.2, 0.3, 0.8));
    robot.addObstacle(Rectangle(3.0, 0.5, 0.4, 0.4));

    // Create visualizer
    RobotVisualizer viz(800, 600, 150, "Basic Robot Simulation");

    // Simulation loop
    double dt = 0.02;  // 20ms timestep (50 Hz)
    uint32_t lastTime = SDL_GetTicks();
    uint32_t frameCount = 0;
    double fps = 0;

    println("Simulation started!");
    println();

    while (viz.isRunning()) {
        // Handle input
        if (!viz.handleEvents()) {
            break;
        }

        // Simple obstacle avoidance behavior
        double frontDistance = robot.getUltrasonicDistance(0);

        if (frontDistance < 0.3) {
            // Too close! Turn
            robot.setMotorPWM(-0.3, 0.3);  // Turn left
        } else if (frontDistance < 0.5) {
            // Slowing down
            robot.setMotorPWM(0.3, 0.3);
        } else {
            // Full speed ahead
            robot.setMotorPWM(0.6, 0.6);
        }

        // Update physics
        robot.update(dt);

        // Render
        viz.render(robot, fps);

        // FPS calculation
        frameCount++;
        uint32_t currentTime = SDL_GetTicks();
        if (currentTime - lastTime >= 1000) {
            fps = frameCount / ((currentTime - lastTime) / 1000.0);
            frameCount = 0;
            lastTime = currentTime;

            // Print stats
            print("Position: (", robot.getX(), ", ", robot.getY(), ")  ");
            print("Angle: ", robot.getTheta() * 57.3, "°  ");
            println("FPS: ", fps);
        }

        // Cap framerate
        SDL_Delay((uint32_t)(dt * 1000));
    }

    println();
    println("Simulation ended.");
    println("Final position: (", robot.getX(), ", ", robot.getY(), ")");

    return 0;
}
