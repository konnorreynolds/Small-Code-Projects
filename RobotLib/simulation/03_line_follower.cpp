// ============================================================================
// Line Follower Simulation
// ============================================================================
// Demonstrates:
// - Line sensor simulation
// - PID line following
// - Testing control parameters before hardware deployment
//
// This simulates a 5-sensor line follower robot that follows a horizontal line
// at the center of the world.
// ============================================================================

#include <RobotLib.h>
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"

using namespace robotlib;
using namespace robotlib::simulation;
using namespace robotlib::visualization;
using namespace robotlib::output;
using namespace robotics;

int main() {
    println("=== Line Follower Simulation ===");
    println("Robot will follow the horizontal line at y = 1.5m");
    println();

    // Create robot
    DifferentialDriveSimulator robot(0.12, 0.08, 1000, 4.0, 3.0);
    robot.setPosition(0.3, 1.55, 0.0);  // Start slightly off-line

    // Create visualizer
    RobotVisualizer viz(800, 600, 180, "Line Follower");

    // PID controller for line following
    PIDController linePID(
        1.5,   // Kp - Proportional (increase for faster correction)
        0.3,   // Ki - Integral (increase to eliminate steady-state error)
        0.1    // Kd - Derivative (increase to reduce oscillation)
    );

    // Sensor positions (relative to robot center)
    // 5 sensors: far left, left, center, right, far right
    double sensorSpacing = 0.025;  // 2.5cm between sensors
    double sensorOffsets[5] = {
        -2 * sensorSpacing,  // Far left
        -1 * sensorSpacing,  // Left
        0,                    // Center
        1 * sensorSpacing,   // Right
        2 * sensorSpacing    // Far right
    };

    double dt = 0.02;
    uint32_t lastTime = SDL_GetTicks();
    double fps = 0;
    uint32_t frameCount = 0;

    const double BASE_SPEED = 0.4;  // 0.4 m/s base speed
    const double MAX_CORRECTION = 0.3;

    println("Line following started!");
    println("PID parameters: Kp=", 1.5, " Ki=", 0.3, " Kd=", 0.1);
    println();

    while (viz.isRunning() && robot.getX() < 3.5) {
        if (!viz.handleEvents()) {
            break;
        }

        // Read line sensors
        bool sensors[5];
        double linePosition = 0.0;
        int sensorsOn = 0;

        for (int i = 0; i < 5; i++) {
            sensors[i] = robot.getLineSensor(0.05, sensorOffsets[i]);
            if (sensors[i]) {
                linePosition += (i - 2);  // -2, -1, 0, 1, 2
                sensorsOn++;
            }
        }

        // Calculate weighted line position
        if (sensorsOn > 0) {
            linePosition /= sensorsOn;
        } else {
            // Lost line! Use last known position
            linePosition = 0.0;  // Assume centered
        }

        // PID control - target is 0 (centered)
        double correction = linePID.calculate(0.0, linePosition, dt);

        // Limit correction
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

        // Apply differential drive
        // Negative correction = line is left, turn left (slow left motor)
        double leftSpeed = (BASE_SPEED - correction) / 0.5;  // Convert to PWM (-1 to 1)
        double rightSpeed = (BASE_SPEED + correction) / 0.5;

        robot.setMotorPWM(leftSpeed, rightSpeed);

        // Update simulation
        robot.update(dt);

        // Render
        viz.render(robot, fps);

        // Stats
        frameCount++;
        uint32_t currentTime = SDL_GetTicks();
        if (currentTime - lastTime >= 500) {
            fps = frameCount / ((currentTime - lastTime) / 1000.0);
            frameCount = 0;
            lastTime = currentTime;

            print("Position: (", robot.getX(), ", ", robot.getY(), ")  ");
            print("Line pos: ", linePosition, "  ");
            print("Sensors: ");
            for (int i = 0; i < 5; i++) {
                print(sensors[i] ? "█" : "░");
            }
            println("  FPS: ", fps);
        }

        SDL_Delay((uint32_t)(dt * 1000));
    }

    println();
    println("Line following completed!");
    println("Final position: (", robot.getX(), ", ", robot.getY(), ")");
    println("Distance traveled: ", robot.getX(), " m");

    return 0;
}
