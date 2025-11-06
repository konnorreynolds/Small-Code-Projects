// ============================================================================
// PID Navigation Simulation
// ============================================================================
// Demonstrates:
// - PID controller for robot navigation
// - Waypoint following
// - Real-time visualization
// - Same code that would run on real robot!
//
// This shows how you can develop and test robot code in simulation
// before deploying to hardware.
// ============================================================================

#include <RobotLib.h>
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"
#include <cmath>

using namespace robotlib;
using namespace robotlib::simulation;
using namespace robotlib::visualization;
using namespace robotlib::output;
using namespace units;
using namespace robotics;

int main() {
    println("=== PID Navigation Simulation ===");
    println("Robot will navigate through waypoints using PID control");
    println();

    // Create robot simulator
    DifferentialDriveSimulator robot(
        0.15,  // wheelbase
        0.10,  // radius
        1000,  // encoder counts
        5.0,   // world width
        5.0    // world height
    );

    robot.setPosition(0.5, 0.5, 0.0);

    // Add obstacles
    robot.addObstacle(Rectangle(2.5, 2.5, 1.0, 0.2));
    robot.addObstacle(Rectangle(1.5, 3.5, 0.3, 0.8));
    robot.addObstacle(Rectangle(3.5, 1.5, 0.5, 0.5));

    // Create visualizer
    RobotVisualizer viz(800, 800, 150, "PID Navigation");

    // PID controllers
    PIDController headingPID(2.0, 0.1, 0.3);  // For steering
    PIDController speedPID(1.0, 0.05, 0.1);    // For speed control

    // Waypoints to visit
    std::vector<Point2D> waypoints = {
        Point2D(0.5, 0.5),   // Start
        Point2D(4.0, 1.0),   // Bottom right
        Point2D(4.0, 4.0),   // Top right
        Point2D(1.0, 4.0),   // Top left
        Point2D(0.5, 0.5)    // Back to start
    };

    size_t currentWaypoint = 1;  // Start with second waypoint
    double waypointTolerance = 0.15;  // 15cm tolerance

    double dt = 0.02;  // 20ms timestep
    uint32_t lastTime = SDL_GetTicks();
    double fps = 0;
    uint32_t frameCount = 0;

    println("Starting navigation...");

    while (viz.isRunning() && currentWaypoint < waypoints.size()) {
        if (!viz.handleEvents()) {
            break;
        }

        // Get current state
        double x = robot.getX();
        double y = robot.getY();
        double theta = robot.getTheta();

        // Calculate error to waypoint
        Point2D target = waypoints[currentWaypoint];
        double dx = target.x - x;
        double dy = target.y - y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double targetAngle = std::atan2(dy, dx);

        // Angle error
        double angleError = targetAngle - theta;
        while (angleError > M_PI) angleError -= 2 * M_PI;
        while (angleError < -M_PI) angleError += 2 * M_PI;

        // Check if reached waypoint
        if (distance < waypointTolerance) {
            println("Reached waypoint ", currentWaypoint, " at (", target.x, ", ", target.y, ")");
            currentWaypoint++;
            headingPID.reset();
            speedPID.reset();
            continue;
        }

        // PID control
        double targetSpeed = 0.3;  // 0.3 m/s target speed

        // Slow down as we approach waypoint
        if (distance < 0.5) {
            targetSpeed *= distance / 0.5;
        }

        double steeringCorrection = headingPID.calculate(0.0, angleError, dt);
        double speedControl = speedPID.calculate(targetSpeed,
                                (robot.getLeftVelocity() + robot.getRightVelocity()) / 2.0, dt);

        // Apply control
        double baseSpeed = 0.5 * speedControl;
        double leftSpeed = baseSpeed - steeringCorrection;
        double rightSpeed = baseSpeed + steeringCorrection;

        // Obstacle avoidance override
        double frontDist = robot.getUltrasonicDistance(0);
        if (frontDist < 0.25) {
            // Emergency stop and turn
            leftSpeed = -0.2;
            rightSpeed = 0.2;
        }

        robot.setMotorPWM(leftSpeed, rightSpeed);

        // Update simulation
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

            print("Waypoint ", currentWaypoint, "/", waypoints.size() - 1, "  ");
            print("Distance: ", distance, " m  ");
            println("FPS: ", fps);
        }

        SDL_Delay((uint32_t)(dt * 1000));
    }

    println();
    if (currentWaypoint >= waypoints.size()) {
        println("✓ Navigation complete! All waypoints reached.");
    } else {
        println("Navigation interrupted.");
    }

    return 0;
}
