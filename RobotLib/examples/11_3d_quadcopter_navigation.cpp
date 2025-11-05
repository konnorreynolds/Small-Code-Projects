// ============================================================================
// Example 11: 3D Quadcopter Navigation with EKF and Path Planning
// ============================================================================
// This example demonstrates the NEW advanced features in RobotLib:
// - 3D transformations and quaternions
// - Extended Kalman Filter for state estimation
// - A* path planning
// - Dubins paths for smooth trajectories
//
// Topics covered:
// - Quaternion-based 3D rotations
// - SE(3) transformations
// - EKF for drone localization
// - Grid-based path planning with obstacles
// - Complete autonomous navigation pipeline
// ============================================================================

#include "../units_core.h"
#include "../units_physics.h"
#include "../units_3d.h"
#include "../units_estimation.h"
#include "../units_planning.h"

#include <iostream>
#include <iomanip>

using namespace units;
using namespace units::spatial;
using namespace units::estimation;
using namespace units::planning;

// ============================================================================
// Demonstrate 3D Transformations and Quaternions
// ============================================================================
void demonstrate3DTransformations() {
    std::cout << "========================================\n";
    std::cout << "  3D Transformations & Quaternions\n";
    std::cout << "========================================\n\n";

    // Create 3D vectors
    Vec3D point(1, 0, 0);
    std::cout << "Original point: (" << point.x << ", " << point.y << ", "
              << point.z << ")\n\n";

    // Create rotation using quaternions (90° around Z-axis)
    Quaternion rot_z = rotationZ(constants::PI / 2.0);
    Vec3D rotated = rot_z.rotate(point);

    std::cout << "After 90° rotation around Z-axis:\n";
    std::cout << "  (" << std::fixed << std::setprecision(3)
              << rotated.x << ", " << rotated.y << ", " << rotated.z << ")\n";
    std::cout << "  → Point rotated to +Y direction\n\n";

    // Create rotation from Euler angles
    double roll = 0.0, pitch = 0.0, yaw = constants::PI / 4.0;  // 45° yaw
    Quaternion rot_euler = Quaternion::fromEuler(roll, pitch, yaw);

    std::cout << "Rotation from Euler angles (45° yaw):\n";
    std::cout << "  Quaternion: w=" << rot_euler.w << ", x=" << rot_euler.x
              << ", y=" << rot_euler.y << ", z=" << rot_euler.z << "\n\n";

    // Demonstrate SLERP (smooth interpolation)
    Quaternion start_rot = rotationZ(0);
    Quaternion end_rot = rotationZ(constants::PI / 2.0);

    std::cout << "SLERP interpolation (0° to 90° around Z):\n";
    for (double t = 0.0; t <= 1.0; t += 0.25) {
        Quaternion interp = start_rot.slerp(end_rot, t);
        double angle = interp.getAngle();
        std::cout << "  t=" << t << ": angle = "
                  << (angle * constants::RAD_TO_DEG) << "°\n";
    }
    std::cout << "\n";

    // SE(3) transformations
    std::cout << "SE(3) Pose Transformations:\n";
    Vec3D drone_position(5, 3, 2);
    Quaternion drone_orientation = rotationZ(constants::PI / 4.0);
    Pose3D drone_pose(drone_position, drone_orientation);

    Vec3D camera_offset(0.2, 0, -0.1);  // Camera 20cm forward, 10cm down
    Vec3D camera_global = drone_pose.transformPoint(camera_offset);

    std::cout << "  Drone at: (" << drone_position.x << ", "
              << drone_position.y << ", " << drone_position.z << ")\n";
    std::cout << "  Camera (in drone frame): (" << camera_offset.x << ", "
              << camera_offset.y << ", " << camera_offset.z << ")\n";
    std::cout << "  Camera (global frame): (" << camera_global.x << ", "
              << camera_global.y << ", " << camera_global.z << ")\n\n";
}

// ============================================================================
// Demonstrate Extended Kalman Filter for Drone Localization
// ============================================================================
void demonstrateEKF() {
    std::cout << "========================================\n";
    std::cout << "  Extended Kalman Filter\n";
    std::cout << "  Drone Localization\n";
    std::cout << "========================================\n\n";

    // Create EKF for 2D position and velocity tracking
    EKF2DPositionVelocity ekf(0.01, 0.5);  // Low process noise, higher measurement noise

    std::cout << "Simulating drone flight with noisy GPS measurements...\n\n";
    std::cout << "Time(s) | True Pos | GPS Meas | EKF Est  | Error\n";
    std::cout << "--------|----------|----------|----------|-------\n";

    double dt = 0.1;  // 10 Hz update rate
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 0.5);  // GPS noise

    // Simulate constant velocity flight
    double true_x = 0.0, true_y = 0.0;
    double vx = 1.0, vy = 0.5;  // 1 m/s in X, 0.5 m/s in Y

    for (double t = 0.0; t <= 2.0; t += dt) {
        // Update true position
        true_x += vx * dt;
        true_y += vy * dt;

        // EKF prediction step
        ekf.predict(dt);

        // GPS measurement (noisy)
        double gps_x = true_x + noise(gen);
        double gps_y = true_y + noise(gen);
        std::array<double, 2> measurement = {gps_x, gps_y};

        // EKF update step
        ekf.update(measurement);

        // Get estimate
        auto state = ekf.getState();
        double est_x = state[0];
        double est_y = state[1];

        // Calculate error
        double error = std::sqrt((est_x - true_x) * (est_x - true_x) +
                                (est_y - true_y) * (est_y - true_y));

        // Print every 0.5 seconds
        if (static_cast<int>(t / 0.5) != static_cast<int>((t - dt) / 0.5)) {
            std::cout << std::fixed << std::setprecision(1);
            std::cout << std::setw(7) << t << " | ";
            std::cout << "(" << std::setw(4) << true_x << ","
                      << std::setw(4) << true_y << ") | ";
            std::cout << "(" << std::setw(4) << gps_x << ","
                      << std::setw(4) << gps_y << ") | ";
            std::cout << "(" << std::setw(4) << est_x << ","
                      << std::setw(4) << est_y << ") | ";
            std::cout << std::setprecision(2) << std::setw(5) << error << "\n";
        }
    }

    std::cout << "\nKey Observations:\n";
    std::cout << "• EKF smooths noisy GPS measurements\n";
    std::cout << "• Velocity estimates help predict position\n";
    std::cout << "• Error decreases as filter converges\n\n";
}

// ============================================================================
// Demonstrate Path Planning
// ============================================================================
void demonstratePathPlanning() {
    std::cout << "========================================\n";
    std::cout << "  A* Path Planning\n";
    std::cout << "========================================\n\n";

    // Create 20x20 grid
    AStarPlanner planner(20, 20);

    // Add obstacles (building)
    std::cout << "Environment: 20x20 grid with building obstacle\n\n";
    for (int x = 8; x <= 12; x++) {
        for (int y = 8; y <= 12; y++) {
            planner.setObstacle(x, y);
        }
    }

    // Plan path
    GridCell start(2, 2);
    GridCell goal(18, 18);

    std::cout << "Planning from (" << start.x << ", " << start.y
              << ") to (" << goal.x << ", " << goal.y << ")\n\n";

    auto path = planner.plan(start, goal, true);  // Allow diagonal

    if (path.empty()) {
        std::cout << "No path found!\n\n";
        return;
    }

    std::cout << "Path found with " << path.size() << " waypoints\n";
    std::cout << "Path length: " << std::fixed << std::setprecision(2)
              << planner.getPathLength(path) << " grid cells\n\n";

    // Visualize path (simple ASCII)
    std::vector<std::vector<char>> grid(20, std::vector<char>(20, '.'));

    // Mark obstacles
    for (int x = 8; x <= 12; x++) {
        for (int y = 8; y <= 12; y++) {
            grid[y][x] = '#';
        }
    }

    // Mark path
    for (const auto& cell : path) {
        grid[cell.y][cell.x] = '*';
    }

    // Mark start and goal
    grid[start.y][start.x] = 'S';
    grid[goal.y][goal.x] = 'G';

    std::cout << "Map (S=start, G=goal, *=path, #=obstacle):\n";
    for (int y = 0; y < 20; y++) {
        for (int x = 0; x < 20; x++) {
            std::cout << grid[y][x] << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// ============================================================================
// Demonstrate Dubins Paths
// ============================================================================
void demonstrateDubinsPaths() {
    std::cout << "========================================\n";
    std::cout << "  Dubins Paths\n";
    std::cout << "  Smooth Trajectories for Drones\n";
    std::cout << "========================================\n\n";

    // Start and goal configurations
    DubinsPath::Config start(0, 0, 0);  // Origin, heading east
    DubinsPath::Config goal(10, 5, constants::PI / 2.0);  // Heading north

    double turning_radius = 2.0;  // Minimum turn radius for drone

    std::cout << "Start: (" << start.x << ", " << start.y
              << ") heading " << (start.theta * constants::RAD_TO_DEG) << "°\n";
    std::cout << "Goal:  (" << goal.x << ", " << goal.y
              << ") heading " << (goal.theta * constants::RAD_TO_DEG) << "°\n";
    std::cout << "Turning radius: " << turning_radius << " m\n\n";

    // Find shortest Dubins path
    auto path = DubinsPath::shortestPath(start, goal, turning_radius);

    std::cout << "Shortest path found:\n";
    std::cout << "  Type: ";
    for (int i = 0; i < 3; i++) {
        switch (path.types[i]) {
            case DubinsPath::SegmentType::LEFT:
                std::cout << "L";
                break;
            case DubinsPath::SegmentType::RIGHT:
                std::cout << "R";
                break;
            case DubinsPath::SegmentType::STRAIGHT:
                std::cout << "S";
                break;
        }
    }
    std::cout << "\n";

    std::cout << "  Segment lengths: "
              << path.lengths[0] << " m, "
              << path.lengths[1] << " m, "
              << path.lengths[2] << " m\n";
    std::cout << "  Total length: " << path.total_length << " m\n\n";

    // Sample points along path
    auto points = DubinsPath::sample(start, path, turning_radius, 0.5);

    std::cout << "Sampled waypoints (every 0.5m):\n";
    for (size_t i = 0; i < std::min(points.size(), size_t(10)); i++) {
        std::cout << "  " << std::setw(2) << i << ": ("
                  << std::fixed << std::setprecision(2)
                  << std::setw(5) << points[i].x << ", "
                  << std::setw(5) << points[i].y << ") heading "
                  << std::setw(5) << (points[i].theta * constants::RAD_TO_DEG) << "°\n";
    }
    if (points.size() > 10) {
        std::cout << "  ... and " << (points.size() - 10) << " more waypoints\n";
    }
    std::cout << "\n";
}

// ============================================================================
// Main Program - Complete Autonomous Navigation Pipeline
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Advanced Robotics Features           ║\n";
    std::cout << "║  3D, EKF, Path Planning                ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrate3DTransformations();
    demonstrateEKF();
    demonstratePathPlanning();
    demonstrateDubinsPaths();

    std::cout << "========================================\n";
    std::cout << "  Complete Navigation Pipeline\n";
    std::cout << "========================================\n\n";

    std::cout << "These features work together for autonomous navigation:\n\n";

    std::cout << "1. Path Planning (A* or Dubins)\n";
    std::cout << "   • Find collision-free path\n";
    std::cout << "   • Generate waypoints\n";
    std::cout << "   • Consider robot constraints\n\n";

    std::cout << "2. State Estimation (EKF)\n";
    std::cout << "   • Fuse GPS + IMU + vision\n";
    std::cout << "   • Track position and velocity\n";
    std::cout << "   • Handle sensor noise\n\n";

    std::cout << "3. 3D Transformations (Quaternions)\n";
    std::cout << "   • Track drone orientation\n";
    std::cout << "   • Transform sensor data\n";
    std::cout << "   • Smooth rotation interpolation\n\n";

    std::cout << "4. Path Following\n";
    std::cout << "   • Convert waypoints to control\n";
    std::cout << "   • Maintain desired trajectory\n";
    std::cout << "   • Handle disturbances\n\n";

    std::cout << "========================================\n";
    std::cout << "  Real-World Applications\n";
    std::cout << "========================================\n\n";

    std::cout << "Drones and UAVs:\n";
    std::cout << "  • Package delivery\n";
    std::cout << "  • Aerial photography\n";
    std::cout << "  • Search and rescue\n";
    std::cout << "  • Agricultural monitoring\n\n";

    std::cout << "Ground Robots:\n";
    std::cout << "  • Warehouse automation\n";
    std::cout << "  • Autonomous vehicles\n";
    std::cout << "  • Lawn mowers and vacuums\n";
    std::cout << "  • Security patrols\n\n";

    std::cout << "Manipulators:\n";
    std::cout << "  • 3D pick-and-place\n";
    std::cout << "  • Welding robots\n";
    std::cout << "  • Surgical robots\n";
    std::cout << "  • Assembly lines\n\n";

    std::cout << "========================================\n\n";

    return 0;
}

/*
Expected Output:
- 3D transformation demonstrations with quaternions
- EKF drone localization with noisy GPS
- A* path planning around obstacles
- Dubins path generation for smooth trajectories

This example demonstrates:
1. **Quaternions**: Gimbal-lock-free 3D rotations
2. **SE(3)**: Position + orientation transformations
3. **SLERP**: Smooth rotation interpolation
4. **EKF**: Optimal state estimation for nonlinear systems
5. **A***: Efficient grid-based path planning
6. **Dubins**: Shortest paths with turning constraints

Perfect for:
- Drone navigation systems
- 3D robot manipulation
- Autonomous vehicle control
- SLAM implementations
- Multi-sensor fusion

Compile:
    g++ -std=c++11 -I.. 11_3d_quadcopter_navigation.cpp -o quad_nav

This showcases RobotLib's NEW advanced features!
*/
