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

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_3d.h"
#include "../../include/units_estimation.h"
#include "../../include/units_planning.h"


using namespace units;
using namespace units::spatial;
using namespace units::estimation;
using namespace units::planning;

// ============================================================================
// Demonstrate 3D Transformations and Quaternions
// ============================================================================
void demonstrate3DTransformations() {
    println("========================================");
    println("  3D Transformations & Quaternions");
    println("========================================\n");

    // Create 3D vectors
    Vec3D point(1, 0, 0);
    print("Original point: (" ,  point.x , ", ");

    // Create rotation using quaternions (90° around Z-axis)
    Quaternion rot_z = rotationZ(constants::PI / 2.0);
    Vec3D rotated = rot_z.rotate(point);

    println("After 90° rotation around Z-axis:");
    print("  (" ,   rotated.x , ", ");
    println("  → Point rotated to +Y direction\n");

    // Create rotation from Euler angles
    double roll = 0.0, pitch = 0.0, yaw = constants::PI / 4.0;  // 45° yaw
    Quaternion rot_euler = Quaternion::fromEuler(roll, pitch, yaw);

    println("Rotation from Euler angles (45° yaw):");
    print("  Quaternion: w=" ,  rot_euler.w , ", x=");

    // Demonstrate SLERP (smooth interpolation)
    Quaternion start_rot = rotationZ(0);
    Quaternion end_rot = rotationZ(constants::PI / 2.0);

    println("SLERP interpolation (0° to 90° around Z):");
    for (double t = 0.0; t <= 1.0; t += 0.25) {
        Quaternion interp = start_rot.slerp(end_rot, t);
        double angle = interp.getAngle();
        print("  t=" ,  t , ": angle = ");
    }
    println("");

    // SE(3) transformations
    println("SE(3) Pose Transformations:");
    Vec3D drone_position(5, 3, 2);
    Quaternion drone_orientation = rotationZ(constants::PI / 4.0);
    Pose3D drone_pose(drone_position, drone_orientation);

    Vec3D camera_offset(0.2, 0, -0.1);  // Camera 20cm forward, 10cm down
    Vec3D camera_global = drone_pose.transformPoint(camera_offset);

    print("  Drone at: (" ,  drone_position.x , ", ");
    print("  Camera (in drone frame): (" ,  camera_offset.x , ", ");
    print("  Camera (global frame): (" ,  camera_global.x , ", ");
}

// ============================================================================
// Demonstrate Extended Kalman Filter for Drone Localization
// ============================================================================
void demonstrateEKF() {
    println("========================================");
    println("  Extended Kalman Filter");
    println("  Drone Localization");
    println("========================================\n");

    // Create EKF for 2D position and velocity tracking
    EKF2DPositionVelocity ekf(0.01, 0.5);  // Low process noise, higher measurement noise

    println("Simulating drone flight with noisy GPS measurements...\n");
    println("Time(s) | True Pos | GPS Meas | EKF Est  | Error");
    println("--------|----------|----------|----------|-------");

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
            print(, );
            print(, t, " | ");
            print("(" , true_x , ",");
            print("(" , gps_x , ",");
            print("(" , est_x , ",");
            println(, error, "");
        }
    }

    println("\nKey Observations:");
    println("• EKF smooths noisy GPS measurements");
    println("• Velocity estimates help predict position");
    println("• Error decreases as filter converges\n");
}

// ============================================================================
// Demonstrate Path Planning
// ============================================================================
void demonstratePathPlanning() {
    println("========================================");
    println("  A* Path Planning");
    println("========================================\n");

    // Create 20x20 grid
    AStarPlanner planner(20, 20);

    // Add obstacles (building)
    println("Environment: 20x20 grid with building obstacle\n");
    for (int x = 8; x <= 12; x++) {
        for (int y = 8; y <= 12; y++) {
            planner.setObstacle(x, y);
        }
    }

    // Plan path
    GridCell start(2, 2);
    GridCell goal(18, 18);

    print("Planning from (" ,  start.x , ", ");

    auto path = planner.plan(start, goal, true);  // Allow diagonal

    if (path.empty()) {
        println("No path found!\n");
        return;
    }

    println("Path found with ", path.size(), " waypoints");
    print("Path length: " ,   planner.getPathLength(path) , " grid cells\n\n");

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

    println("Map (S=start, G=goal, *=path, #=obstacle):");
    for (int y = 0; y < 20; y++) {
        for (int x = 0; x < 20; x++) {
            print(grid[y][x], " ");
        }
        println("");
    }
    println("");
}

// ============================================================================
// Demonstrate Dubins Paths
// ============================================================================
void demonstrateDubinsPaths() {
    println("========================================");
    println("  Dubins Paths");
    println("  Smooth Trajectories for Drones");
    println("========================================\n");

    // Start and goal configurations
    DubinsPath::Config start(0, 0, 0);  // Origin, heading east
    DubinsPath::Config goal(10, 5, constants::PI / 2.0);  // Heading north

    double turning_radius = 2.0;  // Minimum turn radius for drone

    print("Start: (" ,  start.x , ", ");
    print("Goal:  (" ,  goal.x , ", ");
    println("Turning radius: ", turning_radius, " m\n");

    // Find shortest Dubins path
    auto path = DubinsPath::shortestPath(start, goal, turning_radius);

    println("Shortest path found:");
    print("  Type: ");
    for (int i = 0; i < 3; i++) {
        switch (path.types[i]) {
            case DubinsPath::SegmentType::LEFT:
                print("L");
                break;
            case DubinsPath::SegmentType::RIGHT:
                print("R");
                break;
            case DubinsPath::SegmentType::STRAIGHT:
                print("S");
                break;
        }
    }
    println("");

    print("  Segment lengths: "
              ,  path.lengths[0] , " m, ");
    println("  Total length: ", path.total_length, " m\n");

    // Sample points along path
    auto points = DubinsPath::sample(start, path, turning_radius, 0.5);

    println("Sampled waypoints (every 0.5m):");
    for (size_t i = 0; i < std::min(points.size(), size_t(10)); i++) {
        print("  " , i , ": (");
    }
    if (points.size() > 10) {
        println("  ... and ", (points.size() - 10), " more waypoints");
    }
    println("");
}

// ============================================================================
// Main Program - Complete Autonomous Navigation Pipeline
// ============================================================================
int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  Advanced Robotics Features           ║");
    println("║  3D, EKF, Path Planning                ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrate3DTransformations();
    demonstrateEKF();
    demonstratePathPlanning();
    demonstrateDubinsPaths();

    println("========================================");
    println("  Complete Navigation Pipeline");
    println("========================================\n");

    println("These features work together for autonomous navigation:\n");

    println("1. Path Planning (A* or Dubins)");
    println("   • Find collision-free path");
    println("   • Generate waypoints");
    println("   • Consider robot constraints\n");

    println("2. State Estimation (EKF)");
    println("   • Fuse GPS + IMU + vision");
    println("   • Track position and velocity");
    println("   • Handle sensor noise\n");

    println("3. 3D Transformations (Quaternions)");
    println("   • Track drone orientation");
    println("   • Transform sensor data");
    println("   • Smooth rotation interpolation\n");

    println("4. Path Following");
    println("   • Convert waypoints to control");
    println("   • Maintain desired trajectory");
    println("   • Handle disturbances\n");

    println("========================================");
    println("  Real-World Applications");
    println("========================================\n");

    println("Drones and UAVs:");
    println("  • Package delivery");
    println("  • Aerial photography");
    println("  • Search and rescue");
    println("  • Agricultural monitoring\n");

    println("Ground Robots:");
    println("  • Warehouse automation");
    println("  • Autonomous vehicles");
    println("  • Lawn mowers and vacuums");
    println("  • Security patrols\n");

    println("Manipulators:");
    println("  • 3D pick-and-place");
    println("  • Welding robots");
    println("  • Surgical robots");
    println("  • Assembly lines\n");

    println("========================================\n");

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
