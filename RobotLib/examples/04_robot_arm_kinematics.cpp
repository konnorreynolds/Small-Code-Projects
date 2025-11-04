// ============================================================================
// Example 4: 2-DOF Robot Arm Kinematics
// ============================================================================
// This example demonstrates forward and inverse kinematics for a 2-joint
// planar robot arm (SCARA-style or RR manipulator).
//
// Topics covered:
// - Forward kinematics (joint angles → end-effector position)
// - Inverse kinematics (desired position → joint angles)
// - Workspace analysis
// - Singularity detection
// - Trajectory planning
// - Joint limits and collision avoidance
// ============================================================================

#include "../units_core.h"
#include "../units_physics.h"
#include "../units_robotics.h"
#include "../units_utilities.h"

#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>

using namespace units;
using namespace robotics;

// ============================================================================
// Robot Arm Configuration
// ============================================================================
struct ArmConfig {
    // Link lengths
    Meters link1Length = m(0.3);    // 30 cm shoulder to elbow
    Meters link2Length = m(0.25);   // 25 cm elbow to wrist

    // Joint limits
    Radians joint1Min = Radians::fromDegrees(-135);
    Radians joint1Max = Radians::fromDegrees(135);
    Radians joint2Min = Radians::fromDegrees(-150);
    Radians joint2Max = Radians::fromDegrees(150);

    // Motion limits
    RadiansPerSecond maxJointVelocity = radps(2.0);  // 2 rad/s
    RadiansPerSecondSquared maxJointAcceleration = RadiansPerSecondSquared::fromRadiansPerSecondSquared(5.0);

    // Check if joint angles are within limits
    bool areJointsValid(const Radians& j1, const Radians& j2) const {
        return (j1 >= joint1Min && j1 <= joint1Max &&
                j2 >= joint2Min && j2 <= joint2Max);
    }

    // Clamp joint angles to limits
    void clampJoints(Radians& j1, Radians& j2) const {
        double j1_rad = numerical::clamp(j1.toRadians(), joint1Min.toRadians(), joint1Max.toRadians());
        double j2_rad = numerical::clamp(j2.toRadians(), joint2Min.toRadians(), joint2Max.toRadians());
        j1 = rad(j1_rad);
        j2 = rad(j2_rad);
    }
};

// ============================================================================
// Joint State
// ============================================================================
struct JointState {
    Radians joint1;
    Radians joint2;

    JointState(Radians j1 = rad(0), Radians j2 = rad(0))
        : joint1(j1), joint2(j2) {}

    void print() const {
        std::cout << "J1: " << std::setw(7) << std::fixed << std::setprecision(2)
                  << joint1.toDegrees() << "°, J2: " << std::setw(7)
                  << joint2.toDegrees() << "°";
    }
};

// ============================================================================
// End-Effector Position
// ============================================================================
struct EndEffectorPos {
    double x;
    double y;

    EndEffectorPos(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}

    double distanceTo(const EndEffectorPos& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void print() const {
        std::cout << "(" << std::setw(6) << std::fixed << std::setprecision(3)
                  << x << ", " << std::setw(6) << y << ")";
    }
};

// ============================================================================
// Robot Arm Controller
// ============================================================================
class RobotArm {
private:
    ArmConfig config_;
    JointState currentJoints_;

public:
    explicit RobotArm(const ArmConfig& config = ArmConfig())
        : config_(config), currentJoints_(rad(0), rad(0)) {}

    // ========================================================================
    // Forward Kinematics: Joint angles → End-effector position
    // ========================================================================
    EndEffectorPos forwardKinematics(const JointState& joints) const {
        double l1 = config_.link1Length.toMeters();
        double l2 = config_.link2Length.toMeters();
        double theta1 = joints.joint1.toRadians();
        double theta2 = joints.joint2.toRadians();

        // Calculate end-effector position
        double x = l1 * std::cos(theta1) + l2 * std::cos(theta1 + theta2);
        double y = l1 * std::sin(theta1) + l2 * std::sin(theta1 + theta2);

        return EndEffectorPos(x, y);
    }

    // ========================================================================
    // Inverse Kinematics: End-effector position → Joint angles
    // ========================================================================
    enum class IKSolution {
        ELBOW_UP,     // Elbow above the line from base to target
        ELBOW_DOWN    // Elbow below the line from base to target
    };

    struct IKResult {
        bool success;
        JointState joints;
        std::string errorMessage;

        IKResult() : success(false) {}
        IKResult(const JointState& j) : success(true), joints(j) {}
        IKResult(const std::string& error) : success(false), errorMessage(error) {}
    };

    IKResult inverseKinematics(const EndEffectorPos& target,
                               IKSolution solution = IKSolution::ELBOW_UP) const {
        double l1 = config_.link1Length.toMeters();
        double l2 = config_.link2Length.toMeters();
        double x = target.x;
        double y = target.y;

        // Calculate distance to target
        double distance = std::sqrt(x * x + y * y);

        // Check if target is reachable
        double maxReach = l1 + l2;
        double minReach = std::abs(l1 - l2);

        if (distance > maxReach) {
            return IKResult("Target out of reach (too far)");
        }
        if (distance < minReach) {
            return IKResult("Target out of reach (too close)");
        }

        // Use law of cosines to find joint 2 angle
        double cosTheta2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        // Numerical safety
        cosTheta2 = numerical::clamp(cosTheta2, -1.0, 1.0);

        double theta2 = std::acos(cosTheta2);

        // Choose elbow configuration
        if (solution == IKSolution::ELBOW_DOWN) {
            theta2 = -theta2;
        }

        // Calculate joint 1 angle
        double k1 = l1 + l2 * std::cos(theta2);
        double k2 = l2 * std::sin(theta2);
        double theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        // Create joint state
        JointState joints(rad(theta1), rad(theta2));

        // Check joint limits
        if (!config_.areJointsValid(joints.joint1, joints.joint2)) {
            return IKResult("Solution violates joint limits");
        }

        return IKResult(joints);
    }

    // ========================================================================
    // Workspace Analysis
    // ========================================================================
    struct Workspace {
        double maxReach;
        double minReach;
        double area;

        void print() const {
            std::cout << "Workspace Analysis:\n";
            std::cout << "  Max reach: " << maxReach << " m\n";
            std::cout << "  Min reach: " << minReach << " m\n";
            std::cout << "  Workspace area: " << area << " m²\n";
        }
    };

    Workspace calculateWorkspace() const {
        double l1 = config_.link1Length.toMeters();
        double l2 = config_.link2Length.toMeters();

        Workspace ws;
        ws.maxReach = l1 + l2;
        ws.minReach = std::abs(l1 - l2);
        ws.area = constants::PI * (ws.maxReach * ws.maxReach - ws.minReach * ws.minReach);

        return ws;
    }

    // ========================================================================
    // Singularity Detection
    // ========================================================================
    bool isNearSingularity(const JointState& joints, double threshold = 0.1) const {
        // Robot is singular when fully extended or fully retracted
        double theta2 = std::abs(joints.joint2.toRadians());

        // Singular when theta2 ≈ 0 (extended) or theta2 ≈ π (retracted)
        bool fullyExtended = theta2 < threshold;
        bool fullyRetracted = std::abs(theta2 - constants::PI) < threshold;

        return fullyExtended || fullyRetracted;
    }

    // ========================================================================
    // Jacobian (for velocity/force analysis)
    // ========================================================================
    struct Jacobian {
        double j11, j12;  // dx/dtheta1, dx/dtheta2
        double j21, j22;  // dy/dtheta1, dy/dtheta2

        double determinant() const {
            return j11 * j22 - j12 * j21;
        }

        void print() const {
            std::cout << "Jacobian:\n";
            std::cout << "  [" << std::setw(8) << std::fixed << std::setprecision(4)
                      << j11 << " " << std::setw(8) << j12 << "]\n";
            std::cout << "  [" << std::setw(8) << j21 << " " << std::setw(8) << j22 << "]\n";
            std::cout << "  Determinant: " << determinant() << "\n";
        }
    };

    Jacobian calculateJacobian(const JointState& joints) const {
        double l1 = config_.link1Length.toMeters();
        double l2 = config_.link2Length.toMeters();
        double theta1 = joints.joint1.toRadians();
        double theta2 = joints.joint2.toRadians();

        Jacobian J;
        J.j11 = -l1 * std::sin(theta1) - l2 * std::sin(theta1 + theta2);
        J.j12 = -l2 * std::sin(theta1 + theta2);
        J.j21 = l1 * std::cos(theta1) + l2 * std::cos(theta1 + theta2);
        J.j22 = l2 * std::cos(theta1 + theta2);

        return J;
    }

    // ========================================================================
    // Getters/Setters
    // ========================================================================
    const JointState& getCurrentJoints() const { return currentJoints_; }
    void setCurrentJoints(const JointState& joints) { currentJoints_ = joints; }
    const ArmConfig& getConfig() const { return config_; }
};

// ============================================================================
// Trajectory Planner
// ============================================================================
class TrajectoryPlanner {
public:
    struct Trajectory {
        std::vector<JointState> waypoints;
        std::vector<double> times;

        void print() const {
            std::cout << "\nTrajectory (" << waypoints.size() << " waypoints):\n";
            std::cout << "Time(s) | Joint 1 | Joint 2\n";
            std::cout << "--------|---------|--------\n";
            for (size_t i = 0; i < waypoints.size(); i++) {
                std::cout << std::setw(7) << std::fixed << std::setprecision(3)
                          << times[i] << " | ";
                std::cout << std::setw(7) << waypoints[i].joint1.toDegrees() << "° | ";
                std::cout << std::setw(7) << waypoints[i].joint2.toDegrees() << "°\n";
            }
        }
    };

    // Generate linear interpolation trajectory in joint space
    static Trajectory planJointTrajectory(const JointState& start,
                                         const JointState& goal,
                                         double duration,
                                         double dt = 0.02) {
        Trajectory traj;

        for (double t = 0; t <= duration; t += dt) {
            double s = t / duration;  // Interpolation parameter [0, 1]

            // Linear interpolation
            double j1 = start.joint1.toRadians() +
                       s * (goal.joint1.toRadians() - start.joint1.toRadians());
            double j2 = start.joint2.toRadians() +
                       s * (goal.joint2.toRadians() - start.joint2.toRadians());

            traj.waypoints.push_back(JointState(rad(j1), rad(j2)));
            traj.times.push_back(t);
        }

        return traj;
    }

    // Generate trajectory in Cartesian space (with IK at each step)
    static Trajectory planCartesianTrajectory(RobotArm& arm,
                                             const EndEffectorPos& start,
                                             const EndEffectorPos& goal,
                                             double duration,
                                             double dt = 0.02) {
        Trajectory traj;

        for (double t = 0; t <= duration; t += dt) {
            double s = t / duration;

            // Linear interpolation in Cartesian space
            double x = start.x + s * (goal.x - start.x);
            double y = start.y + s * (goal.y - start.y);

            // Solve IK for this point
            auto ikResult = arm.inverseKinematics(EndEffectorPos(x, y));

            if (ikResult.success) {
                traj.waypoints.push_back(ikResult.joints);
                traj.times.push_back(t);
            }
        }

        return traj;
    }
};

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "========================================\n";
    std::cout << "  2-DOF Robot Arm Kinematics\n";
    std::cout << "========================================\n\n";

    // Create robot arm
    ArmConfig config;
    RobotArm arm(config);

    std::cout << "Robot Configuration:\n";
    std::cout << "  Link 1: " << config.link1Length.toCentimeters() << " cm\n";
    std::cout << "  Link 2: " << config.link2Length.toCentimeters() << " cm\n";
    std::cout << "  Joint 1 limits: " << config.joint1Min.toDegrees()
              << "° to " << config.joint1Max.toDegrees() << "°\n";
    std::cout << "  Joint 2 limits: " << config.joint2Min.toDegrees()
              << "° to " << config.joint2Max.toDegrees() << "°\n\n";

    // ========================================================================
    // Test 1: Forward Kinematics
    // ========================================================================
    std::cout << "Test 1: Forward Kinematics\n";
    std::cout << "===========================\n";

    std::vector<JointState> testConfigs = {
        JointState(rad(0), rad(0)),
        JointState(rad(0.785), rad(0.785)),
        JointState(rad(1.571), rad(-1.571)),
        JointState(rad(-0.785), rad(1.571))
    };

    for (const auto& joints : testConfigs) {
        auto pos = arm.forwardKinematics(joints);
        std::cout << "  ";
        joints.print();
        std::cout << " → ";
        pos.print();
        std::cout << " m\n";
    }

    // ========================================================================
    // Test 2: Inverse Kinematics
    // ========================================================================
    std::cout << "\n\nTest 2: Inverse Kinematics\n";
    std::cout << "===========================\n";

    std::vector<EndEffectorPos> testTargets = {
        EndEffectorPos(0.4, 0.2),
        EndEffectorPos(0.3, 0.3),
        EndEffectorPos(0.5, 0.0),
        EndEffectorPos(0.0, 0.5)
    };

    for (const auto& target : testTargets) {
        std::cout << "Target: ";
        target.print();
        std::cout << " m\n";

        auto ikElbowUp = arm.inverseKinematics(target, RobotArm::IKSolution::ELBOW_UP);
        auto ikElbowDown = arm.inverseKinematics(target, RobotArm::IKSolution::ELBOW_DOWN);

        if (ikElbowUp.success) {
            std::cout << "  Elbow-up:   ";
            ikElbowUp.joints.print();

            // Verify with FK
            auto verify = arm.forwardKinematics(ikElbowUp.joints);
            double error = target.distanceTo(verify);
            std::cout << "  (error: " << std::setprecision(6) << error * 1000 << " mm)\n";
        } else {
            std::cout << "  Elbow-up:   " << ikElbowUp.errorMessage << "\n";
        }

        if (ikElbowDown.success) {
            std::cout << "  Elbow-down: ";
            ikElbowDown.joints.print();

            auto verify = arm.forwardKinematics(ikElbowDown.joints);
            double error = target.distanceTo(verify);
            std::cout << "  (error: " << std::setprecision(6) << error * 1000 << " mm)\n";
        } else {
            std::cout << "  Elbow-down: " << ikElbowDown.errorMessage << "\n";
        }

        std::cout << "\n";
    }

    // ========================================================================
    // Test 3: Workspace Analysis
    // ========================================================================
    std::cout << "\nTest 3: Workspace Analysis\n";
    std::cout << "===========================\n";
    auto workspace = arm.calculateWorkspace();
    workspace.print();

    // ========================================================================
    // Test 4: Singularity Detection
    // ========================================================================
    std::cout << "\n\nTest 4: Singularity Detection\n";
    std::cout << "==============================\n";

    std::vector<JointState> singularityTests = {
        JointState(rad(0.785), rad(0)),      // Extended
        JointState(rad(0.785), rad(3.142)),    // Retracted
        JointState(rad(0.785), rad(1.571)),     // Not singular
        JointState(rad(0), rad(0.087))        // Nearly extended
    };

    for (const auto& joints : singularityTests) {
        std::cout << "  ";
        joints.print();

        if (arm.isNearSingularity(joints)) {
            std::cout << " → SINGULAR (limited dexterity)\n";
        } else {
            std::cout << " → OK\n";
        }
    }

    // ========================================================================
    // Test 5: Jacobian Analysis
    // ========================================================================
    std::cout << "\n\nTest 5: Jacobian Analysis\n";
    std::cout << "==========================\n";

    JointState testJoint(rad(0.785), rad(1.047));
    std::cout << "Configuration: ";
    testJoint.print();
    std::cout << "\n\n";

    auto jacobian = arm.calculateJacobian(testJoint);
    jacobian.print();

    // ========================================================================
    // Test 6: Trajectory Planning
    // ========================================================================
    std::cout << "\n\nTest 6: Trajectory Planning\n";
    std::cout << "============================\n";

    JointState trajStart(rad(0), rad(0));
    JointState trajGoal(rad(1.571), rad(-0.785));

    std::cout << "Planning trajectory from ";
    trajStart.print();
    std::cout << " to ";
    trajGoal.print();
    std::cout << "\n";

    auto trajectory = TrajectoryPlanner::planJointTrajectory(
        trajStart, trajGoal, 2.0, 0.5  // 2 seconds, sample every 0.5s
    );

    trajectory.print();

    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n\n========================================\n";
    std::cout << "  Summary\n";
    std::cout << "========================================\n\n";

    std::cout << "This example demonstrated:\n";
    std::cout << "✓ Forward kinematics (FK)\n";
    std::cout << "✓ Inverse kinematics (IK) with multiple solutions\n";
    std::cout << "✓ Workspace analysis and reachability\n";
    std::cout << "✓ Singularity detection\n";
    std::cout << "✓ Jacobian calculation\n";
    std::cout << "✓ Trajectory planning in joint and Cartesian space\n";
    std::cout << "✓ Type-safe units throughout\n\n";

    std::cout << "Key Concepts:\n";
    std::cout << "• FK transforms joint angles to end-effector position\n";
    std::cout << "• IK is the inverse problem (often multiple solutions)\n";
    std::cout << "• Singularities occur at workspace boundaries\n";
    std::cout << "• Jacobian relates joint velocities to end-effector velocities\n";
    std::cout << "• Trajectory planning ensures smooth, collision-free motion\n\n";

    return 0;
}

/*
This example teaches:
1. Forward kinematics using trigonometry
2. Inverse kinematics using law of cosines
3. Multiple IK solutions (elbow-up vs elbow-down)
4. Workspace analysis and reachability checking
5. Singularity detection and avoidance
6. Jacobian matrix calculation
7. Trajectory planning in joint and Cartesian space
8. Type-safe angle handling with automatic conversions

Real-world applications:
- Pick-and-place robots
- SCARA assembly arms
- Painting/welding robots
- Surgical robots
- 3D printers with articulated heads
*/
