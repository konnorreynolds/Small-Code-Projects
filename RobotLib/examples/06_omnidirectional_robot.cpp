// ============================================================================
// Example 6: Omnidirectional Robot with Mecanum Wheels
// ============================================================================
// This example demonstrates how to control an omnidirectional robot using
// mecanum wheels, which can move in any direction without rotating.
//
// Topics covered:
// - Mecanum wheel kinematics (forward and inverse)
// - Holonomic motion control (independent translation and rotation)
// - Velocity vector decomposition
// - Wheel slip compensation
// - Field-centric vs robot-centric control
// - Odometry for omnidirectional robots
// ============================================================================

#include "../include/units_core.h"
#include "../include/units_physics.h"
#include "../include/units_robotics.h"
#include "../include/units_utilities.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>

using namespace units;
using namespace robotics;

// ============================================================================
// Mecanum Wheel Configuration
// ============================================================================
struct MecanumConfig {
    // Robot dimensions
    Meters wheelRadius = m(0.05);          // 5 cm wheel radius
    Meters trackWidth = m(0.30);           // 30 cm between left/right wheels
    Meters wheelBase = m(0.30);            // 30 cm between front/back wheels

    // Motion limits
    MetersPerSecond maxLinearSpeed = mps(1.0);
    RadiansPerSecond maxAngularSpeed = radps(2.0);

    // Wheel slip factor (1.0 = no slip, <1.0 = slippage compensation)
    double wheelSlipFactor = 0.95;
};

// ============================================================================
// Mecanum Wheel Velocities
// ============================================================================
struct MecanumWheelVelocities {
    MetersPerSecond frontLeft;
    MetersPerSecond frontRight;
    MetersPerSecond backLeft;
    MetersPerSecond backRight;

    // Check if any wheel exceeds maximum speed
    MetersPerSecond maxWheelSpeed() const {
        double max_mps = numerical::max(
            numerical::max(std::abs(frontLeft.toMetersPerSecond()),
                          std::abs(frontRight.toMetersPerSecond())),
            numerical::max(std::abs(backLeft.toMetersPerSecond()),
                          std::abs(backRight.toMetersPerSecond()))
        );
        return mps(max_mps);
    }

    // Normalize all wheel speeds if any exceed limit
    void normalize(MetersPerSecond maxSpeed) {
        auto currentMax = maxWheelSpeed();
        if (currentMax.toMetersPerSecond() > maxSpeed.toMetersPerSecond()) {
            double scale = maxSpeed.toMetersPerSecond() / currentMax.toMetersPerSecond();
            frontLeft = mps(frontLeft.toMetersPerSecond() * scale);
            frontRight = mps(frontRight.toMetersPerSecond() * scale);
            backLeft = mps(backLeft.toMetersPerSecond() * scale);
            backRight = mps(backRight.toMetersPerSecond() * scale);
        }
    }
};

// ============================================================================
// Mecanum Drive Kinematics
// ============================================================================
class MecanumDrive {
private:
    MecanumConfig config_;

public:
    explicit MecanumDrive(const MecanumConfig& config = MecanumConfig())
        : config_(config) {}

    // Inverse Kinematics: Robot velocity → Wheel velocities
    // Given desired robot motion, calculate required wheel speeds
    MecanumWheelVelocities inverseKinematics(
        MetersPerSecond vx,        // Forward velocity (robot frame)
        MetersPerSecond vy,        // Strafe velocity (robot frame)
        RadiansPerSecond omega     // Angular velocity
    ) const {
        // Robot geometry factor for rotation
        double L = config_.trackWidth.toMeters();    // Half track width
        double W = config_.wheelBase.toMeters();     // Half wheel base
        double R = config_.wheelRadius.toMeters();

        // Mecanum wheel kinematics equations
        // Each wheel's velocity is a combination of:
        // - Forward motion (vx)
        // - Sideways motion (vy) - with opposite signs for opposite diagonal wheels
        // - Rotational motion (omega)

        double vx_val = vx.toMetersPerSecond();
        double vy_val = vy.toMetersPerSecond();
        double omega_val = omega.toRadiansPerSecond();

        // Apply wheel slip compensation
        vy_val *= config_.wheelSlipFactor;

        // Mecanum wheel equations (roller at 45°)
        double fl = (vx_val + vy_val + (L + W) * omega_val) / R;
        double fr = (vx_val - vy_val - (L + W) * omega_val) / R;
        double bl = (vx_val - vy_val + (L + W) * omega_val) / R;
        double br = (vx_val + vy_val - (L + W) * omega_val) / R;

        MecanumWheelVelocities velocities;
        velocities.frontLeft = mps(fl);
        velocities.frontRight = mps(fr);
        velocities.backLeft = mps(bl);
        velocities.backRight = mps(br);

        return velocities;
    }

    // Forward Kinematics: Wheel velocities → Robot velocity
    // Given measured wheel speeds, calculate actual robot motion
    struct RobotVelocity {
        MetersPerSecond vx;      // Forward velocity
        MetersPerSecond vy;      // Strafe velocity
        RadiansPerSecond omega;  // Angular velocity
    };

    RobotVelocity forwardKinematics(const MecanumWheelVelocities& wheels) const {
        double L = config_.trackWidth.toMeters();
        double W = config_.wheelBase.toMeters();
        double R = config_.wheelRadius.toMeters();

        double fl = wheels.frontLeft.toMetersPerSecond();
        double fr = wheels.frontRight.toMetersPerSecond();
        double bl = wheels.backLeft.toMetersPerSecond();
        double br = wheels.backRight.toMetersPerSecond();

        // Inverse of the kinematic equations
        double vx = R * (fl + fr + bl + br) / 4.0;
        double vy = R * (fl - fr - bl + br) / 4.0;
        double omega = R * (fl - fr + bl - br) / (4.0 * (L + W));

        return {mps(vx), mps(vy), radps(omega)};
    }

    // Field-Centric Control: Convert field velocities to robot velocities
    // Useful when driver wants to control robot relative to field, not robot
    static RobotVelocity fieldToRobot(
        MetersPerSecond vx_field,
        MetersPerSecond vy_field,
        RadiansPerSecond omega,
        Radians robotHeading
    ) {
        double cos_h = std::cos(robotHeading.toRadians());
        double sin_h = std::sin(robotHeading.toRadians());

        double vx_f = vx_field.toMetersPerSecond();
        double vy_f = vy_field.toMetersPerSecond();

        // Rotate velocity vector by robot heading
        double vx_robot = vx_f * cos_h + vy_f * sin_h;
        double vy_robot = -vx_f * sin_h + vy_f * cos_h;

        return {mps(vx_robot), mps(vy_robot), omega};
    }
};

// ============================================================================
// Omnidirectional Odometry
// ============================================================================
class OmniOdometry {
private:
    Meters x_;
    Meters y_;
    Radians theta_;

    MecanumWheelVelocities lastWheelVelocities_;
    MecanumDrive kinematics_;

public:
    explicit OmniOdometry(const MecanumConfig& config = MecanumConfig())
        : x_(m(0)), y_(m(0)), theta_(rad(0)), kinematics_(config) {
        lastWheelVelocities_.frontLeft = mps(0);
        lastWheelVelocities_.frontRight = mps(0);
        lastWheelVelocities_.backLeft = mps(0);
        lastWheelVelocities_.backRight = mps(0);
    }

    void update(const MecanumWheelVelocities& wheelVelocities, double dt) {
        // Calculate robot velocity from wheel velocities
        auto robotVel = kinematics_.forwardKinematics(wheelVelocities);

        // Update position using current heading
        double cos_theta = std::cos(theta_.toRadians());
        double sin_theta = std::sin(theta_.toRadians());

        double dx_robot = robotVel.vx.toMetersPerSecond() * dt;
        double dy_robot = robotVel.vy.toMetersPerSecond() * dt;

        // Transform to field frame
        double dx_field = dx_robot * cos_theta - dy_robot * sin_theta;
        double dy_field = dx_robot * sin_theta + dy_robot * cos_theta;

        x_ = m(x_.toMeters() + dx_field);
        y_ = m(y_.toMeters() + dy_field);
        theta_ = rad(theta_.toRadians() + robotVel.omega.toRadiansPerSecond() * dt);

        // Normalize angle to [-π, π]
        while (theta_.toRadians() > constants::PI) {
            theta_ = rad(theta_.toRadians() - constants::TWO_PI);
        }
        while (theta_.toRadians() < -constants::PI) {
            theta_ = rad(theta_.toRadians() + constants::TWO_PI);
        }

        lastWheelVelocities_ = wheelVelocities;
    }

    Meters getX() const { return x_; }
    Meters getY() const { return y_; }
    Radians getTheta() const { return theta_; }

    void reset() {
        x_ = m(0);
        y_ = m(0);
        theta_ = rad(0);
    }

    void setPose(Meters x, Meters y, Radians theta) {
        x_ = x;
        y_ = y;
        theta_ = theta;
    }
};

// ============================================================================
// Demonstration Scenarios
// ============================================================================
void printWheelVelocities(const MecanumWheelVelocities& wheels) {
    std::cout << "FL:" << std::setw(7) << std::fixed << std::setprecision(3)
              << wheels.frontLeft.toMetersPerSecond() << " m/s  ";
    std::cout << "FR:" << std::setw(7)
              << wheels.frontRight.toMetersPerSecond() << " m/s  ";
    std::cout << "BL:" << std::setw(7)
              << wheels.backLeft.toMetersPerSecond() << " m/s  ";
    std::cout << "BR:" << std::setw(7)
              << wheels.backRight.toMetersPerSecond() << " m/s";
}

void demonstrateBasicMotions() {
    std::cout << "========================================\n";
    std::cout << "  Basic Omnidirectional Motions\n";
    std::cout << "========================================\n\n";

    MecanumDrive drive;

    // 1. Forward motion
    std::cout << "1. Forward Motion (vx = 0.5 m/s):\n";
    auto forward = drive.inverseKinematics(mps(0.5), mps(0), radps(0));
    std::cout << "   ";
    printWheelVelocities(forward);
    std::cout << "\n   → All wheels move at same speed forward\n\n";

    // 2. Strafe right
    std::cout << "2. Strafe Right (vy = 0.5 m/s):\n";
    auto strafe = drive.inverseKinematics(mps(0), mps(0.5), radps(0));
    std::cout << "   ";
    printWheelVelocities(strafe);
    std::cout << "\n   → FL/BR forward, FR/BL backward (diagonal motion)\n\n";

    // 3. Rotate in place
    std::cout << "3. Rotate in Place (ω = 1.0 rad/s):\n";
    auto rotate = drive.inverseKinematics(mps(0), mps(0), radps(1.0));
    std::cout << "   ";
    printWheelVelocities(rotate);
    std::cout << "\n   → Left wheels forward, right wheels backward\n\n";

    // 4. Diagonal motion (45°)
    std::cout << "4. Diagonal Motion (vx = 0.5, vy = 0.5):\n";
    auto diagonal = drive.inverseKinematics(mps(0.5), mps(0.5), radps(0));
    std::cout << "   ";
    printWheelVelocities(diagonal);
    std::cout << "\n   → FL/BR move, FR/BL stationary\n\n";

    // 5. Complex motion (forward + strafe + rotate)
    std::cout << "5. Complex Motion (vx=0.3, vy=0.2, ω=0.5):\n";
    auto complex = drive.inverseKinematics(mps(0.3), mps(0.2), radps(0.5));
    std::cout << "   ";
    printWheelVelocities(complex);
    std::cout << "\n   → All wheels at different speeds (holonomic control)\n\n";
}

void demonstrateVelocityNormalization() {
    std::cout << "========================================\n";
    std::cout << "  Velocity Normalization\n";
    std::cout << "========================================\n\n";

    MecanumConfig config;
    config.maxLinearSpeed = mps(1.0);
    MecanumDrive drive(config);

    std::cout << "Maximum wheel speed: " << config.maxLinearSpeed.toMetersPerSecond() << " m/s\n\n";

    // Request very high velocities
    std::cout << "Requested: vx=1.5, vy=1.5, ω=2.0 (exceeds limits)\n";
    auto wheels = drive.inverseKinematics(mps(1.5), mps(1.5), radps(2.0));

    std::cout << "Before normalization:\n   ";
    printWheelVelocities(wheels);
    std::cout << "\n   Max: " << wheels.maxWheelSpeed().toMetersPerSecond() << " m/s (EXCEEDS LIMIT!)\n\n";

    wheels.normalize(config.maxLinearSpeed);
    std::cout << "After normalization:\n   ";
    printWheelVelocities(wheels);
    std::cout << "\n   Max: " << wheels.maxWheelSpeed().toMetersPerSecond() << " m/s ✓\n";
    std::cout << "   → Motion direction preserved, speed scaled down\n\n";
}

void demonstrateFieldCentricControl() {
    std::cout << "========================================\n";
    std::cout << "  Field-Centric vs Robot-Centric\n";
    std::cout << "========================================\n\n";

    MecanumDrive drive;

    // Robot is facing 90° (pointing left in field frame)
    Radians robotHeading = rad(constants::PI / 2.0);  // 90 degrees

    std::cout << "Robot heading: " << robotHeading.toDegrees() << "° (pointing left)\n\n";

    // Driver wants to move forward in field frame (positive X)
    MetersPerSecond vx_field = mps(0.5);
    MetersPerSecond vy_field = mps(0.0);

    std::cout << "Field-centric command: Forward (field +X)\n";
    std::cout << "Field velocities: vx=" << vx_field.toMetersPerSecond()
              << ", vy=" << vy_field.toMetersPerSecond() << "\n\n";

    // Convert to robot frame
    auto robotVel = MecanumDrive::fieldToRobot(
        vx_field, vy_field, radps(0), robotHeading
    );

    std::cout << "Converted to robot frame:\n";
    std::cout << "Robot velocities: vx=" << robotVel.vx.toMetersPerSecond()
              << ", vy=" << robotVel.vy.toMetersPerSecond() << "\n";
    std::cout << "→ Robot must strafe right to move forward in field\n\n";

    // Calculate wheel velocities
    auto wheels = drive.inverseKinematics(robotVel.vx, robotVel.vy, robotVel.omega);
    std::cout << "Wheel velocities: ";
    printWheelVelocities(wheels);
    std::cout << "\n\n";

    std::cout << "This allows the driver to control the robot relative to\n";
    std::cout << "the field, regardless of robot orientation!\n\n";
}

void demonstrateOdometry() {
    std::cout << "========================================\n";
    std::cout << "  Omnidirectional Odometry\n";
    std::cout << "========================================\n\n";

    MecanumDrive drive;
    OmniOdometry odometry;

    double dt = 0.05;  // 50ms update rate

    std::cout << "Scenario: Square path (1m × 1m) using holonomic motion\n\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s) | Command           | X(m)  | Y(m)  | θ(°)  |\n";
    std::cout << "--------|-------------------|-------|-------|-------|\n";

    // Square path using omnidirectional motion
    struct PathSegment {
        double duration;
        MetersPerSecond vx;
        MetersPerSecond vy;
        RadiansPerSecond omega;
        const char* description;
    };

    PathSegment path[] = {
        {1.0, mps(1.0), mps(0.0), radps(0.0), "Forward 1m"},
        {1.0, mps(0.0), mps(1.0), radps(0.0), "Strafe left 1m"},
        {1.0, mps(-1.0), mps(0.0), radps(0.0), "Backward 1m"},
        {1.0, mps(0.0), mps(-1.0), radps(0.0), "Strafe right 1m"}
    };

    double totalTime = 0.0;
    int segmentIndex = 0;
    double segmentTime = 0.0;

    while (segmentIndex < 4) {
        auto& segment = path[segmentIndex];

        // Calculate wheel velocities
        auto wheels = drive.inverseKinematics(segment.vx, segment.vy, segment.omega);

        // Update odometry
        odometry.update(wheels, dt);

        totalTime += dt;
        segmentTime += dt;

        // Print status every 0.2 seconds
        if (static_cast<int>(totalTime / 0.2) != static_cast<int>((totalTime - dt) / 0.2)) {
            std::cout << std::setw(6) << totalTime << "  | ";
            std::cout << std::setw(17) << std::left << segment.description << std::right << " | ";
            std::cout << std::setw(5) << odometry.getX().toMeters() << " | ";
            std::cout << std::setw(5) << odometry.getY().toMeters() << " | ";
            std::cout << std::setw(5) << odometry.getTheta().toDegrees() << " |\n";
        }

        // Move to next segment
        if (segmentTime >= segment.duration) {
            segmentIndex++;
            segmentTime = 0.0;
        }
    }

    std::cout << "\nFinal position: ("
              << odometry.getX().toMeters() << ", "
              << odometry.getY().toMeters() << ") m\n";
    std::cout << "Expected: (0, 0) - Close to start position ✓\n";
    std::cout << "Heading: " << odometry.getTheta().toDegrees() << "° (unchanged)\n\n";

    std::cout << "Key Advantage: Robot maintained heading throughout path!\n";
    std::cout << "With differential drive, this would require 4 turns.\n\n";
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Omnidirectional Robot Kinematics     ║\n";
    std::cout << "║  Mecanum Wheel Drive System            ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrateBasicMotions();
    demonstrateVelocityNormalization();
    demonstrateFieldCentricControl();
    demonstrateOdometry();

    std::cout << "========================================\n";
    std::cout << "  Key Concepts Summary\n";
    std::cout << "========================================\n\n";

    std::cout << "1. Holonomic Motion:\n";
    std::cout << "   • Independent control of X, Y, and rotation\n";
    std::cout << "   • Can move in any direction without turning\n";
    std::cout << "   • 3 degrees of freedom (vs 2 for differential)\n\n";

    std::cout << "2. Mecanum Kinematics:\n";
    std::cout << "   • Inverse: Desired motion → Wheel speeds\n";
    std::cout << "   • Forward: Measured wheels → Actual motion\n";
    std::cout << "   • 45° rollers enable sideways force\n\n";

    std::cout << "3. Field-Centric Control:\n";
    std::cout << "   • Driver controls relative to field, not robot\n";
    std::cout << "   • Requires IMU/gyro for heading\n";
    std::cout << "   • Much easier for human operators\n\n";

    std::cout << "4. Velocity Normalization:\n";
    std::cout << "   • Prevents wheel saturation\n";
    std::cout << "   • Preserves motion direction\n";
    std::cout << "   • Critical for safe operation\n\n";

    std::cout << "5. Applications:\n";
    std::cout << "   • Warehouse robots (precise maneuvering)\n";
    std::cout << "   • Competition robots (FRC, VEX)\n";
    std::cout << "   • Industrial AGVs\n";
    std::cout << "   • Mobile manipulation platforms\n\n";

    std::cout << "Advantages over Differential Drive:\n";
    std::cout << "+ Can strafe sideways\n";
    std::cout << "+ No need to turn before moving\n";
    std::cout << "+ Better maneuverability in tight spaces\n";
    std::cout << "+ Can maintain heading while translating\n\n";

    std::cout << "Disadvantages:\n";
    std::cout << "- More complex kinematics\n";
    std::cout << "- Mecanum wheels are expensive\n";
    std::cout << "- Lower traction than normal wheels\n";
    std::cout << "- More sensitive to wheel slip\n\n";

    std::cout << "========================================\n\n";

    return 0;
}

/*
Expected Output:
- Demonstration of 5 basic motion types
- Wheel velocity calculations for each motion
- Velocity normalization example
- Field-centric control conversion
- Odometry tracking for square path

This example demonstrates:
1. Complete mecanum wheel kinematics (inverse and forward)
2. Holonomic motion control (3 DOF)
3. Velocity normalization for saturation prevention
4. Field-centric control for intuitive operation
5. Omnidirectional odometry for position tracking
6. Type-safe units throughout all calculations
7. Real-world considerations (wheel slip, limits)

Educational value:
- Shows advantage of holonomic drive over differential
- Explains mecanum wheel principle (45° rollers)
- Demonstrates field vs robot frame transformations
- Provides practical velocity limiting strategies
- Illustrates odometry for omnidirectional motion
*/
