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

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_utilities.h"

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
    print("FL:" , wheels.frontLeft.toMetersPerSecond() , " m/s  ");
    print("FR:" , wheels.frontRight.toMetersPerSecond() , " m/s  ");
    print("BL:" , wheels.backLeft.toMetersPerSecond() , " m/s  ");
    print("BR:" , wheels.backRight.toMetersPerSecond() , " m/s");
}

void demonstrateBasicMotions() {
    println("========================================");
    println("  Basic Omnidirectional Motions");
    println("========================================\n");

    MecanumDrive drive;

    // 1. Forward motion
    println("1. Forward Motion (vx = 0.5 m/s):");
    auto forward = drive.inverseKinematics(mps(0.5), mps(0), radps(0));
    print("   ");
    printWheelVelocities(forward);
    println("\n   → All wheels move at same speed forward\n");

    // 2. Strafe right
    println("2. Strafe Right (vy = 0.5 m/s):");
    auto strafe = drive.inverseKinematics(mps(0), mps(0.5), radps(0));
    print("   ");
    printWheelVelocities(strafe);
    println("\n   → FL/BR forward, FR/BL backward (diagonal motion)\n");

    // 3. Rotate in place
    println("3. Rotate in Place (ω = 1.0 rad/s):");
    auto rotate = drive.inverseKinematics(mps(0), mps(0), radps(1.0));
    print("   ");
    printWheelVelocities(rotate);
    println("\n   → Left wheels forward, right wheels backward\n");

    // 4. Diagonal motion (45°)
    println("4. Diagonal Motion (vx = 0.5, vy = 0.5):");
    auto diagonal = drive.inverseKinematics(mps(0.5), mps(0.5), radps(0));
    print("   ");
    printWheelVelocities(diagonal);
    println("\n   → FL/BR move, FR/BL stationary\n");

    // 5. Complex motion (forward + strafe + rotate)
    println("5. Complex Motion (vx=0.3, vy=0.2, ω=0.5):");
    auto complex = drive.inverseKinematics(mps(0.3), mps(0.2), radps(0.5));
    print("   ");
    printWheelVelocities(complex);
    println("\n   → All wheels at different speeds (holonomic control)\n");
}

void demonstrateVelocityNormalization() {
    println("========================================");
    println("  Velocity Normalization");
    println("========================================\n");

    MecanumConfig config;
    config.maxLinearSpeed = mps(1.0);
    MecanumDrive drive(config);

    println("Maximum wheel speed: ", config.maxLinearSpeed.toMetersPerSecond(), " m/s\n");

    // Request very high velocities
    println("Requested: vx=1.5, vy=1.5, ω=2.0 (exceeds limits)");
    auto wheels = drive.inverseKinematics(mps(1.5), mps(1.5), radps(2.0));

    println("Before normalization:\n   ");
    printWheelVelocities(wheels);
    println("\n   Max: ", wheels.maxWheelSpeed().toMetersPerSecond(), " m/s (EXCEEDS LIMIT!)\n");

    wheels.normalize(config.maxLinearSpeed);
    println("After normalization:\n   ");
    printWheelVelocities(wheels);
    println("\n   Max: ", wheels.maxWheelSpeed().toMetersPerSecond(), " m/s ✓");
    println("   → Motion direction preserved, speed scaled down\n");
}

void demonstrateFieldCentricControl() {
    println("========================================");
    println("  Field-Centric vs Robot-Centric");
    println("========================================\n");

    MecanumDrive drive;

    // Robot is facing 90° (pointing left in field frame)
    Radians robotHeading = rad(constants::PI / 2.0);  // 90 degrees

    println("Robot heading: ", robotHeading.toDegrees(), "° (pointing left)\n");

    // Driver wants to move forward in field frame (positive X)
    MetersPerSecond vx_field = mps(0.5);
    MetersPerSecond vy_field = mps(0.0);

    println("Field-centric command: Forward (field +X)");
    print("Field velocities: vx=" ,  vx_field.toMetersPerSecond()
              , ", vy=");

    // Convert to robot frame
    auto robotVel = MecanumDrive::fieldToRobot(
        vx_field, vy_field, radps(0), robotHeading
    );

    println("Converted to robot frame:");
    print("Robot velocities: vx=" ,  robotVel.vx.toMetersPerSecond()
              , ", vy=");
    println("→ Robot must strafe right to move forward in field\n");

    // Calculate wheel velocities
    auto wheels = drive.inverseKinematics(robotVel.vx, robotVel.vy, robotVel.omega);
    print("Wheel velocities: ");
    printWheelVelocities(wheels);
    println("\n");

    println("This allows the driver to control the robot relative to");
    println("the field, regardless of robot orientation!\n");
}

void demonstrateOdometry() {
    println("========================================");
    println("  Omnidirectional Odometry");
    println("========================================\n");

    MecanumDrive drive;
    OmniOdometry odometry;

    double dt = 0.05;  // 50ms update rate

    println("Scenario: Square path (1m × 1m) using holonomic motion\n");
    print(, );
    println("Time(s) | Command           | X(m)  | Y(m)  | θ(°)  |");
    println("--------|-------------------|-------|-------|-------|");

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
            print(, totalTime, "  | ");
            print(, segment.description, std::right, " | ");
            print(, odometry.getX().toMeters(), " | ");
            print(, odometry.getY().toMeters(), " | ");
            println(, odometry.getTheta().toDegrees(), " |");
        }

        // Move to next segment
        if (segmentTime >= segment.duration) {
            segmentIndex++;
            segmentTime = 0.0;
        }
    }

    print("\nFinal position: ("
              ,  odometry.getX().toMeters() , ", ");
    println("Expected: (0, 0) - Close to start position ✓");
    println("Heading: ", odometry.getTheta().toDegrees(), "° (unchanged)\n");

    println("Key Advantage: Robot maintained heading throughout path!");
    println("With differential drive, this would require 4 turns.\n");
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  Omnidirectional Robot Kinematics     ║");
    println("║  Mecanum Wheel Drive System            ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrateBasicMotions();
    demonstrateVelocityNormalization();
    demonstrateFieldCentricControl();
    demonstrateOdometry();

    println("========================================");
    println("  Key Concepts Summary");
    println("========================================\n");

    println("1. Holonomic Motion:");
    println("   • Independent control of X, Y, and rotation");
    println("   • Can move in any direction without turning");
    println("   • 3 degrees of freedom (vs 2 for differential)\n");

    println("2. Mecanum Kinematics:");
    println("   • Inverse: Desired motion → Wheel speeds");
    println("   • Forward: Measured wheels → Actual motion");
    println("   • 45° rollers enable sideways force\n");

    println("3. Field-Centric Control:");
    println("   • Driver controls relative to field, not robot");
    println("   • Requires IMU/gyro for heading");
    println("   • Much easier for human operators\n");

    println("4. Velocity Normalization:");
    println("   • Prevents wheel saturation");
    println("   • Preserves motion direction");
    println("   • Critical for safe operation\n");

    println("5. Applications:");
    println("   • Warehouse robots (precise maneuvering)");
    println("   • Competition robots (FRC, VEX)");
    println("   • Industrial AGVs");
    println("   • Mobile manipulation platforms\n");

    println("Advantages over Differential Drive:");
    println("+ Can strafe sideways");
    println("+ No need to turn before moving");
    println("+ Better maneuverability in tight spaces");
    println("+ Can maintain heading while translating\n");

    println("Disadvantages:");
    println("- More complex kinematics");
    println("- Mecanum wheels are expensive");
    println("- Lower traction than normal wheels");
    println("- More sensitive to wheel slip\n");

    println("========================================\n");

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
