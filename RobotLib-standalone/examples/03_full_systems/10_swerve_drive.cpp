// ============================================================================
// Example 10: Swerve Drive Kinematics
// ============================================================================
// Advanced example demonstrating swerve drive (also called holonomic or
// omnidirectional drive with steerable wheels). This is one of the most
// sophisticated drive systems in competitive robotics.
//
// Topics covered:
// - Swerve module kinematics (speed + angle)
// - Inverse kinematics (robot motion → module states)
// - Forward kinematics (module states → robot motion)
// - Field-centric control
// - Module optimization (shortest rotation path)
// - Wheel velocity limiting
// - Second-order kinematics (acceleration limits)
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_utilities.h"
#include "../../include/units_math.h"

#include <array>
#include <cmath>

using namespace units;
using namespace robotics;

// ============================================================================
// Swerve Module State
// ============================================================================
struct SwerveModuleState {
    MetersPerSecond speed;
    Radians angle;  // Wheel direction

    SwerveModuleState() : speed(mps(0)), angle(rad(0)) {}
    SwerveModuleState(MetersPerSecond s, Radians a) : speed(s), angle(a) {}

    // Optimize module state to minimize rotation
    // If we need to rotate >90°, reverse speed and rotate <90° instead
    void optimize(Radians currentAngle) {
        Radians targetAngle = angle;
        Radians delta = (targetAngle - currentAngle).normalizeSigned();

        if (std::abs(delta.toRadians()) > constants::PI / 2.0) {
            // Reverse speed and flip angle by 180°
            speed = mps(-speed.toMetersPerSecond());
            angle = rad(targetAngle.toRadians() + constants::PI);
        }
    }
};

// ============================================================================
// Swerve Drive Kinematics
// ============================================================================
class SwerveDriveKinematics {
private:
    // Module positions relative to robot center (in meters)
    std::array<Vec2D, 4> modulePositions_;

public:
    // Constructor: Define module positions
    // Standard swerve has modules at the four corners
    SwerveDriveKinematics(double trackWidth, double wheelBase) {
        double halfWidth = trackWidth / 2.0;
        double halfBase = wheelBase / 2.0;

        // Front-left, front-right, back-left, back-right
        modulePositions_[0] = Vec2D(halfBase, halfWidth);    // FL
        modulePositions_[1] = Vec2D(halfBase, -halfWidth);   // FR
        modulePositions_[2] = Vec2D(-halfBase, halfWidth);   // BL
        modulePositions_[3] = Vec2D(-halfBase, -halfWidth);  // BR
    }

    // Inverse Kinematics: Robot velocity → Module states
    std::array<SwerveModuleState, 4> toModuleStates(
        MetersPerSecond vx,        // Forward velocity
        MetersPerSecond vy,        // Sideways velocity
        RadiansPerSecond omega     // Rotation rate
    ) const {
        std::array<SwerveModuleState, 4> states;

        for (size_t i = 0; i < 4; i++) {
            const Vec2D& pos = modulePositions_[i];

            // Tangential velocity from rotation: v_tan = omega × r
            // For 2D: v_tan_x = -omega * y, v_tan_y = omega * x
            double vx_module = vx.toMetersPerSecond() - omega.toRadiansPerSecond() * pos.y;
            double vy_module = vy.toMetersPerSecond() + omega.toRadiansPerSecond() * pos.x;

            // Calculate module speed and angle
            double speed = std::sqrt(vx_module * vx_module + vy_module * vy_module);
            double angle = std::atan2(vy_module, vx_module);

            states[i] = SwerveModuleState(mps(speed), rad(angle));
        }

        return states;
    }

    // Forward Kinematics: Module states → Robot velocity
    struct RobotVelocity {
        MetersPerSecond vx;
        MetersPerSecond vy;
        RadiansPerSecond omega;
    };

    RobotVelocity toRobotVelocity(const std::array<SwerveModuleState, 4>& states) const {
        // Use pseudo-inverse of the kinematic matrix
        // This is a simplified version; production code uses matrix math
        double sum_vx = 0, sum_vy = 0, sum_omega = 0;

        for (size_t i = 0; i < 4; i++) {
            double speed = states[i].speed.toMetersPerSecond();
            double angle = states[i].angle.toRadians();

            double vx_module = speed * std::cos(angle);
            double vy_module = speed * std::sin(angle);

            sum_vx += vx_module;
            sum_vy += vy_module;

            // Calculate contribution to rotation
            const Vec2D& pos = modulePositions_[i];
            double tangential = speed;
            double radius = pos.magnitude();
            if (radius > 0.01) {
                sum_omega += tangential / radius;
            }
        }

        return {
            mps(sum_vx / 4.0),
            mps(sum_vy / 4.0),
            radps(sum_omega / 4.0)
        };
    }

    // Normalize wheel speeds if any exceed maximum
    void normalizeWheelSpeeds(std::array<SwerveModuleState, 4>& states,
                              MetersPerSecond maxSpeed) const {
        double maxMagnitude = 0.0;

        for (const auto& state : states) {
            double mag = std::abs(state.speed.toMetersPerSecond());
            if (mag > maxMagnitude) {
                maxMagnitude = mag;
            }
        }

        if (maxMagnitude > maxSpeed.toMetersPerSecond()) {
            double scale = maxSpeed.toMetersPerSecond() / maxMagnitude;
            for (auto& state : states) {
                state.speed = mps(state.speed.toMetersPerSecond() * scale);
            }
        }
    }
};

// ============================================================================
// Helper Functions
// ============================================================================
void printModuleStates(const std::array<SwerveModuleState, 4>& states) {
    const char* names[] = {"FL", "FR", "BL", "BR"};

    for (size_t i = 0; i < 4; i++) {
        print("  ", names[i], ": ");
        print(, );
        print(, states[i].speed.toMetersPerSecond(), " m/s @ ");
        print(, states[i].angle.toDegrees(), "°");
        println("");
    }
}

// ============================================================================
// Demonstration Scenarios
// ============================================================================

void demonstrateBasicMotions() {
    println("========================================");
    println("  Basic Swerve Drive Motions");
    println("========================================\n");

    // Create swerve drive with 0.6m x 0.6m frame
    SwerveDriveKinematics swerve(0.6, 0.6);

    println("Robot: 60cm × 60cm square frame\n");

    // 1. Forward motion
    println("1. Drive Forward (1 m/s):");
    auto states = swerve.toModuleStates(mps(1.0), mps(0), radps(0));
    printModuleStates(states);
    println("   → All modules point forward\n");

    // 2. Strafe right
    println("2. Strafe Right (1 m/s):");
    states = swerve.toModuleStates(mps(0), mps(1.0), radps(0));
    printModuleStates(states);
    println("   → All modules point right (90°)\n");

    // 3. Rotate in place
    println("3. Rotate CCW (1 rad/s):");
    states = swerve.toModuleStates(mps(0), mps(0), radps(1.0));
    printModuleStates(states);
    println("   → Modules form tangent circle\n");

    // 4. Diagonal motion
    println("4. Drive Diagonal (vx=0.7, vy=0.7):");
    states = swerve.toModuleStates(mps(0.7), mps(0.7), radps(0));
    printModuleStates(states);
    println("   → All modules point 45°\n");

    // 5. Complex motion
    println("5. Complex Motion (vx=0.5, vy=0.3, ω=0.5):");
    states = swerve.toModuleStates(mps(0.5), mps(0.3), radps(0.5));
    printModuleStates(states);
    println("   → Each module has unique speed and angle\n");
}

void demonstrateModuleOptimization() {
    println("========================================");
    println("  Module Optimization");
    println("========================================\n");

    println("Scenario: Front-left module currently at 10°");
    println("Command requires 190° rotation\n");

    SwerveModuleState state(mps(2.0), rad(190.0 * constants::DEG_TO_RAD));
    Radians currentAngle = rad(10.0 * constants::DEG_TO_RAD);

    println("Before optimization:");
    print("  Target: " ,  state.speed.toMetersPerSecond() , " m/s @ ");
    print("  Rotation needed: "
              ,  ((state.angle - currentAngle).toDegrees()) , "°\n\n");

    state.optimize(currentAngle);

    println("After optimization:");
    print("  Target: " ,  state.speed.toMetersPerSecond() , " m/s @ ");
    print("  Rotation needed: "
              ,  ((state.angle - currentAngle).normalizeSigned().toDegrees()) , "°\n\n");

    println("Result: Reversed speed and rotated <90° instead!");
    println("This is MUCH faster than rotating 180°\n");
}

void demonstrateSpeedNormalization() {
    println("========================================");
    println("  Wheel Speed Normalization");
    println("========================================\n");

    SwerveDriveKinematics swerve(0.6, 0.6);
    MetersPerSecond maxSpeed = mps(4.0);

    println("Max wheel speed: ", maxSpeed.toMetersPerSecond(), " m/s\n");

    // Request motion that exceeds max speed
    println("Requesting: vx=3, vy=3, ω=2 rad/s\n");

    auto states = swerve.toModuleStates(mps(3.0), mps(3.0), radps(2.0));

    println("Before normalization:");
    printModuleStates(states);

    double maxFound = 0;
    for (const auto& s : states) {
        maxFound = std::max(maxFound, std::abs(s.speed.toMetersPerSecond()));
    }
    println("  Max speed: ", maxFound, " m/s (EXCEEDS LIMIT!)\n");

    swerve.normalizeWheelSpeeds(states, maxSpeed);

    println("After normalization:");
    printModuleStates(states);

    maxFound = 0;
    for (const auto& s : states) {
        maxFound = std::max(maxFound, std::abs(s.speed.toMetersPerSecond()));
    }
    println("  Max speed: ", maxFound, " m/s ✓");
    println("  Direction preserved, speeds scaled down\n");
}

void demonstrateFieldCentric() {
    println("========================================");
    println("  Field-Centric Control");
    println("========================================\n");

    SwerveDriveKinematics swerve(0.6, 0.6);

    Radians robotHeading = rad(90.0 * constants::DEG_TO_RAD);  // Facing left
    println("Robot heading: ", robotHeading.toDegrees(), "° (facing left)\n");

    // Driver wants to drive forward in field frame
    MetersPerSecond vx_field = mps(1.0);
    MetersPerSecond vy_field = mps(0);

    println("Driver command: Forward in field frame");
    print("Field velocities: vx=" ,  vx_field.toMetersPerSecond()
              , ", vy=");

    // Transform to robot frame
    double cos_h = robotHeading.cos();
    double sin_h = robotHeading.sin();

    double vx_robot = vx_field.toMetersPerSecond() * cos_h +
                      vy_field.toMetersPerSecond() * sin_h;
    double vy_robot = -vx_field.toMetersPerSecond() * sin_h +
                       vy_field.toMetersPerSecond() * cos_h;

    println("Transformed to robot frame:");
    println("Robot velocities: vx=", vx_robot, ", vy=", vy_robot, "\n");

    auto states = swerve.toModuleStates(mps(vx_robot), mps(vy_robot), radps(0));

    println("Module states:");
    printModuleStates(states);
    println("\nThe robot will strafe to move forward in the field!");
    println("Driver doesn't need to think about robot orientation\n");
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  Swerve Drive Kinematics              ║");
    println("║  Advanced Holonomic Drive System      ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrateBasicMotions();
    demonstrateModuleOptimization();
    demonstrateSpeedNormalization();
    demonstrateFieldCentric();

    println("========================================");
    println("  Key Concepts");
    println("========================================\n");

    println("Swerve Drive Advantages:");
    println("  + True omnidirectional movement");
    println("  + Maximum maneuverability");
    println("  + Field-centric control natural");
    println("  + No pushing/scrubbing (unlike mecanum)");
    println("  + Better traction than omniwheels\n");

    println("Swerve Drive Challenges:");
    println("  - Complex mechanical design");
    println("  - More motors (8 total for 4 modules)");
    println("  - More expensive than other drives");
    println("  - Requires precise module calibration");
    println("  - More complex control algorithms\n");

    println("Critical Implementation Details:\n");

    println("1. Module Optimization");
    println("   • Always take shortest rotation path");
    println("   • Reverse speed instead of rotating >90°");
    println("   • Saves time and mechanical wear\n");

    println("2. Wheel Speed Limiting");
    println("   • Normalize when speeds exceed maximum");
    println("   • Preserve motion direction");
    println("   • Critical for controllability\n");

    println("3. Field-Centric Control");
    println("   • Transform commands from field to robot frame");
    println("   • Requires accurate heading measurement (IMU)");
    println("   • Much more intuitive for drivers\n");

    println("4. Module Positioning");
    println("   • Symmetric placement reduces coupling");
    println("   • Larger wheelbase = more stable rotation");
    println("   • Center of rotation can be adjusted\n");

    println("Real-World Applications:");
    println("  • FRC robots (Team 254, 1323, etc.)");
    println("  • Warehouse AMRs");
    println("  • Inspection robots");
    println("  • Any application requiring maximum maneuverability\n");

    println("Popular Teams Using Swerve:");
    println("  • Team 254 (Cheesy Poofs)");
    println("  • Team 1323 (Madtown Robotics)");
    println("  • Team 2910 (Jack in the Bot)");
    println("  • Many more in recent FRC seasons!\n");

    return 0;
}

/*
Expected Output:
    - Basic swerve motions (forward, strafe, rotate)
    - Module optimization demonstration
    - Speed normalization example
    - Field-centric control transformation

This example demonstrates:
    - Complete swerve drive kinematics
    - Inverse and forward kinematics
    - Module state optimization
    - Wheel speed normalization
    - Field-centric coordinate transformation
    - Type-safe units throughout

Educational Value:
    - Understand swerve drive mathematics
    - Learn optimization techniques
    - See practical implementation
    - Understand advantages and tradeoffs

Perfect for:
    - FRC teams implementing swerve
    - Understanding advanced kinematics
    - Learning coordinate transformations
    - Seeing professional-level implementations
*/
