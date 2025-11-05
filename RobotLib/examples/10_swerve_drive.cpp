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

#include "../include/units_core.h"
#include "../include/units_physics.h"
#include "../include/units_robotics.h"
#include "../include/units_utilities.h"
#include "../include/units_math.h"

#include <iostream>
#include <iomanip>
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
        std::cout << "  " << names[i] << ": ";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << std::setw(6) << states[i].speed.toMetersPerSecond() << " m/s @ ";
        std::cout << std::setw(6) << states[i].angle.toDegrees() << "°";
        std::cout << "\n";
    }
}

// ============================================================================
// Demonstration Scenarios
// ============================================================================

void demonstrateBasicMotions() {
    std::cout << "========================================\n";
    std::cout << "  Basic Swerve Drive Motions\n";
    std::cout << "========================================\n\n";

    // Create swerve drive with 0.6m x 0.6m frame
    SwerveDriveKinematics swerve(0.6, 0.6);

    std::cout << "Robot: 60cm × 60cm square frame\n\n";

    // 1. Forward motion
    std::cout << "1. Drive Forward (1 m/s):\n";
    auto states = swerve.toModuleStates(mps(1.0), mps(0), radps(0));
    printModuleStates(states);
    std::cout << "   → All modules point forward\n\n";

    // 2. Strafe right
    std::cout << "2. Strafe Right (1 m/s):\n";
    states = swerve.toModuleStates(mps(0), mps(1.0), radps(0));
    printModuleStates(states);
    std::cout << "   → All modules point right (90°)\n\n";

    // 3. Rotate in place
    std::cout << "3. Rotate CCW (1 rad/s):\n";
    states = swerve.toModuleStates(mps(0), mps(0), radps(1.0));
    printModuleStates(states);
    std::cout << "   → Modules form tangent circle\n\n";

    // 4. Diagonal motion
    std::cout << "4. Drive Diagonal (vx=0.7, vy=0.7):\n";
    states = swerve.toModuleStates(mps(0.7), mps(0.7), radps(0));
    printModuleStates(states);
    std::cout << "   → All modules point 45°\n\n";

    // 5. Complex motion
    std::cout << "5. Complex Motion (vx=0.5, vy=0.3, ω=0.5):\n";
    states = swerve.toModuleStates(mps(0.5), mps(0.3), radps(0.5));
    printModuleStates(states);
    std::cout << "   → Each module has unique speed and angle\n\n";
}

void demonstrateModuleOptimization() {
    std::cout << "========================================\n";
    std::cout << "  Module Optimization\n";
    std::cout << "========================================\n\n";

    std::cout << "Scenario: Front-left module currently at 10°\n";
    std::cout << "Command requires 190° rotation\n\n";

    SwerveModuleState state(mps(2.0), rad(190.0 * constants::DEG_TO_RAD));
    Radians currentAngle = rad(10.0 * constants::DEG_TO_RAD);

    std::cout << "Before optimization:\n";
    std::cout << "  Target: " << state.speed.toMetersPerSecond() << " m/s @ "
              << state.angle.toDegrees() << "°\n";
    std::cout << "  Rotation needed: "
              << ((state.angle - currentAngle).toDegrees()) << "°\n\n";

    state.optimize(currentAngle);

    std::cout << "After optimization:\n";
    std::cout << "  Target: " << state.speed.toMetersPerSecond() << " m/s @ "
              << state.angle.toDegrees() << "°\n";
    std::cout << "  Rotation needed: "
              << ((state.angle - currentAngle).normalizeSigned().toDegrees()) << "°\n\n";

    std::cout << "Result: Reversed speed and rotated <90° instead!\n";
    std::cout << "This is MUCH faster than rotating 180°\n\n";
}

void demonstrateSpeedNormalization() {
    std::cout << "========================================\n";
    std::cout << "  Wheel Speed Normalization\n";
    std::cout << "========================================\n\n";

    SwerveDriveKinematics swerve(0.6, 0.6);
    MetersPerSecond maxSpeed = mps(4.0);

    std::cout << "Max wheel speed: " << maxSpeed.toMetersPerSecond() << " m/s\n\n";

    // Request motion that exceeds max speed
    std::cout << "Requesting: vx=3, vy=3, ω=2 rad/s\n\n";

    auto states = swerve.toModuleStates(mps(3.0), mps(3.0), radps(2.0));

    std::cout << "Before normalization:\n";
    printModuleStates(states);

    double maxFound = 0;
    for (const auto& s : states) {
        maxFound = std::max(maxFound, std::abs(s.speed.toMetersPerSecond()));
    }
    std::cout << "  Max speed: " << maxFound << " m/s (EXCEEDS LIMIT!)\n\n";

    swerve.normalizeWheelSpeeds(states, maxSpeed);

    std::cout << "After normalization:\n";
    printModuleStates(states);

    maxFound = 0;
    for (const auto& s : states) {
        maxFound = std::max(maxFound, std::abs(s.speed.toMetersPerSecond()));
    }
    std::cout << "  Max speed: " << maxFound << " m/s ✓\n";
    std::cout << "  Direction preserved, speeds scaled down\n\n";
}

void demonstrateFieldCentric() {
    std::cout << "========================================\n";
    std::cout << "  Field-Centric Control\n";
    std::cout << "========================================\n\n";

    SwerveDriveKinematics swerve(0.6, 0.6);

    Radians robotHeading = rad(90.0 * constants::DEG_TO_RAD);  // Facing left
    std::cout << "Robot heading: " << robotHeading.toDegrees() << "° (facing left)\n\n";

    // Driver wants to drive forward in field frame
    MetersPerSecond vx_field = mps(1.0);
    MetersPerSecond vy_field = mps(0);

    std::cout << "Driver command: Forward in field frame\n";
    std::cout << "Field velocities: vx=" << vx_field.toMetersPerSecond()
              << ", vy=" << vy_field.toMetersPerSecond() << "\n\n";

    // Transform to robot frame
    double cos_h = robotHeading.cos();
    double sin_h = robotHeading.sin();

    double vx_robot = vx_field.toMetersPerSecond() * cos_h +
                      vy_field.toMetersPerSecond() * sin_h;
    double vy_robot = -vx_field.toMetersPerSecond() * sin_h +
                       vy_field.toMetersPerSecond() * cos_h;

    std::cout << "Transformed to robot frame:\n";
    std::cout << "Robot velocities: vx=" << vx_robot << ", vy=" << vy_robot << "\n\n";

    auto states = swerve.toModuleStates(mps(vx_robot), mps(vy_robot), radps(0));

    std::cout << "Module states:\n";
    printModuleStates(states);
    std::cout << "\nThe robot will strafe to move forward in the field!\n";
    std::cout << "Driver doesn't need to think about robot orientation\n\n";
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Swerve Drive Kinematics              ║\n";
    std::cout << "║  Advanced Holonomic Drive System      ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrateBasicMotions();
    demonstrateModuleOptimization();
    demonstrateSpeedNormalization();
    demonstrateFieldCentric();

    std::cout << "========================================\n";
    std::cout << "  Key Concepts\n";
    std::cout << "========================================\n\n";

    std::cout << "Swerve Drive Advantages:\n";
    std::cout << "  + True omnidirectional movement\n";
    std::cout << "  + Maximum maneuverability\n";
    std::cout << "  + Field-centric control natural\n";
    std::cout << "  + No pushing/scrubbing (unlike mecanum)\n";
    std::cout << "  + Better traction than omniwheels\n\n";

    std::cout << "Swerve Drive Challenges:\n";
    std::cout << "  - Complex mechanical design\n";
    std::cout << "  - More motors (8 total for 4 modules)\n";
    std::cout << "  - More expensive than other drives\n";
    std::cout << "  - Requires precise module calibration\n";
    std::cout << "  - More complex control algorithms\n\n";

    std::cout << "Critical Implementation Details:\n\n";

    std::cout << "1. Module Optimization\n";
    std::cout << "   • Always take shortest rotation path\n";
    std::cout << "   • Reverse speed instead of rotating >90°\n";
    std::cout << "   • Saves time and mechanical wear\n\n";

    std::cout << "2. Wheel Speed Limiting\n";
    std::cout << "   • Normalize when speeds exceed maximum\n";
    std::cout << "   • Preserve motion direction\n";
    std::cout << "   • Critical for controllability\n\n";

    std::cout << "3. Field-Centric Control\n";
    std::cout << "   • Transform commands from field to robot frame\n";
    std::cout << "   • Requires accurate heading measurement (IMU)\n";
    std::cout << "   • Much more intuitive for drivers\n\n";

    std::cout << "4. Module Positioning\n";
    std::cout << "   • Symmetric placement reduces coupling\n";
    std::cout << "   • Larger wheelbase = more stable rotation\n";
    std::cout << "   • Center of rotation can be adjusted\n\n";

    std::cout << "Real-World Applications:\n";
    std::cout << "  • FRC robots (Team 254, 1323, etc.)\n";
    std::cout << "  • Warehouse AMRs\n";
    std::cout << "  • Inspection robots\n";
    std::cout << "  • Any application requiring maximum maneuverability\n\n";

    std::cout << "Popular Teams Using Swerve:\n";
    std::cout << "  • Team 254 (Cheesy Poofs)\n";
    std::cout << "  • Team 1323 (Madtown Robotics)\n";
    std::cout << "  • Team 2910 (Jack in the Bot)\n";
    std::cout << "  • Many more in recent FRC seasons!\n\n";

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
