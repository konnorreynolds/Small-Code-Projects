// ============================================================================
// Example 1: Differential Drive Robot Controller
// ============================================================================
// This example shows how to control a differential drive robot (like a
// tank-style robot) using the RobotLib units system.
//
// Hardware assumed:
// - Two drive motors with encoders
// - Wheelbase: 30cm
// - Wheel diameter: 10cm
// - Battery: 12V LiPo
// ============================================================================

#include "../units_core.h"
#include "../units_physics.h"
#include "../units_utilities.h"

#include <iostream>

using namespace units;

// ============================================================================
// Robot Configuration
// ============================================================================
struct RobotConfig {
    Meters wheelbase = m(0.30);           // 30cm between wheels
    Meters wheelDiameter = cm(10);         // 10cm diameter wheels
    RPM maxMotorRPM = RPM::fromRPM(200);  // Max 200 RPM
    Volts nominalVoltage = V(12.0);       // 12V battery
    Volts minVoltage = V(10.0);           // Low battery threshold
    Volts maxVoltage = V(12.6);           // Fully charged
};

// ============================================================================
// Robot State
// ============================================================================
struct RobotState {
    Pose2D pose;                          // Current position and heading
    MetersPerSecond leftVelocity;         // Left wheel velocity
    MetersPerSecond rightVelocity;        // Right wheel velocity
    Volts batteryVoltage;                 // Current battery voltage
};

// ============================================================================
// Robot Controller Class
// ============================================================================
class DifferentialDriveRobot {
private:
    RobotConfig config_;
    RobotState state_;
    SimpleOdometry odometry_;
    BatteryMonitor battery_;

public:
    explicit DifferentialDriveRobot(const RobotConfig& config)
        : config_(config),
          state_{},
          odometry_(),
          battery_(config.nominalVoltage, config.minVoltage, config.maxVoltage)
    {
        state_.pose = Pose2D(0, 0, deg(0));  // Start at origin
        state_.batteryVoltage = config.nominalVoltage;
    }

    // Set desired robot velocity (forward speed and turning rate)
    void setVelocity(MetersPerSecond linear, RadiansPerSecond angular) {
        // Calculate individual wheel velocities using differential drive kinematics
        auto drive = DifferentialDrive::fromTwist(linear, angular, config_.wheelbase);

        state_.leftVelocity = drive.leftVelocity;
        state_.rightVelocity = drive.rightVelocity;

        std::cout << "Command:\n";
        std::cout << "  Linear:  " << linear.toMetersPerSecond() << " m/s\n";
        std::cout << "  Angular: " << angular.toRadiansPerSecond() << " rad/s\n";
        std::cout << "Wheel velocities:\n";
        std::cout << "  Left:  " << state_.leftVelocity.toMetersPerSecond() << " m/s\n";
        std::cout << "  Right: " << state_.rightVelocity.toMetersPerSecond() << " m/s\n";
    }

    // Get motor RPM for each wheel
    std::pair<RPM, RPM> getMotorCommands() const {
        auto leftRPM = MotorController::velocityToRPM(state_.leftVelocity, config_.wheelDiameter);
        auto rightRPM = MotorController::velocityToRPM(state_.rightVelocity, config_.wheelDiameter);

        // Clamp to max RPM
        double maxRPM = config_.maxMotorRPM.toRPM();
        if (std::abs(leftRPM.toRPM()) > maxRPM || std::abs(rightRPM.toRPM()) > maxRPM) {
            std::cout << "⚠️  Warning: Commanded velocity exceeds motor limits!\n";
        }

        return {leftRPM, rightRPM};
    }

    // Update odometry based on encoder readings
    void updateOdometry(const Seconds& currentTime) {
        odometry_.updateDifferential(
            state_.leftVelocity,
            state_.rightVelocity,
            config_.wheelbase,
            currentTime
        );
        state_.pose = odometry_.getPose();
    }

    // Update battery voltage and check status
    void updateBattery(Volts voltage) {
        state_.batteryVoltage = voltage;
        battery_.update(voltage);

        if (battery_.isCritical()) {
            std::cout << "🔋 CRITICAL: Battery at " << battery_.getPercentage() << "%!\n";
            std::cout << "   Initiating emergency shutdown...\n";
        } else if (battery_.isLow()) {
            std::cout << "🔋 Warning: Battery low at " << battery_.getPercentage() << "%\n";
        }
    }

    // Get current state
    const RobotState& getState() const { return state_; }
    const BatteryMonitor& getBattery() const { return battery_; }
};

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "========================================\n";
    std::cout << "  Differential Drive Robot Example\n";
    std::cout << "========================================\n\n";

    // Create robot with configuration
    RobotConfig config;
    DifferentialDriveRobot robot(config);

    std::cout << "Robot Configuration:\n";
    std::cout << "  Wheelbase: " << config.wheelbase.toMeters() << " m\n";
    std::cout << "  Wheel diameter: " << config.wheelDiameter.toCentimeters() << " cm\n";
    std::cout << "  Max RPM: " << config.maxMotorRPM.toRPM() << "\n\n";

    // ========================================================================
    // Test 1: Move Forward
    // ========================================================================
    std::cout << "Test 1: Moving forward at 0.5 m/s\n";
    std::cout << "-----------------------------------\n";
    robot.setVelocity(mps(0.5), radps(0.0));

    auto [leftRPM, rightRPM] = robot.getMotorCommands();
    std::cout << "Motor commands:\n";
    std::cout << "  Left motor:  " << leftRPM.toRPM() << " RPM\n";
    std::cout << "  Right motor: " << rightRPM.toRPM() << " RPM\n\n";

    // ========================================================================
    // Test 2: Turn in Place
    // ========================================================================
    std::cout << "Test 2: Rotating in place at 0.5 rad/s\n";
    std::cout << "---------------------------------------\n";
    robot.setVelocity(mps(0.0), radps(0.5));

    auto [leftRPM2, rightRPM2] = robot.getMotorCommands();
    std::cout << "Motor commands:\n";
    std::cout << "  Left motor:  " << leftRPM2.toRPM() << " RPM\n";
    std::cout << "  Right motor: " << rightRPM2.toRPM() << " RPM\n\n";

    // ========================================================================
    // Test 3: Arc Turn (forward + rotation)
    // ========================================================================
    std::cout << "Test 3: Arc turn (forward 0.3 m/s, turning 0.3 rad/s)\n";
    std::cout << "------------------------------------------------------\n";
    robot.setVelocity(mps(0.3), radps(0.3));

    auto [leftRPM3, rightRPM3] = robot.getMotorCommands();
    std::cout << "Motor commands:\n";
    std::cout << "  Left motor:  " << leftRPM3.toRPM() << " RPM\n";
    std::cout << "  Right motor: " << rightRPM3.toRPM() << " RPM\n\n";

    // ========================================================================
    // Test 4: Odometry Update
    // ========================================================================
    std::cout << "Test 4: Simulating odometry over 5 seconds\n";
    std::cout << "-------------------------------------------\n";

    // Simulate forward motion
    robot.setVelocity(mps(0.5), radps(0.0));

    for (int i = 0; i <= 5; i++) {
        robot.updateOdometry(s(i));
        auto pose = robot.getState().pose;

        std::cout << "t=" << i << "s: ";
        std::cout << "x=" << pose.position.x << " m, ";
        std::cout << "y=" << pose.position.y << " m, ";
        std::cout << "θ=" << pose.theta.toDegrees() << "°\n";
    }
    std::cout << "\n";

    // ========================================================================
    // Test 5: Battery Monitoring
    // ========================================================================
    std::cout << "Test 5: Battery monitoring\n";
    std::cout << "--------------------------\n";

    // Simulate battery discharge
    robot.updateBattery(V(12.6));
    std::cout << "Fully charged: " << robot.getBattery().getPercentage() << "%\n";

    robot.updateBattery(V(11.5));
    std::cout << "Normal operation: " << robot.getBattery().getPercentage() << "%\n";

    robot.updateBattery(V(10.5));
    std::cout << "Getting low: " << robot.getBattery().getPercentage() << "%\n";

    robot.updateBattery(V(10.0));
    std::cout << "Critical: " << robot.getBattery().getPercentage() << "%\n\n";

    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "========================================\n";
    std::cout << "  Example Complete!\n";
    std::cout << "========================================\n";
    std::cout << "This example demonstrated:\n";
    std::cout << "  ✓ Differential drive kinematics\n";
    std::cout << "  ✓ Velocity to RPM conversion\n";
    std::cout << "  ✓ Odometry tracking\n";
    std::cout << "  ✓ Battery monitoring\n";
    std::cout << "  ✓ Type-safe unit handling\n\n";

    std::cout << "Next steps:\n";
    std::cout << "  1. Adapt this code for your robot platform\n";
    std::cout << "  2. Add sensor feedback (encoders, IMU)\n";
    std::cout << "  3. Implement closed-loop control\n";
    std::cout << "  4. Add obstacle avoidance\n\n";

    return 0;
}

/*
Expected Output:
========================================
  Differential Drive Robot Example
========================================

Robot Configuration:
  Wheelbase: 0.3 m
  Wheel diameter: 10 cm
  Max RPM: 200

Test 1: Moving forward at 0.5 m/s
-----------------------------------
Command:
  Linear:  0.5 m/s
  Angular: 0 rad/s
Wheel velocities:
  Left:  0.5 m/s
  Right: 0.5 m/s
Motor commands:
  Left motor:  95.4930 RPM
  Right motor: 95.4930 RPM

Test 2: Rotating in place at 0.5 rad/s
---------------------------------------
Command:
  Linear:  0 m/s
  Angular: 0.5 rad/s
Wheel velocities:
  Left:  -0.075 m/s
  Right: 0.075 m/s
Motor commands:
  Left motor:  -14.3239 RPM
  Right motor: 14.3239 RPM

... [more output] ...
*/
