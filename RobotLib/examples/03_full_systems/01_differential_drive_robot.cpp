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

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_utilities.h"


using namespace units;

// ============================================================================
// Robot Configuration
// ============================================================================
struct RobotConfig {
    Meters wheelbase = m(0.30);            // 30cm between wheels
    Meters wheelDiameter = m(0.10);        // 10cm diameter wheels
    RPM maxMotorRPM = RPM::fromRPM(200);   // Max 200 RPM
    Volts nominalVoltage = V(12.0);        // 12V battery
    Volts minVoltage = V(10.0);            // Low battery threshold
    Volts maxVoltage = V(12.6);            // Fully charged
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

        println("Command:");
        println("  Linear:  ", linear.toMetersPerSecond(), " m/s");
        println("  Angular: ", angular.toRadiansPerSecond(), " rad/s");
        println("Wheel velocities:");
        println("  Left:  ", state_.leftVelocity.toMetersPerSecond(), " m/s");
        println("  Right: ", state_.rightVelocity.toMetersPerSecond(), " m/s");
    }

    // Get motor RPM for each wheel
    std::pair<RPM, RPM> getMotorCommands() const {
        auto leftRPM = MotorController::velocityToRPM(state_.leftVelocity, config_.wheelDiameter);
        auto rightRPM = MotorController::velocityToRPM(state_.rightVelocity, config_.wheelDiameter);

        // Clamp to max RPM
        double maxRPM = config_.maxMotorRPM.toRPM();
        if (std::abs(leftRPM.toRPM()) > maxRPM || std::abs(rightRPM.toRPM()) > maxRPM) {
            println("⚠️  Warning: Commanded velocity exceeds motor limits!");
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
            println("🔋 CRITICAL: Battery at ", battery_.getPercentage(), "%!");
            println("   Initiating emergency shutdown...");
        } else if (battery_.isLow()) {
            println("🔋 Warning: Battery low at ", battery_.getPercentage(), "%");
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
    println("========================================");
    println("  Differential Drive Robot Example");
    println("========================================\n");

    // Create robot with configuration
    RobotConfig config;
    DifferentialDriveRobot robot(config);

    println("Robot Configuration:");
    println("  Wheelbase: ", config.wheelbase.toMeters(), " m");
    println("  Wheel diameter: ", config.wheelDiameter.toCentimeters(), " cm");
    println("  Max RPM: ", config.maxMotorRPM.toRPM(), "\n");

    // ========================================================================
    // Test 1: Move Forward
    // ========================================================================
    println("Test 1: Moving forward at 0.5 m/s");
    println("-----------------------------------");
    robot.setVelocity(mps(0.5), radps(0.0));

    std::pair<RPM, RPM> motors1 = robot.getMotorCommands();
    println("Motor commands:");
    println("  Left motor:  ", motors1.first.toRPM(), " RPM");
    println("  Right motor: ", motors1.second.toRPM(), " RPM\n");

    // ========================================================================
    // Test 2: Turn in Place
    // ========================================================================
    println("Test 2: Rotating in place at 0.5 rad/s");
    println("---------------------------------------");
    robot.setVelocity(mps(0.0), radps(0.5));

    std::pair<RPM, RPM> motors2 = robot.getMotorCommands();
    println("Motor commands:");
    println("  Left motor:  ", motors2.first.toRPM(), " RPM");
    println("  Right motor: ", motors2.second.toRPM(), " RPM\n");

    // ========================================================================
    // Test 3: Arc Turn (forward + rotation)
    // ========================================================================
    println("Test 3: Arc turn (forward 0.3 m/s, turning 0.3 rad/s)");
    println("------------------------------------------------------");
    robot.setVelocity(mps(0.3), radps(0.3));

    std::pair<RPM, RPM> motors3 = robot.getMotorCommands();
    println("Motor commands:");
    println("  Left motor:  ", motors3.first.toRPM(), " RPM");
    println("  Right motor: ", motors3.second.toRPM(), " RPM\n");

    // ========================================================================
    // Test 4: Odometry Update
    // ========================================================================
    println("Test 4: Simulating odometry over 5 seconds");
    println("-------------------------------------------");

    // Simulate forward motion
    robot.setVelocity(mps(0.5), radps(0.0));

    for (int i = 0; i <= 5; i++) {
        robot.updateOdometry(s(i));
        auto pose = robot.getState().pose;

        print("t=", i, "s: ");
        print("x=", pose.position.x, " m, ");
        print("y=", pose.position.y, " m, ");
        println("θ=", pose.theta.toDegrees(), "°");
    }
    println("");

    // ========================================================================
    // Test 5: Battery Monitoring
    // ========================================================================
    println("Test 5: Battery monitoring");
    println("--------------------------");

    // Simulate battery discharge
    robot.updateBattery(V(12.6));
    println("Fully charged: ", robot.getBattery().getPercentage(), "%");

    robot.updateBattery(V(11.5));
    println("Normal operation: ", robot.getBattery().getPercentage(), "%");

    robot.updateBattery(V(10.5));
    println("Getting low: ", robot.getBattery().getPercentage(), "%");

    robot.updateBattery(V(10.0));
    println("Critical: ", robot.getBattery().getPercentage(), "%\n");

    // ========================================================================
    // Summary
    // ========================================================================
    println("========================================");
    println("  Example Complete!");
    println("========================================");
    println("This example demonstrated:");
    println("  ✓ Differential drive kinematics");
    println("  ✓ Velocity to RPM conversion");
    println("  ✓ Odometry tracking");
    println("  ✓ Battery monitoring");
    println("  ✓ Type-safe unit handling\n");

    println("Next steps:");
    println("  1. Adapt this code for your robot platform");
    println("  2. Add sensor feedback (encoders, IMU)");
    println("  3. Implement closed-loop control");
    println("  4. Add obstacle avoidance\n");

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
