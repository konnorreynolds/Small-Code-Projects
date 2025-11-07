// ============================================================================
// EXAMPLE: Complete Robot Configuration
// ============================================================================
// This example shows how to organize a complete robot project with:
//   - Hardware pin definitions
//   - Physical constants
//   - Subsystem configuration
//   - Autonomous routines
//   - Clean separation of concerns
//
// BEST PRACTICES SHOWN:
//   1. Centralize all configuration in one place
//   2. Use meaningful names (not magic numbers)
//   3. Document units and physical meaning
//   4. Separate hardware config from logic
//   5. Make it easy to update for different robots
//
// ============================================================================

#ifndef CUSTOM_ROBOT_CONFIG_H
#define CUSTOM_ROBOT_CONFIG_H

#include "../../include/robotlib_api.h"
#include "custom_motor_driver.h"
#include "custom_subsystems.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
// Define all pin numbers and hardware IDs in one place
// Makes it easy to rewire robot without changing code logic
// ============================================================================
namespace hardware {
    // ========================================================================
    // DRIVE SYSTEM
    // ========================================================================
    constexpr int DRIVE_LEFT_PWM = 9;
    constexpr int DRIVE_LEFT_DIR = 8;
    constexpr int DRIVE_RIGHT_PWM = 11;
    constexpr int DRIVE_RIGHT_DIR = 10;

    // Encoder pins (if using encoders)
    constexpr int LEFT_ENCODER_A = 2;
    constexpr int LEFT_ENCODER_B = 3;
    constexpr int RIGHT_ENCODER_A = 4;
    constexpr int RIGHT_ENCODER_B = 5;

    // ========================================================================
    // ARM SYSTEM
    // ========================================================================
    constexpr int ARM_CAN_ID = 5;           // CAN bus ID
    constexpr int ARM_LIMIT_SWITCH_MIN = 12; // Lower limit switch
    constexpr int ARM_LIMIT_SWITCH_MAX = 13; // Upper limit switch

    // ========================================================================
    // INTAKE/MANIPULATOR
    // ========================================================================
    constexpr int INTAKE_PWM = 6;
    constexpr int INTAKE_DIR = 7;
    constexpr int GRIPPER_SERVO = 14;

    // ========================================================================
    // SENSORS
    // ========================================================================
    constexpr int ULTRASONIC_TRIGGER = 15;
    constexpr int ULTRASONIC_ECHO = 16;
    constexpr int GYRO_I2C_ADDRESS = 0x68;  // MPU6050 default
    constexpr int COLOR_SENSOR_I2C = 0x29;

    // ========================================================================
    // USER CONTROLS
    // ========================================================================
    constexpr int EMERGENCY_STOP_BUTTON = 17;
    constexpr int MODE_SWITCH = 18;
}

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================
// Robot's physical measurements and characteristics
// These are CRITICAL for accurate odometry and control!
// ============================================================================
namespace physical {
    using namespace units::literals;

    // ========================================================================
    // DRIVE GEOMETRY
    // ========================================================================
    // MEASURE CAREFULLY!
    // Wheelbase: Distance between left and right wheels (center to center)
    // Wheel diameter: Measure with calipers, account for tire compression
    //
    constexpr auto WHEELBASE = m(0.508);         // 20 inches = 0.508 meters
    constexpr auto WHEEL_DIAMETER = m(0.1016);   // 4 inches = 0.1016 meters
    constexpr auto TRACK_WIDTH = WHEELBASE;      // Synonym

    // Maximum speeds (for safety and control)
    // Note: Can't use constexpr for these due to C++11 limitations
    inline units::MetersPerSecond maxLinearSpeed() { return mps(2.0); }
    inline units::RadiansPerSecond maxAngularSpeed() { return radps(3.0); }

    // ========================================================================
    // MOTOR CHARACTERISTICS
    // ========================================================================
    // Look these up in motor datasheets!
    //
    inline units::RPM driveMotorFreeSpeed() { return rpm(5880); }  // NEO motor
    constexpr double DRIVE_GEAR_RATIO = 10.71;                     // 10.71:1 reduction
    inline units::Newtons driveStallTorque() { return units::Newtons::fromNewtons(2.6); }

    inline units::RPM armMotorFreeSpeed() { return rpm(6000); }
    constexpr double ARM_GEAR_RATIO = 100.0;                       // 100:1 reduction
    inline units::Newtons armStallTorque() { return units::Newtons::fromNewtons(3.0); }

    // ========================================================================
    // ARM GEOMETRY
    // ========================================================================
    constexpr auto ARM_LENGTH = m(0.6);           // 60cm arm
    constexpr auto ARM_MIN_ANGLE = deg(-90);      // -90° (below horizontal)
    constexpr auto ARM_MAX_ANGLE = deg(135);      // 135° (above horizontal)
    constexpr auto ARM_STARTING_ANGLE = deg(0);   // Horizontal at start

    // ========================================================================
    // ROBOT MASS (for physics calculations)
    // ========================================================================
    constexpr auto ROBOT_MASS = kg(50.0);         // 50 kg robot
    constexpr auto ARM_MASS = kg(2.5);            // 2.5 kg arm
}

// ============================================================================
// CONTROL PARAMETERS
// ============================================================================
// Tuned constants for PID controllers, filters, etc.
// TUNE THESE ON YOUR ACTUAL ROBOT!
// ============================================================================
namespace control {
    // ========================================================================
    // DRIVE PID (for position/velocity control)
    // ========================================================================
    constexpr double DRIVE_KP = 0.8;
    constexpr double DRIVE_KI = 0.01;
    constexpr double DRIVE_KD = 0.05;

    // ========================================================================
    // ARM PID
    // ========================================================================
    constexpr double ARM_KP = 1.5;
    constexpr double ARM_KI = 0.1;
    constexpr double ARM_KD = 0.08;

    // Feedforward (overcome gravity and friction)
    constexpr double ARM_KS = 0.05;   // Static friction
    constexpr double ARM_KV = 0.002;  // Velocity feedforward
    constexpr double ARM_KG = 0.3;    // Gravity compensation

    // ========================================================================
    // FILTER PARAMETERS
    // ========================================================================
    constexpr double GYRO_FILTER_ALPHA = 0.9;        // Low-pass filter
    constexpr double IMU_COMP_FILTER_ALPHA = 0.98;   // Complementary filter
    constexpr size_t DISTANCE_FILTER_WINDOW = 10;    // Moving average
}

// ============================================================================
// COMPLETE ROBOT CLASS (using all configuration)
// ============================================================================
class ConfiguredRobot {
private:
    // Subsystems
    DriveSubsystem drive_;
    ArmSubsystem arm_;
    IntakeSubsystem intake_;
    ServoMotor gripper_;

    // Timing
    double last_update_time_;

    // State
    enum class RobotMode {
        DISABLED,
        TELEOP,
        AUTONOMOUS
    };
    RobotMode mode_;

public:
    // ========================================================================
    // Constructor - Apply all configuration
    // ========================================================================
    ConfiguredRobot()
        : drive_(hardware::DRIVE_LEFT_PWM, hardware::DRIVE_LEFT_DIR,
                hardware::DRIVE_RIGHT_PWM, hardware::DRIVE_RIGHT_DIR)
        , arm_(hardware::ARM_CAN_ID)
        , intake_(hardware::INTAKE_PWM, hardware::INTAKE_DIR)
        , gripper_(hardware::GRIPPER_SERVO)
        , last_update_time_(0.0)
        , mode_(RobotMode::DISABLED)
    {
        // ====================================================================
        // CONFIGURE DRIVE
        // ====================================================================
        // Note: In a real implementation, you would expose the drive controller
        // from the DriveSubsystem and configure it. For this example,
        // configuration happens in the subsystem constructor.

        // ====================================================================
        // CONFIGURE ARM
        // ====================================================================
        // Note: In a real implementation, you would expose the arm controller
        // from the ArmSubsystem and configure it. For this example,
        // configuration happens in the subsystem constructor.
    }

    // ========================================================================
    // UPDATE - Main periodic function
    // ========================================================================
    void update(double current_time) {
        double dt = current_time - last_update_time_;
        last_update_time_ = current_time;

        // Safety check
        if (dt > 0.1 || dt <= 0.0) dt = 0.02;

        // Update based on mode
        switch (mode_) {
            case RobotMode::DISABLED:
                // Don't update motors when disabled
                break;

            case RobotMode::TELEOP:
            case RobotMode::AUTONOMOUS:
                // Normal operation
                updateSubsystems(dt);
                break;
        }
    }

    // ========================================================================
    // AUTONOMOUS ROUTINES
    // ========================================================================

    // Simple autonomous: drive forward, raise arm, intake
    void autonomousRoutine1() {
        static double auto_start_time = 0;
        double auto_time = last_update_time_ - auto_start_time;

        if (auto_start_time == 0) {
            auto_start_time = last_update_time_;
        }

        // Time-based autonomous
        if (auto_time < 2.0) {
            // First 2 seconds: drive forward
            drive_.arcade(0.5, 0.0);
        }
        else if (auto_time < 4.0) {
            // Next 2 seconds: raise arm while stopped
            drive_.stop();
            arm_.moveTo(physical::ARM_MAX_ANGLE);
        }
        else if (auto_time < 6.0) {
            // Next 2 seconds: intake
            intake_.intake();
        }
        else {
            // Done: stop everything
            drive_.stop();
            arm_.stop();
            intake_.stop();
        }
    }

    // Position-based autonomous using odometry
    void autonomousDriveToPosition(double target_x, double target_y) {
        double current_x = drive_.getX();
        double current_y = drive_.getY();

        // Calculate error
        double error_x = target_x - current_x;
        double error_y = target_y - current_y;
        double distance = std::sqrt(error_x * error_x + error_y * error_y);

        if (distance > 0.1) {  // 10cm threshold
            // Calculate heading to target
            double target_heading = std::atan2(error_y, error_x);
            double current_heading = drive_.getHeading();
            double heading_error = target_heading - current_heading;

            // Simple proportional control
            double forward = distance * 0.5;  // P control on distance
            double turn = heading_error * 1.0; // P control on heading

            drive_.arcade(forward, turn);
        } else {
            drive_.stop();  // Close enough!
        }
    }

    // ========================================================================
    // MODE CONTROL
    // ========================================================================
    void setMode(RobotMode mode) { mode_ = mode; }
    RobotMode getMode() const { return mode_; }

    void enable() { mode_ = RobotMode::TELEOP; }
    void disable() {
        mode_ = RobotMode::DISABLED;
        emergencyStop();
    }

    // ========================================================================
    // SUBSYSTEM ACCESS
    // ========================================================================
    DriveSubsystem& getDrive() { return drive_; }
    ArmSubsystem& getArm() { return arm_; }
    IntakeSubsystem& getIntake() { return intake_; }
    ServoMotor& getGripper() { return gripper_; }

    // ========================================================================
    // SAFETY
    // ========================================================================
    void emergencyStop() {
        drive_.stop();
        arm_.stop();
        intake_.stop();
    }

private:
    void updateSubsystems(double dt) {
        // Read sensors (in real code)
        auto left_enc = units::m(0);
        auto right_enc = units::m(0);

        // Update all subsystems
        drive_.update(dt, left_enc, right_enc);
        arm_.update(dt);
        intake_.update(dt);
    }
};

#endif // CUSTOM_ROBOT_CONFIG_H


// ============================================================================
// USAGE IN MAIN.CPP
// ============================================================================
#if 0

// ============================================================================
// CLEAN MAIN FILE - All configuration is in the header!
// ============================================================================

#include "custom_robot_config.h"

ConfiguredRobot robot;

void setup() {
    // That's it! Robot is fully configured from header file
}

void loop() {
    double current_time = millis() / 1000.0;

    // Update everything
    robot.update(current_time);

    // Simple teleop control
    // double forward = analogRead(A0) / 512.0 - 1.0;  // Joystick
    // double turn = analogRead(A1) / 512.0 - 1.0;
    // robot.getDrive().arcade(forward, turn);
}

// ============================================================================
// DIFFERENT ROBOT? Just change the configuration header!
// ============================================================================
// For a different robot:
// 1. Copy custom_robot_config.h → my_robot_config.h
// 2. Change the namespace values (pins, dimensions, etc.)
// 3. #include "my_robot_config.h"
// 4. Done! Same code works with different hardware
//
// This is the power of good configuration management!
// ============================================================================

#endif
