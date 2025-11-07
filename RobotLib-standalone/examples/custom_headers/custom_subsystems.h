// ============================================================================
// EXAMPLE: Custom Subsystems
// ============================================================================
// This example shows how to create custom subsystems that can be easily
// updated in a single loop.
//
// KEY PATTERN:
//   1. Each subsystem has an update() method
//   2. Robot class manages all subsystems
//   3. Single loop calls robot.update() → updates everything
//
// BENEFITS:
//   - Clean separation of concerns
//   - Easy to add/remove subsystems
//   - Consistent timing across all systems
//   - Simple main loop
//
// ============================================================================

#ifndef CUSTOM_SUBSYSTEMS_H
#define CUSTOM_SUBSYSTEMS_H

#include "../../include/robotlib_api.h"
#include "custom_motor_driver.h"

// ============================================================================
// SUBSYSTEM BASE PATTERN
// ============================================================================
// All subsystems should follow this pattern for consistency
// ============================================================================

// ============================================================================
// EXAMPLE 1: Drive Subsystem
// ============================================================================
// Manages robot drivetrain (wheels, motors, odometry)
// ============================================================================
class DriveSubsystem {
private:
    // Hardware
    SimplePWMMotor left_motor_;
    SimplePWMMotor right_motor_;

    // Robot configuration
    robotlib::DifferentialDrive drive_;

    // State
    bool stopped_;

public:
    // ========================================================================
    // Constructor - Define robot hardware configuration
    // ========================================================================
    DriveSubsystem(int left_pwm, int left_dir,
                   int right_pwm, int right_dir)
        : left_motor_(left_pwm, left_dir, units::rpm(200), 20.0)
        , right_motor_(right_pwm, right_dir, units::rpm(200), 20.0)
        , drive_()
        , stopped_(true)
    {
        // Configure drive system
        drive_.withWheelbase(units::m(0.5))
              .withWheelDiameter(units::m(0.1))
              .withMaxSpeed(units::mps(2.0));
    }

    // ========================================================================
    // UPDATE - Called every loop iteration
    // ========================================================================
    // PARAMETERS:
    //   dt: Time since last update (seconds)
    //   encoder_left: Left wheel encoder reading
    //   encoder_right: Right wheel encoder reading
    //
    // WHAT THIS DOES:
    //   1. Updates odometry from encoder readings
    //   2. Applies motor commands from drive controller
    //   3. Returns current state
    //
    void update(double dt,
                const units::Meters& encoder_left,
                const units::Meters& encoder_right)
    {
        if (!stopped_) {
            // Update odometry
            drive_.updateEncoders(encoder_left, encoder_right)
                  .updateOdometry(dt);

            // Apply motor commands
            left_motor_.setDutyCycle(drive_.getLeftDuty());
            right_motor_.setDutyCycle(drive_.getRightDuty());
        }
    }

    // ========================================================================
    // COMMAND INTERFACE
    // ========================================================================
    void drive(const units::MetersPerSecond& linear,
               const units::RadiansPerSecond& angular)
    {
        drive_.drive(linear, angular);
        stopped_ = false;
    }

    void arcade(double forward, double turn) {
        drive_.arcade(forward, turn);
        stopped_ = false;
    }

    void stop() {
        drive_.stop();
        left_motor_.stop();
        right_motor_.stop();
        stopped_ = true;
    }

    // ========================================================================
    // STATE GETTERS
    // ========================================================================
    double getX() const { return drive_.getX(); }
    double getY() const { return drive_.getY(); }
    double getHeading() const { return drive_.getTheta(); }
    bool isStopped() const { return stopped_; }
};


// ============================================================================
// EXAMPLE 2: Arm Subsystem
// ============================================================================
// Manages robotic arm with position control
// ============================================================================
class ArmSubsystem {
private:
    CANMotorController motor_;
    robotlib::Arm arm_controller_;

    units::Radians target_angle_;
    bool enabled_;

public:
    ArmSubsystem(int can_id)
        : motor_(can_id)
        , arm_controller_()
        , target_angle_(units::rad(0))
        , enabled_(false)
    {
        // Configure arm controller with PID
        arm_controller_.withPID(1.0, 0.1, 0.05)
                      .withFeedforward(0.05, 0.001)
                      .withLimits(units::deg(-90), units::deg(90));

        // Configure motor controller
        motor_.configurePID(1.0, 0.1, 0.05);
    }

    // ========================================================================
    // UPDATE - Called every loop
    // ========================================================================
    void update(double dt) {
        if (enabled_) {
            // Read current position from motor encoder
            auto current_position = motor_.getPosition();

            // Update arm controller (calculates new duty cycle)
            arm_controller_.update(dt, current_position);

            // Apply control output to motor
            motor_.setDutyCycle(arm_controller_.getDutyCycle());
        }
    }

    // ========================================================================
    // COMMANDS
    // ========================================================================
    template<typename AngleRatio>
    void moveTo(const units::Angle<AngleRatio>& angle) {
        arm_controller_.moveTo(angle);
        enabled_ = true;
    }

    void stop() {
        arm_controller_.stop();
        motor_.setDutyCycle(0.0);
        enabled_ = false;
    }

    // ========================================================================
    // STATE
    // ========================================================================
    units::Radians getPosition() const {
        return arm_controller_.getPosition();
    }

    bool atTarget() const {
        return arm_controller_.isAtTarget();
    }
};


// ============================================================================
// EXAMPLE 3: Intake Subsystem
// ============================================================================
// Simple motor for intaking game pieces
// ============================================================================
class IntakeSubsystem {
private:
    SimplePWMMotor motor_;
    double speed_;

public:
    IntakeSubsystem(int pwm_pin, int dir_pin)
        : motor_(pwm_pin, dir_pin, units::rpm(500), 5.0)
        , speed_(0.0)
    {}

    // ========================================================================
    // UPDATE - Apply current speed
    // ========================================================================
    void update(double dt) {
        (void)dt;  // Not used for simple motor
        motor_.setDutyCycle(speed_);
    }

    // ========================================================================
    // COMMANDS
    // ========================================================================
    void intake() { speed_ = 0.8; }      // Run forward at 80%
    void outtake() { speed_ = -0.8; }    // Run backward at 80%
    void stop() { speed_ = 0.0; }        // Stop

    bool isRunning() const { return speed_ != 0.0; }
};


// ============================================================================
// EXAMPLE 4: Sensor Subsystem
// ============================================================================
// Manages all robot sensors and filtering
// ============================================================================
class SensorSubsystem {
private:
    // Filters for noisy sensors
    units::robotics::LowPassFilter gyro_filter_;
    units::robotics::MovingAverageFilter<10> distance_filter_;
    units::robotics::ComplementaryFilter imu_filter_;

    // Sensor readings (cached)
    units::Radians heading_;
    units::Meters distance_to_target_;

public:
    SensorSubsystem()
        : gyro_filter_(0.9)
        , distance_filter_()
        , imu_filter_(0.98)
        , heading_(units::rad(0))
        , distance_to_target_(units::m(0))
    {}

    // ========================================================================
    // UPDATE - Read and filter all sensors
    // ========================================================================
    void update(double dt) {
        // ====================================================================
        // READ RAW SENSOR DATA (hardware-specific)
        // ====================================================================
        // double raw_gyro = readGyroSensor();
        // double raw_distance = readUltrasonicSensor();
        // double raw_accel_angle = readAccelerometerAngle();

        // Placeholder values for example
        double raw_gyro = 0.0;
        double raw_distance = 1.0;
        units::RadiansPerSecond gyro_rate = units::radps(0.0);
        units::Radians accel_angle = units::rad(0.0);

        // ====================================================================
        // APPLY FILTERS
        // ====================================================================
        double filtered_gyro = gyro_filter_.update(raw_gyro);
        double filtered_distance = distance_filter_.update(raw_distance);

        // Fuse gyro and accelerometer for accurate heading
        heading_ = imu_filter_.updateAngle(gyro_rate, accel_angle, dt);

        distance_to_target_ = units::Meters::fromMeters(filtered_distance);
    }

    // ========================================================================
    // GETTERS
    // ========================================================================
    units::Radians getHeading() const { return heading_; }
    units::Meters getDistanceToTarget() const { return distance_to_target_; }
};


// ============================================================================
// COMPLETE ROBOT CLASS
// ============================================================================
// Manages all subsystems and provides unified update loop
//
// PATTERN: This is the key to easy multi-subsystem updates!
// ============================================================================
class MyRobot {
private:
    // All subsystems
    DriveSubsystem drive_;
    ArmSubsystem arm_;
    IntakeSubsystem intake_;
    SensorSubsystem sensors_;

    // Timing
    double last_update_time_;

public:
    // ========================================================================
    // Constructor - Initialize all subsystems
    // ========================================================================
    MyRobot()
        : drive_(9, 8, 11, 10)    // Left PWM/DIR, Right PWM/DIR
        , arm_(5)                  // CAN ID 5
        , intake_(6, 7)            // PWM 6, DIR 7
        , sensors_()
        , last_update_time_(0.0)
    {
        // All subsystems initialized!
    }

    // ========================================================================
    // UPDATE - Single method updates EVERYTHING!
    // ========================================================================
    // CALL THIS FROM YOUR MAIN LOOP!
    //
    // PATTERN:
    //   void loop() {
    //       robot.update(current_time);
    //       // All subsystems are now updated!
    //   }
    //
    void update(double current_time) {
        // Calculate dt (time since last update)
        double dt = current_time - last_update_time_;
        last_update_time_ = current_time;

        // Safety: prevent huge dt on first call or after pause
        if (dt > 0.1 || dt <= 0.0) dt = 0.02;  // Default to 50Hz

        // ====================================================================
        // UPDATE ALL SUBSYSTEMS (order can matter!)
        // ====================================================================

        // 1. Read sensors first (need fresh data for control)
        sensors_.update(dt);

        // 2. Update drive (uses encoder data)
        // In real code, you'd read actual encoders here
        auto left_encoder = units::m(0);
        auto right_encoder = units::m(0);
        drive_.update(dt, left_encoder, right_encoder);

        // 3. Update arm
        arm_.update(dt);

        // 4. Update intake
        intake_.update(dt);

        // All subsystems updated in one call!
    }

    // ========================================================================
    // SUBSYSTEM ACCESS - Get references to control them
    // ========================================================================
    DriveSubsystem& getDrive() { return drive_; }
    ArmSubsystem& getArm() { return arm_; }
    IntakeSubsystem& getIntake() { return intake_; }
    SensorSubsystem& getSensors() { return sensors_; }

    // ========================================================================
    // EMERGENCY STOP - Stop everything immediately
    // ========================================================================
    void emergencyStop() {
        drive_.stop();
        arm_.stop();
        intake_.stop();
    }
};

#endif // CUSTOM_SUBSYSTEMS_H


// ============================================================================
// USAGE EXAMPLE
// ============================================================================
#if 0  // Set to 1 to include in compilation

// ============================================================================
// CLEAN MAIN LOOP - This is the goal!
// ============================================================================
MyRobot robot;

void setup() {
    // Initialize robot (done in constructor)
}

void loop() {
    // Get current time (milliseconds → seconds)
    double current_time = millis() / 1000.0;

    // ========================================================================
    // UPDATE EVERYTHING WITH ONE CALL!
    // ========================================================================
    robot.update(current_time);

    // ========================================================================
    // CONTROL LOGIC (after update, with fresh data)
    // ========================================================================

    // Example: Autonomous driving
    robot.getDrive().arcade(0.5, 0.0);  // Drive forward

    // Example: Arm control
    if (!robot.getArm().atTarget()) {
        // Still moving to target
    }

    // Example: Intake control based on sensor
    if (robot.getSensors().getDistanceToTarget().toMeters() < 0.5) {
        robot.getIntake().intake();  // Close enough, intake!
    } else {
        robot.getIntake().stop();
    }
}

// ============================================================================
// TELEOP CONTROL EXAMPLE
// ============================================================================
void teleopLoop() {
    double current_time = millis() / 1000.0;

    // Update all subsystems
    robot.update(current_time);

    // Read joystick inputs (pseudo-code)
    // double forward = getJoystickY();
    // double turn = getJoystickX();
    // bool intake_button = getButtonA();
    // bool arm_up = getButtonB();

    // Apply controls
    // robot.getDrive().arcade(forward, turn);

    // if (intake_button) {
    //     robot.getIntake().intake();
    // } else {
    //     robot.getIntake().stop();
    // }

    // if (arm_up) {
    //     robot.getArm().moveTo(units::deg(90));
    // }
}

#endif
