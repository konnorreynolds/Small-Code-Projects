// ============================================================================
// EXAMPLE: Custom Motor Driver Header
// ============================================================================
// This example shows how to create a custom motor driver that wraps
// hardware-specific motor controller code while using RobotLib's type-safe
// units system.
//
// USE CASE:
//   - Wrapping vendor motor controller libraries
//   - Interfacing with CAN bus motor controllers
//   - Creating hardware abstraction layers
//   - Supporting different motor controller chips
//
// PATTERN:
//   1. Include RobotLib headers for units and utilities
//   2. Define your motor driver class
//   3. Store configuration (gear ratio, limits, etc.)
//   4. Provide type-safe interface using RobotLib units
//   5. Translate to hardware-specific commands internally
//
// ============================================================================

#ifndef CUSTOM_MOTOR_DRIVER_H
#define CUSTOM_MOTOR_DRIVER_H

#include "../../include/units_core.h"
#include "../../include/units_physics.h"

// ============================================================================
// EXAMPLE 1: Simple PWM Motor Driver
// ============================================================================
// Wraps basic PWM motor control (like L298N, TB6612, etc.)
// ============================================================================
class SimplePWMMotor {
private:
    // ========================================================================
    // Hardware Configuration
    // ========================================================================
    int pwm_pin_;        // PWM pin for speed control (0-255)
    int dir_pin_;        // Direction pin (HIGH/LOW)
    bool inverted_;      // Reverse direction if true

    // ========================================================================
    // Motor Characteristics (type-safe units!)
    // ========================================================================
    units::RPM max_rpm_;           // Maximum motor speed
    double gear_ratio_;            // Gear reduction (e.g., 20:1 = 20.0)
    units::Newtons stall_torque_;  // Stall torque at 100% duty cycle

    // ========================================================================
    // Current State
    // ========================================================================
    double current_duty_cycle_;    // -1.0 to 1.0

public:
    // ========================================================================
    // Constructor
    // ========================================================================
    // PARAMETERS:
    //   pwm_pin: Arduino PWM pin (must support analogWrite)
    //   dir_pin: Digital pin for direction
    //   max_rpm: Motor's free speed in RPM
    //   gear_ratio: Gearbox reduction (output/input)
    //
    SimplePWMMotor(int pwm_pin, int dir_pin,
                   const units::RPM& max_rpm,
                   double gear_ratio = 1.0)
        : pwm_pin_(pwm_pin)
        , dir_pin_(dir_pin)
        , inverted_(false)
        , max_rpm_(max_rpm)
        , gear_ratio_(gear_ratio)
        , stall_torque_(units::Newtons::fromNewtons(1.0))
        , current_duty_cycle_(0.0)
    {
        // Hardware initialization would go here
        // pinMode(pwm_pin_, OUTPUT);
        // pinMode(dir_pin_, OUTPUT);
    }

    // ========================================================================
    // SET DUTY CYCLE - Basic control interface
    // ========================================================================
    // INPUT: duty_cycle ∈ [-1.0, 1.0]
    //   -1.0 = full reverse
    //    0.0 = stopped
    //   +1.0 = full forward
    //
    // WHAT THIS DOES:
    //   1. Clamps duty cycle to safe range
    //   2. Determines direction from sign
    //   3. Converts to PWM value (0-255)
    //   4. Writes to hardware pins
    //
    void setDutyCycle(double duty_cycle) {
        // Safety: clamp to valid range
        duty_cycle = units::numerical::clamp(duty_cycle, -1.0, 1.0);
        current_duty_cycle_ = duty_cycle;

        // Determine direction
        bool forward = duty_cycle >= 0;
        if (inverted_) forward = !forward;

        // Calculate PWM value (0-255)
        int pwm_value = static_cast<int>(std::abs(duty_cycle) * 255);

        // ====================================================================
        // HARDWARE-SPECIFIC CODE GOES HERE
        // ====================================================================
        // digitalWrite(dir_pin_, forward ? HIGH : LOW);
        // analogWrite(pwm_pin_, pwm_value);
    }

    // ========================================================================
    // SET VELOCITY - Type-safe velocity control
    // ========================================================================
    // INPUT: desired_velocity in type-safe units (RPM, RadiansPerSecond, etc.)
    //
    // WHAT THIS DOES:
    //   1. Converts desired velocity to duty cycle
    //   2. Accounts for gear ratio
    //   3. Calls setDutyCycle()
    //
    // EXAMPLE:
    //   motor.setVelocity(units::rpm(100));  // Run at 100 RPM
    //
    void setVelocity(const units::RPM& velocity) {
        // Calculate duty cycle needed for this velocity
        // duty = (desired_rpm / max_rpm) * gear_ratio
        double duty = velocity.toRPM() / max_rpm_.toRPM();
        setDutyCycle(duty);
    }

    // ========================================================================
    // SET POSITION - For position control (requires encoder feedback)
    // ========================================================================
    // NOTE: This is a simplified example. Real position control needs:
    //   - Encoder feedback
    //   - PID controller
    //   - Regular update() calls
    //
    void setTargetPosition(const units::Radians& target) {
        // In real implementation, you would:
        // 1. Read current position from encoder
        // 2. Calculate error: target - current
        // 3. Run PID controller: duty = pid.calculate(error, dt)
        // 4. Call setDutyCycle(duty)

        // Placeholder for example
        (void)target;
    }

    // ========================================================================
    // GETTERS - Query motor state
    // ========================================================================
    double getDutyCycle() const { return current_duty_cycle_; }
    units::RPM getMaxRPM() const { return max_rpm_; }
    double getGearRatio() const { return gear_ratio_; }

    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    void setInverted(bool inverted) { inverted_ = inverted; }
    void setStallTorque(const units::Newtons& torque) { stall_torque_ = torque; }

    // ========================================================================
    // STOP - Emergency stop
    // ========================================================================
    void stop() {
        setDutyCycle(0.0);
    }
};


// ============================================================================
// EXAMPLE 2: CAN Bus Motor Controller (e.g., SparkMax, TalonSRX, CTRE)
// ============================================================================
// Shows how to wrap CAN-based motor controllers
// ============================================================================
class CANMotorController {
private:
    int can_id_;                    // CAN bus device ID
    units::Radians current_position_;
    units::RadiansPerSecond current_velocity_;

    // Vendor-specific controller object would go here
    // e.g., CANSparkMax* motor_controller_;

public:
    // ========================================================================
    // Constructor
    // ========================================================================
    CANMotorController(int can_id)
        : can_id_(can_id)
        , current_position_(units::Radians::fromRadians(0))
        , current_velocity_(units::RadiansPerSecond::fromRadiansPerSecond(0))
    {
        // Initialize CAN motor controller
        // motor_controller_ = new CANSparkMax(can_id, MotorType::kBrushless);
    }

    // ========================================================================
    // CONTROL MODES
    // ========================================================================

    // Duty Cycle Mode (Percent Output)
    void setDutyCycle(double duty_cycle) {
        duty_cycle = units::numerical::clamp(duty_cycle, -1.0, 1.0);
        // motor_controller_->Set(duty_cycle);
    }

    // Velocity Mode (requires closed-loop control on motor controller)
    void setVelocity(const units::RadiansPerSecond& velocity) {
        double rpm = velocity.toRPM();
        // motor_controller_->Set(ControlMode::Velocity, rpm);
    }

    // Position Mode (requires closed-loop control)
    void setPosition(const units::Radians& position) {
        double rotations = position.toRadians() / (2.0 * units::constants::PI);
        // motor_controller_->Set(ControlMode::Position, rotations);
    }

    // ========================================================================
    // FEEDBACK - Read encoder values
    // ========================================================================
    units::Radians getPosition() const {
        // double rotations = motor_controller_->GetEncoder().GetPosition();
        // return units::Radians::fromRadians(rotations * 2.0 * units::constants::PI);
        return current_position_;
    }

    units::RadiansPerSecond getVelocity() const {
        // double rpm = motor_controller_->GetEncoder().GetVelocity();
        // return units::RPM::fromRPM(rpm).toRadiansPerSecond();
        return current_velocity_;
    }

    // ========================================================================
    // PID CONFIGURATION
    // ========================================================================
    void configurePID(double kP, double kI, double kD) {
        // motor_controller_->GetPIDController().SetP(kP);
        // motor_controller_->GetPIDController().SetI(kI);
        // motor_controller_->GetPIDController().SetD(kD);
    }

    // ========================================================================
    // CURRENT LIMITING (protect motor from burning out)
    // ========================================================================
    void setCurrentLimit(const units::Amperes& limit) {
        // motor_controller_->SetSmartCurrentLimit(limit.toAmperes());
    }
};


// ============================================================================
// EXAMPLE 3: Servo Motor Driver
// ============================================================================
// For hobby servos (PWM pulse width 1000-2000μs)
// ============================================================================
class ServoMotor {
private:
    int pin_;
    units::Degrees min_angle_;
    units::Degrees max_angle_;
    units::Degrees current_angle_;

public:
    ServoMotor(int pin,
               const units::Degrees& min_angle = units::deg(0),
               const units::Degrees& max_angle = units::deg(180))
        : pin_(pin)
        , min_angle_(min_angle)
        , max_angle_(max_angle)
        , current_angle_(units::deg(90))
    {
        // Initialize servo
        // servo_.attach(pin_);
    }

    // ========================================================================
    // SET ANGLE - Move to specific angle
    // ========================================================================
    // INPUT: angle in type-safe units (Degrees or Radians)
    //
    // TEMPLATE: Accepts both Degrees and Radians automatically!
    //
    template<typename AngleRatio>
    void setAngle(const units::Angle<AngleRatio>& angle) {
        // Convert to degrees and clamp to limits
        double degrees = angle.toDegrees();
        degrees = units::numerical::clamp(degrees,
                                         min_angle_.toDegrees(),
                                         max_angle_.toDegrees());

        current_angle_ = units::Degrees::fromDegrees(degrees);

        // Write to servo
        // servo_.write(degrees);
    }

    // ========================================================================
    // SET NORMALIZED POSITION - Control with 0.0 to 1.0
    // ========================================================================
    // INPUT: position ∈ [0.0, 1.0]
    //   0.0 = min_angle
    //   0.5 = middle
    //   1.0 = max_angle
    //
    void setNormalizedPosition(double position) {
        position = units::numerical::clamp(position, 0.0, 1.0);

        // Linear interpolation between min and max
        double degrees = min_angle_.toDegrees() +
                        position * (max_angle_.toDegrees() - min_angle_.toDegrees());

        setAngle(units::deg(degrees));
    }

    units::Degrees getAngle() const { return current_angle_; }
};

#endif // CUSTOM_MOTOR_DRIVER_H

// ============================================================================
// USAGE EXAMPLES
// ============================================================================
#if 0  // Set to 1 to include usage examples in compilation

int main() {
    using namespace units::literals;

    // ------------------------------------------------------------------------
    // Example 1: PWM Motor
    // ------------------------------------------------------------------------
    SimplePWMMotor drive_motor(9, 8,    // PWM pin 9, DIR pin 8
                               rpm(200),  // Max 200 RPM
                               20.0);     // 20:1 gear ratio

    drive_motor.setDutyCycle(0.5);       // 50% forward
    drive_motor.setVelocity(rpm(100));   // Run at 100 RPM
    drive_motor.stop();                  // Emergency stop

    // ------------------------------------------------------------------------
    // Example 2: CAN Motor
    // ------------------------------------------------------------------------
    CANMotorController arm_motor(5);     // CAN ID 5

    arm_motor.configurePID(0.1, 0.01, 0.05);
    arm_motor.setPosition(rad(1.57));    // Move to 90 degrees
    arm_motor.setCurrentLimit(amp(20));  // Limit to 20A

    auto current_pos = arm_motor.getPosition();
    auto current_vel = arm_motor.getVelocity();

    // ------------------------------------------------------------------------
    // Example 3: Servo
    // ------------------------------------------------------------------------
    ServoMotor gripper(10,               // Pin 10
                       deg(0),            // Min 0°
                       deg(180));         // Max 180°

    gripper.setAngle(deg(90));           // Center position
    gripper.setAngle(rad(1.57));         // Also 90° (using radians!)
    gripper.setNormalizedPosition(0.5);  // 50% = center

    return 0;
}

#endif
