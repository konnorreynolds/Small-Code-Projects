// ============================================================================
// RobotLib Fluent API - Easy-to-use chaining interface
// ============================================================================
// High-level fluent API that wraps RobotLib's type-safe units system.
// Every configuration method returns *this for chaining.
//
// Example usage:
//   Arm leftArm = Arm()
//       .withPID(1.0, 0.1, 0.05)
//       .withLimits(deg(-90), deg(90))
//       .withFeedforward(0.5, 0.002);
//
//   leftArm.moveTo(deg(90))
//          .setSpeed(0.8)
//          .waitUntilAtTarget();
//
// Part of RobotLib v2.2
// C++11 compatible, header-only, chainable API
// ============================================================================

#ifndef ROBOTLIB_API_H
#define ROBOTLIB_API_H

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
#include <cmath>

namespace robotlib {

using namespace units;
using namespace units::robotics;

// ============================================================================
// Motor/Arm Controller - Fluent API
// ============================================================================
class Arm {
private:
    // State
    Radians current_position_;
    Radians target_position_;
    Radians min_angle_;
    Radians max_angle_;
    double speed_;
    double duty_cycle_;
    bool has_limits_;

    // PID
    PIDController pid_;
    double kP_, kI_, kD_;
    bool pid_enabled_;

    // Feedforward
    double kS_;  // Static friction
    double kV_;  // Velocity feedforward
    bool ff_enabled_;

public:
    // Constructor
    Arm()
        : current_position_(Radians::fromRadians(0.0))
        , target_position_(Radians::fromRadians(0.0))
        , min_angle_(Radians::fromRadians(-M_PI))
        , max_angle_(Radians::fromRadians(M_PI))
        , speed_(1.0)
        , duty_cycle_(0.0)
        , has_limits_(false)
        , pid_(PIDController::Gains(1.0, 0.0, 0.0))
        , kP_(1.0), kI_(0.0), kD_(0.0)
        , pid_enabled_(false)
        , kS_(0.0), kV_(0.0)
        , ff_enabled_(false)
    {}

    // ========================================================================
    // Configuration Methods (return *this for chaining)
    // ========================================================================

    Arm& withPID(double kP, double kI, double kD) {
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        PIDController::Gains gains(kP, kI, kD);
        gains.outputMin = -1.0;
        gains.outputMax = 1.0;
        pid_ = PIDController(gains);
        pid_enabled_ = true;

        return *this;
    }

    Arm& withExponentialPID(double kP, double kI, double kD, double exponent = 2.0) {
        (void)exponent;  // Reserved for future exponential control implementation
        withPID(kP, kI, kD);
        return *this;
    }

    Arm& withFeedforward(double kS, double kV) {
        kS_ = kS;
        kV_ = kV;
        ff_enabled_ = true;
        return *this;
    }

    template<typename AngleRatio>
    Arm& withLimits(const Angle<AngleRatio>& min_angle, const Angle<AngleRatio>& max_angle) {
        min_angle_ = Radians::fromRadians(min_angle.toRadians());
        max_angle_ = Radians::fromRadians(max_angle.toRadians());
        has_limits_ = true;
        return *this;
    }

    Arm& withoutLimits() {
        has_limits_ = false;
        return *this;
    }

    Arm& enablePID(bool enable = true) {
        pid_enabled_ = enable;
        return *this;
    }

    Arm& disablePID() {
        pid_enabled_ = false;
        return *this;
    }

    // ========================================================================
    // Control Methods (return *this for chaining where sensible)
    // ========================================================================

    template<typename AngleRatio>
    Arm& moveTo(const Angle<AngleRatio>& target) {
        target_position_ = Radians::fromRadians(target.toRadians());

        // Clamp to limits if enabled
        if (has_limits_) {
            if (target_position_.toRadians() < min_angle_.toRadians()) {
                target_position_ = min_angle_;
            }
            if (target_position_.toRadians() > max_angle_.toRadians()) {
                target_position_ = max_angle_;
            }
        }

        return *this;
    }

    template<typename AngleRatio>
    Arm& moveBy(const Angle<AngleRatio>& delta) {
        return moveTo(Radians::fromRadians(
            current_position_.toRadians() + delta.toRadians()
        ));
    }

    Arm& setSpeed(double speed) {
        speed_ = speed;
        if (speed_ < 0.0) speed_ = 0.0;
        if (speed_ > 1.0) speed_ = 1.0;
        return *this;
    }

    Arm& setDutyCycle(double duty) {
        duty_cycle_ = duty;
        if (duty_cycle_ < -1.0) duty_cycle_ = -1.0;
        if (duty_cycle_ > 1.0) duty_cycle_ = 1.0;
        return *this;
    }

    Arm& stop() {
        duty_cycle_ = 0.0;
        target_position_ = current_position_;
        return *this;
    }

    Arm& brake() {
        return stop();
    }

    template<typename AngleRatio>
    Arm& resetPosition(const Angle<AngleRatio>& position) {
        current_position_ = Radians::fromRadians(position.toRadians());
        target_position_ = Radians::fromRadians(position.toRadians());
        return *this;
    }

    template<typename AngleRatio>
    Arm& update(double dt, const Angle<AngleRatio>& measured_position) {
        current_position_ = Radians::fromRadians(measured_position.toRadians());

        if (pid_enabled_) {
            double error = target_position_.toRadians() - current_position_.toRadians();
            duty_cycle_ = pid_.calculate(error, dt) * speed_;

            // Add feedforward if enabled
            if (ff_enabled_) {
                double velocity = error / dt;  // Simplified
                duty_cycle_ += kS_ * (error > 0 ? 1.0 : -1.0) + kV_ * velocity;
            }
        }

        // Clamp output
        if (duty_cycle_ < -1.0) duty_cycle_ = -1.0;
        if (duty_cycle_ > 1.0) duty_cycle_ = 1.0;

        return *this;
    }

    // ========================================================================
    // Query Methods (return values, end chain)
    // ========================================================================

    double getDutyCycle() const { return duty_cycle_; }

    Radians getPosition() const { return current_position_; }

    Radians getTarget() const { return target_position_; }

    double getError() const {
        return target_position_.toRadians() - current_position_.toRadians();
    }

    bool isAtTarget(double tolerance_deg = 2.0) const {
        double tolerance_rad = tolerance_deg * M_PI / 180.0;
        return std::abs(getError()) < tolerance_rad;
    }

    bool isMoving(double threshold = 0.01) const {
        return std::abs(duty_cycle_) > threshold;
    }
};

// ============================================================================
// Differential Drive - Fluent API
// ============================================================================
class DifferentialDrive {
private:
    Meters wheelbase_;
    Meters wheel_diameter_;
    MetersPerSecond max_speed_;
    double left_duty_;
    double right_duty_;

    // Current velocity
    MetersPerSecond linear_velocity_;
    RadiansPerSecond angular_velocity_;

public:
    DifferentialDrive()
        : wheelbase_(m(0.5))
        , wheel_diameter_(m(0.1))
        , max_speed_(mps(1.0))
        , left_duty_(0.0)
        , right_duty_(0.0)
        , linear_velocity_(mps(0.0))
        , angular_velocity_(radps(0.0))
    {}

    // ========================================================================
    // Configuration
    // ========================================================================

    DifferentialDrive& withWheelbase(const Meters& wheelbase) {
        wheelbase_ = wheelbase;
        return *this;
    }

    DifferentialDrive& withWheelDiameter(const Meters& diameter) {
        wheel_diameter_ = diameter;
        return *this;
    }

    DifferentialDrive& withMaxSpeed(const MetersPerSecond& max_speed) {
        max_speed_ = max_speed;
        return *this;
    }

    // ========================================================================
    // Control
    // ========================================================================

    DifferentialDrive& drive(const MetersPerSecond& linear, const RadiansPerSecond& angular) {
        linear_velocity_ = linear;
        angular_velocity_ = angular;

        // Convert to wheel velocities
        double v = linear.toMetersPerSecond();
        double w = angular.toRadiansPerSecond();
        double L = wheelbase_.toMeters();

        double left_vel = v - (w * L / 2.0);
        double right_vel = v + (w * L / 2.0);

        // Convert to duty cycles (-1.0 to 1.0)
        left_duty_ = left_vel / max_speed_.toMetersPerSecond();
        right_duty_ = right_vel / max_speed_.toMetersPerSecond();

        // Clamp
        if (left_duty_ < -1.0) left_duty_ = -1.0;
        if (left_duty_ > 1.0) left_duty_ = 1.0;
        if (right_duty_ < -1.0) right_duty_ = -1.0;
        if (right_duty_ > 1.0) right_duty_ = 1.0;

        return *this;
    }

    DifferentialDrive& arcade(double forward, double turn) {
        // Arcade drive: forward/backward + turn
        return drive(
            MetersPerSecond::fromMetersPerSecond(forward * max_speed_.toMetersPerSecond()),
            RadiansPerSecond::fromRadiansPerSecond(turn * 2.0)  // Scale turning
        );
    }

    DifferentialDrive& tank(double left, double right) {
        left_duty_ = left;
        right_duty_ = right;

        // Clamp
        if (left_duty_ < -1.0) left_duty_ = -1.0;
        if (left_duty_ > 1.0) left_duty_ = 1.0;
        if (right_duty_ < -1.0) right_duty_ = -1.0;
        if (right_duty_ > 1.0) right_duty_ = 1.0;

        return *this;
    }

    DifferentialDrive& stop() {
        left_duty_ = 0.0;
        right_duty_ = 0.0;
        linear_velocity_ = mps(0.0);
        angular_velocity_ = radps(0.0);
        return *this;
    }

    DifferentialDrive& brake() {
        return stop();
    }

    // ========================================================================
    // Query
    // ========================================================================

    double getLeftDuty() const { return left_duty_; }
    double getRightDuty() const { return right_duty_; }

    MetersPerSecond getLinearVelocity() const { return linear_velocity_; }
    RadiansPerSecond getAngularVelocity() const { return angular_velocity_; }
};

// ============================================================================
// Sensor with Filtering - Fluent API
// ============================================================================
class Sensor {
private:
    double raw_value_;
    double filtered_value_;
    MovingAverageFilter<5> ma_filter_;
    LowPassFilter lpf_;
    bool use_ma_;
    bool use_lpf_;
    double lpf_alpha_;

public:
    Sensor()
        : raw_value_(0.0)
        , filtered_value_(0.0)
        , lpf_(0.5)
        , use_ma_(false)
        , use_lpf_(false)
        , lpf_alpha_(0.5)
    {}

    // ========================================================================
    // Configuration
    // ========================================================================

    Sensor& withMovingAverage(bool enable = true) {
        use_ma_ = enable;
        return *this;
    }

    Sensor& withLowPassFilter(double alpha) {
        lpf_alpha_ = alpha;
        lpf_ = LowPassFilter(alpha);
        use_lpf_ = true;
        return *this;
    }

    Sensor& withoutFiltering() {
        use_ma_ = false;
        use_lpf_ = false;
        return *this;
    }

    // ========================================================================
    // Control/Update
    // ========================================================================

    Sensor& read(double value) {
        raw_value_ = value;
        filtered_value_ = raw_value_;

        if (use_ma_) {
            filtered_value_ = ma_filter_.update(filtered_value_);
        }

        if (use_lpf_) {
            filtered_value_ = lpf_.update(filtered_value_);
        }

        return *this;
    }

    Sensor& reset() {
        raw_value_ = 0.0;
        filtered_value_ = 0.0;
        return *this;
    }

    // ========================================================================
    // Query
    // ========================================================================

    double getValue() const { return filtered_value_; }
    double getRawValue() const { return raw_value_; }
};

// ============================================================================
// PID Controller Wrapper - Fluent API
// ============================================================================
class PID {
private:
    PIDController controller_;
    double kP_, kI_, kD_;
    double output_;
    double error_;

public:
    PID()
        : controller_(PIDController::Gains(1.0, 0.0, 0.0))
        , kP_(1.0), kI_(0.0), kD_(0.0)
        , output_(0.0)
        , error_(0.0)
    {}

    // ========================================================================
    // Configuration
    // ========================================================================

    PID& withGains(double kP, double kI, double kD) {
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        PIDController::Gains gains(kP, kI, kD);
        controller_ = PIDController(gains);

        return *this;
    }

    PID& withOutputLimits(double min, double max) {
        PIDController::Gains gains(kP_, kI_, kD_);
        gains.outputMin = min;
        gains.outputMax = max;
        controller_ = PIDController(gains);
        return *this;
    }

    PID& withIntegralLimit(double max) {
        PIDController::Gains gains(kP_, kI_, kD_);
        gains.iMax = max;
        controller_ = PIDController(gains);
        return *this;
    }

    // ========================================================================
    // Control
    // ========================================================================

    PID& calculate(double error, double dt) {
        error_ = error;
        output_ = controller_.calculate(error, dt);
        return *this;
    }

    PID& reset() {
        controller_.reset();
        output_ = 0.0;
        error_ = 0.0;
        return *this;
    }

    // ========================================================================
    // Query
    // ========================================================================

    double getOutput() const { return output_; }
    double getError() const { return error_; }
};

} // namespace robotlib

#endif // ROBOTLIB_API_H
