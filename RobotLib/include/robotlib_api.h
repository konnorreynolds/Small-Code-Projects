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
    // Configuration
    Radians min_angle_;
    Radians max_angle_;
    double speed_;
    bool has_limits_;

    // PID configuration
    PIDController pid_;
    double kP_, kI_, kD_;
    bool pid_enabled_;

    // Feedforward configuration
    double kS_;  // Static friction
    double kV_;  // Velocity feedforward
    bool ff_enabled_;

    // Command state (what we commanded)
    Radians target_position_;
    double duty_cycle_;

    // Measured state (sensor feedback)
    Radians current_position_;      // Actual measured position
    RadiansPerSecond current_velocity_;  // Actual measured velocity

public:
    // Constructor
    Arm()
        : min_angle_(Radians::fromRadians(-M_PI))
        , max_angle_(Radians::fromRadians(M_PI))
        , speed_(1.0)
        , has_limits_(false)
        , pid_(PIDController::Gains(1.0, 0.0, 0.0))
        , kP_(1.0), kI_(0.0), kD_(0.0)
        , pid_enabled_(false)
        , kS_(0.0), kV_(0.0)
        , ff_enabled_(false)
        , target_position_(Radians::fromRadians(0.0))
        , duty_cycle_(0.0)
        , current_position_(Radians::fromRadians(0.0))
        , current_velocity_(radps(0.0))
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
        // Calculate velocity from position change
        double prev_pos = current_position_.toRadians();
        current_position_ = Radians::fromRadians(measured_position.toRadians());
        double new_pos = current_position_.toRadians();
        current_velocity_ = RadiansPerSecond::fromRadiansPerSecond((new_pos - prev_pos) / dt);

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

    // Update with measured velocity (if available from sensor)
    template<typename AngleRatio, typename AngularVelocityRatio>
    Arm& update(double dt, const Angle<AngleRatio>& measured_position,
                const AngularVelocity<AngularVelocityRatio>& measured_velocity) {
        current_position_ = Radians::fromRadians(measured_position.toRadians());
        current_velocity_ = RadiansPerSecond::fromRadiansPerSecond(measured_velocity.toRadiansPerSecond());

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

    // Configuration
    Radians getMinAngle() const { return min_angle_; }
    Radians getMaxAngle() const { return max_angle_; }
    double getSpeed() const { return speed_; }
    bool hasLimits() const { return has_limits_; }

    // PID configuration
    double getKP() const { return kP_; }
    double getKI() const { return kI_; }
    double getKD() const { return kD_; }
    bool isPIDEnabled() const { return pid_enabled_; }

    // Feedforward configuration
    double getKS() const { return kS_; }
    double getKV() const { return kV_; }
    bool isFeedforwardEnabled() const { return ff_enabled_; }

    // Command state (what we commanded)
    Radians getTarget() const { return target_position_; }
    double getDutyCycle() const { return duty_cycle_; }

    // Measured state (sensor feedback)
    Radians getPosition() const { return current_position_; }
    RadiansPerSecond getVelocity() const { return current_velocity_; }

    // Calculated status
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
    // Configuration (robot hardware parameters)
    Meters wheelbase_;
    Meters wheel_diameter_;
    MetersPerSecond max_speed_;

    // Command state (what we commanded the robot to do)
    double left_duty_;
    double right_duty_;
    MetersPerSecond linear_velocity_;
    RadiansPerSecond angular_velocity_;

    // Odometry state (where the robot thinks it is)
    double x_position_;      // meters
    double y_position_;      // meters
    double theta_;           // radians (heading)

    // Measured state (sensor feedback)
    Meters left_distance_;   // Total distance traveled by left wheel
    Meters right_distance_;  // Total distance traveled by right wheel
    MetersPerSecond left_velocity_;   // Measured left wheel velocity
    MetersPerSecond right_velocity_;  // Measured right wheel velocity

public:
    DifferentialDrive()
        : wheelbase_(m(0.5))
        , wheel_diameter_(m(0.1))
        , max_speed_(mps(1.0))
        , left_duty_(0.0)
        , right_duty_(0.0)
        , linear_velocity_(mps(0.0))
        , angular_velocity_(radps(0.0))
        , x_position_(0.0)
        , y_position_(0.0)
        , theta_(0.0)
        , left_distance_(m(0.0))
        , right_distance_(m(0.0))
        , left_velocity_(mps(0.0))
        , right_velocity_(mps(0.0))
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
    // State Update (sensor feedback and odometry)
    // ========================================================================

    DifferentialDrive& updateEncoders(const Meters& left_dist, const Meters& right_dist) {
        left_distance_ = left_dist;
        right_distance_ = right_dist;
        return *this;
    }

    DifferentialDrive& updateVelocities(const MetersPerSecond& left_vel, const MetersPerSecond& right_vel) {
        left_velocity_ = left_vel;
        right_velocity_ = right_vel;
        return *this;
    }

    DifferentialDrive& updateOdometry(double dt) {
        // Calculate distance traveled by each wheel since last update
        double left_vel = left_velocity_.toMetersPerSecond();
        double right_vel = right_velocity_.toMetersPerSecond();

        double left_delta = left_vel * dt;
        double right_delta = right_vel * dt;

        // Calculate robot motion
        double distance = (left_delta + right_delta) / 2.0;
        double dtheta = (right_delta - left_delta) / wheelbase_.toMeters();

        // Update position
        theta_ += dtheta;
        x_position_ += distance * std::cos(theta_);
        y_position_ += distance * std::sin(theta_);

        return *this;
    }

    DifferentialDrive& resetOdometry(double x = 0.0, double y = 0.0, double theta = 0.0) {
        x_position_ = x;
        y_position_ = y;
        theta_ = theta;
        left_distance_ = m(0.0);
        right_distance_ = m(0.0);
        return *this;
    }

    // ========================================================================
    // Query
    // ========================================================================

    // Configuration (robot hardware parameters)
    Meters getWheelbase() const { return wheelbase_; }
    Meters getWheelDiameter() const { return wheel_diameter_; }
    MetersPerSecond getMaxSpeed() const { return max_speed_; }

    // Command state (what we commanded)
    double getLeftDuty() const { return left_duty_; }
    double getRightDuty() const { return right_duty_; }
    MetersPerSecond getLinearVelocity() const { return linear_velocity_; }
    RadiansPerSecond getAngularVelocity() const { return angular_velocity_; }

    // Odometry state (where the robot thinks it is)
    double getX() const { return x_position_; }
    double getY() const { return y_position_; }
    double getTheta() const { return theta_; }
    double getThetaDegrees() const { return theta_ * 180.0 / M_PI; }

    // Measured state (sensor feedback)
    Meters getLeftDistance() const { return left_distance_; }
    Meters getRightDistance() const { return right_distance_; }
    MetersPerSecond getLeftVelocity() const { return left_velocity_; }
    MetersPerSecond getRightVelocity() const { return right_velocity_; }

    // Status
    bool isStopped(double threshold = 0.01) const {
        return std::abs(left_duty_) < threshold && std::abs(right_duty_) < threshold;
    }
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

    // Current values
    double getValue() const { return filtered_value_; }
    double getRawValue() const { return raw_value_; }

    // Configuration
    bool isMovingAverageEnabled() const { return use_ma_; }
    bool isLowPassEnabled() const { return use_lpf_; }
    double getLowPassAlpha() const { return lpf_alpha_; }
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

    // Current values
    double getOutput() const { return output_; }
    double getError() const { return error_; }

    // Configuration
    double getKP() const { return kP_; }
    double getKI() const { return kI_; }
    double getKD() const { return kD_; }
};

} // namespace robotlib

#endif // ROBOTLIB_API_H
