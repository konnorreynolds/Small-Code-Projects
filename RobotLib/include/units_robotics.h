// ============================================================================
// units_robotics.h - Robotics Control Systems and Components
// ============================================================================
// Purpose: Control systems, filters, and robotics-specific components
// Dependencies: units_core.h, units_physics.h
//
// This file contains:
// - 2D vectors and poses (SE(2) group)
// - PID and feedforward controllers
// - Motion profiles (trapezoid, S-curve)
// - Digital filters (low-pass, Kalman, complementary)
// - Path following algorithms
// ============================================================================

#ifndef ROBOTICS_UNITS_ROBOTICS_H
#define ROBOTICS_UNITS_ROBOTICS_H

#include "units_core.h"
#include "units_physics.h"
#include <array>
#include <vector>
#include <algorithm>
#include <functional>

namespace units {
namespace robotics {

// ============================================================================
// 2D VECTOR
// ============================================================================
// Why a custom Vec2D instead of std::pair or std::array:
// - Domain-specific operations (rotate, project, etc.)
// - Clear semantic meaning (x, y not first, second)
// - Efficient implementations of robotics operations
// ============================================================================
class Vec2D {
public:
    double x, y;

    // Constructors
    constexpr Vec2D() : x(0), y(0) {}
    constexpr Vec2D(double x, double y) : x(x), y(y) {}

    // Create from polar coordinates
    static UNITS_CONSTEXPR14 Vec2D fromPolar(double r, const Radians& theta) {
        return Vec2D(r * theta.cos(), r * theta.sin());
    }

    // Magnitude and direction
    UNITS_CONSTEXPR14 double magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    constexpr double magnitudeSquared() const {
        return x * x + y * y;
    }

    UNITS_CONSTEXPR14 Vec2D normalized() const {
        double mag = magnitude();
        return numerical::isZero(mag) ? Vec2D() : Vec2D(x / mag, y / mag);
    }

    UNITS_CONSTEXPR14 Radians angle() const {
        return Radians::atan2(y, x);
    }

    // Vector operations
    constexpr double dot(const Vec2D& other) const {
        return x * other.x + y * other.y;
    }

    constexpr double cross(const Vec2D& other) const {
        return x * other.y - y * other.x;  // 2D cross product (scalar)
    }

    UNITS_CONSTEXPR14 Vec2D project(const Vec2D& onto) const {
        double d = onto.dot(onto);
        return numerical::isZero(d) ? Vec2D() : onto * (this->dot(onto) / d);
    }

    UNITS_CONSTEXPR14 Vec2D reflect(const Vec2D& normal) const {
        Vec2D n = normal.normalized();
        return *this - n * (2.0 * this->dot(n));
    }

    // Rotation
    UNITS_CONSTEXPR14 Vec2D rotate(const Radians& angle) const {
        double c = angle.cos();
        double s = angle.sin();
        return Vec2D(x * c - y * s, x * s + y * c);
    }

    // Distance operations
    UNITS_CONSTEXPR14 double distanceTo(const Vec2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }

    constexpr double distanceSquaredTo(const Vec2D& other) const {
        return (other.x - x) * (other.x - x) + (other.y - y) * (other.y - y);
    }

    UNITS_CONSTEXPR14 Radians angleTo(const Vec2D& other) const {
        return Radians::atan2(other.y - y, other.x - x);
    }

    // Interpolation
    constexpr Vec2D lerp(const Vec2D& target, double t) const {
        return Vec2D(x + (target.x - x) * t, y + (target.y - y) * t);
    }

    // Operators
    constexpr Vec2D operator+(const Vec2D& other) const {
        return Vec2D(x + other.x, y + other.y);
    }
    constexpr Vec2D operator-(const Vec2D& other) const {
        return Vec2D(x - other.x, y - other.y);
    }
    constexpr Vec2D operator*(double scalar) const {
        return Vec2D(x * scalar, y * scalar);
    }
    constexpr Vec2D operator/(double scalar) const {
        return Vec2D(numerical::safeDivide(x, scalar),
                    numerical::safeDivide(y, scalar));
    }
    constexpr Vec2D operator-() const { return Vec2D(-x, -y); }

    Vec2D& operator+=(const Vec2D& other) { x += other.x; y += other.y; return *this; }
    Vec2D& operator-=(const Vec2D& other) { x -= other.x; y -= other.y; return *this; }
    Vec2D& operator*=(double scalar) { x *= scalar; y *= scalar; return *this; }
    Vec2D& operator/=(double scalar) {
        x = numerical::safeDivide(x, scalar);
        y = numerical::safeDivide(y, scalar);
        return *this;
    }

    constexpr bool operator==(const Vec2D& other) const {
        return numerical::approxEqual(x, other.x) &&
               numerical::approxEqual(y, other.y);
    }
    constexpr bool operator!=(const Vec2D& other) const {
        return !(*this == other);
    }
};

// Scalar multiplication from left
inline constexpr Vec2D operator*(double scalar, const Vec2D& vec) {
    return vec * scalar;
}

// ============================================================================
// POSE2D (Position + Orientation)
// ============================================================================
// Represents SE(2) - Special Euclidean group in 2D
// Used for robot localization and coordinate transformations
// ============================================================================
class Pose2D {
public:
    Vec2D position;
    Radians theta;

    // Constructors
    constexpr Pose2D() : position(), theta() {}
    constexpr Pose2D(double x, double y, const Radians& theta)
        : position(x, y), theta(theta) {}
    constexpr Pose2D(const Vec2D& pos, const Radians& theta)
        : position(pos), theta(theta) {}

    // Template constructors to accept any Angle type
    template<typename AngleRatio>
    constexpr Pose2D(double x, double y, const Angle<AngleRatio>& angle)
        : position(x, y), theta(Radians::fromRadians(angle.toRadians())) {}

    template<typename AngleRatio>
    constexpr Pose2D(const Vec2D& pos, const Angle<AngleRatio>& angle)
        : position(pos), theta(Radians::fromRadians(angle.toRadians())) {}

    // Transform point from local to global frame
    UNITS_CONSTEXPR14 Vec2D toGlobal(const Vec2D& local) const {
        double c = theta.cos();
        double s = theta.sin();
        return Vec2D(
            position.x + local.x * c - local.y * s,
            position.y + local.x * s + local.y * c
        );
    }

    // Transform point from global to local frame
    UNITS_CONSTEXPR14 Vec2D toLocal(const Vec2D& global) const {
        Vec2D diff = global - position;
        double c = theta.cos();
        double s = theta.sin();
        // Inverse rotation (transpose of rotation matrix)
        return Vec2D(diff.x * c + diff.y * s, -diff.x * s + diff.y * c);
    }

    // Compose transformations (this * other)
    UNITS_CONSTEXPR14 Pose2D operator*(const Pose2D& other) const {
        return Pose2D(toGlobal(other.position), theta + other.theta);
    }

    // Inverse transformation
    UNITS_CONSTEXPR14 Pose2D inverse() const {
        Radians inv_theta = -theta;
        double c = inv_theta.cos();
        double s = inv_theta.sin();
        return Pose2D(
            -(position.x * c + position.y * s),
            -(-position.x * s + position.y * c),
            inv_theta
        );
    }

    // Relative pose from this to target
    UNITS_CONSTEXPR14 Pose2D relativeTo(const Pose2D& target) const {
        return inverse() * target;
    }

    // Distance to another pose (position only)
    UNITS_CONSTEXPR14 double distanceTo(const Pose2D& other) const {
        return position.distanceTo(other.position);
    }

    // Interpolation
    UNITS_CONSTEXPR14 Pose2D lerp(const Pose2D& target, double t) const {
        Radians angle_diff = (target.theta - theta).normalizeSigned();
        return Pose2D(
            position.lerp(target.position, t),
            theta + angle_diff * t
        );
    }
};

// ============================================================================
// PID CONTROLLER
// ============================================================================
// Why this implementation:
// - Derivative filtering reduces noise amplification
// - Anti-windup prevents integral accumulation during saturation
// - Feedforward improves tracking performance
// - Separated gains structure for easy tuning
// ============================================================================
class PIDController {
public:
    struct Gains {
        double kP;       // Proportional gain
        double kI;       // Integral gain
        double kD;       // Derivative gain
        double kF;       // Feedforward gain
        double iMax;     // Integral windup limit
        double outputMin, outputMax;  // Output saturation limits

        Gains() : kP(0), kI(0), kD(0), kF(0),
                 iMax(std::numeric_limits<double>::max()),
                 outputMin(-std::numeric_limits<double>::max()),
                 outputMax(std::numeric_limits<double>::max()) {}

        Gains(double p, double i, double d)
            : kP(p), kI(i), kD(d), kF(0),
              iMax(std::numeric_limits<double>::max()),
              outputMin(-std::numeric_limits<double>::max()),
              outputMax(std::numeric_limits<double>::max()) {}
    };

private:
    Gains gains_;
    double prev_error_;
    double integral_;
    double prev_derivative_;
    double derivative_alpha_;  // Low-pass filter coefficient for derivative
    bool initialized_;

public:
    explicit PIDController(const Gains& gains, double derivative_filter = 0.9)
        : gains_(gains), prev_error_(0), integral_(0), prev_derivative_(0),
          derivative_alpha_(derivative_filter), initialized_(false) {}

    PIDController(double kP, double kI, double kD)
        : PIDController(Gains(kP, kI, kD)) {}

    double calculate(double error, double dt, double feedforward = 0) {
        if (!initialized_) {
            prev_error_ = error;
            prev_derivative_ = 0;
            initialized_ = true;
        }

        // Proportional term
        double p = gains_.kP * error;

        // Integral term with clamping
        integral_ += error * dt;
        integral_ = numerical::clamp(integral_, -gains_.iMax, gains_.iMax);
        double i = gains_.kI * integral_;

        // Derivative term with filtering
        double raw_derivative = numerical::safeDivide(error - prev_error_, dt);
        double derivative = derivative_alpha_ * prev_derivative_ +
                          (1.0 - derivative_alpha_) * raw_derivative;
        double d = gains_.kD * derivative;

        prev_error_ = error;
        prev_derivative_ = derivative;

        // Calculate output with feedforward
        double output = p + i + d + gains_.kF * feedforward;

        // Apply output saturation
        double saturated_output = numerical::clamp(output, gains_.outputMin, gains_.outputMax);

        // Anti-windup: reduce integral if output is saturated
        if (output != saturated_output && gains_.kI != 0) {
            // Back-calculation anti-windup
            double excess = output - saturated_output;
            integral_ -= excess / gains_.kI * 0.1;  // Gradual reduction
        }

        return saturated_output;
    }

    void reset() {
        prev_error_ = 0;
        integral_ = 0;
        prev_derivative_ = 0;
        initialized_ = false;
    }

    // Getters and setters
    void setGains(const Gains& gains) { gains_ = gains; }
    const Gains& getGains() const { return gains_; }
    double getIntegral() const { return integral_; }
    void setIntegralLimit(double limit) { gains_.iMax = limit; }
    void setOutputLimits(double min, double max) {
        gains_.outputMin = min;
        gains_.outputMax = max;
    }
};

// ============================================================================
// FEEDFORWARD CONTROLLER
// ============================================================================
// Model-based controller for improved tracking
// Compensates for known system dynamics
// ============================================================================
class FeedforwardController {
public:
    struct Model {
        double kS;  // Static friction compensation
        double kV;  // Velocity feedforward
        double kA;  // Acceleration feedforward
        double kG;  // Gravity compensation

        Model() : kS(0), kV(0), kA(0), kG(0) {}
        Model(double s, double v, double a = 0, double g = 0)
            : kS(s), kV(v), kA(a), kG(g) {}
    };

private:
    Model model_;

public:
    explicit FeedforwardController(const Model& model) : model_(model) {}

    FeedforwardController(double kS, double kV, double kA = 0, double kG = 0)
        : model_(kS, kV, kA, kG) {}

    double calculate(double velocity, double acceleration = 0,
                    const Radians& angle = Radians()) const {
        double sign = (velocity > 0) ? 1.0 : (velocity < 0 ? -1.0 : 0.0);
        return model_.kS * sign +
               model_.kV * velocity +
               model_.kA * acceleration +
               model_.kG * angle.cos();  // Gravity term for arms/elevators
    }

    void setModel(const Model& model) { model_ = model; }
    const Model& getModel() const { return model_; }
};

// ============================================================================
// TRAPEZOID PROFILE
// ============================================================================
// Generates smooth motion profiles with acceleration limits
// Prevents mechanical stress and improves tracking
// ============================================================================
class TrapezoidProfile {
public:
    struct Constraints {
        double maxVelocity;
        double maxAcceleration;

        Constraints(double v, double a)
            : maxVelocity(std::abs(v)), maxAcceleration(std::abs(a)) {}
    };

    struct State {
        double position;
        double velocity;
        double acceleration;

        State(double p = 0, double v = 0, double a = 0)
            : position(p), velocity(v), acceleration(a) {}
    };

private:
    Constraints constraints_;
    State initial_;
    State goal_;
    double end_time_;
    double accel_time_;
    double cruise_time_;

    void calculate() {
        double distance = std::abs(goal_.position - initial_.position);

        // Calculate time to reach max velocity
        double accel_dist = constraints_.maxVelocity * constraints_.maxVelocity /
                           (2.0 * constraints_.maxAcceleration);

        if (2.0 * accel_dist > distance) {
            // Triangular profile (never reaches max velocity)
            accel_time_ = std::sqrt(distance / constraints_.maxAcceleration);
            cruise_time_ = 0;
        } else {
            // Trapezoidal profile
            accel_time_ = constraints_.maxVelocity / constraints_.maxAcceleration;
            double cruise_dist = distance - 2.0 * accel_dist;
            cruise_time_ = cruise_dist / constraints_.maxVelocity;
        }

        end_time_ = 2.0 * accel_time_ + cruise_time_;
    }

public:
    TrapezoidProfile(const Constraints& constraints, const State& goal,
                     const State& initial = State())
        : constraints_(constraints), initial_(initial), goal_(goal) {
        calculate();
    }

    State calculate(double t) const {
        if (t <= 0) return initial_;
        if (t >= end_time_) return goal_;

        double direction = (goal_.position > initial_.position) ? 1.0 : -1.0;

        if (t < accel_time_) {
            // Acceleration phase
            double a = direction * constraints_.maxAcceleration;
            double v = initial_.velocity + a * t;
            double p = initial_.position + initial_.velocity * t + 0.5 * a * t * t;
            return State(p, v, a);
        } else if (t < accel_time_ + cruise_time_) {
            // Cruise phase
            double dt = t - accel_time_;
            double v = direction * constraints_.maxVelocity;
            double accel_dist = 0.5 * constraints_.maxAcceleration * accel_time_ * accel_time_;
            double p = initial_.position + direction * accel_dist + v * dt;
            return State(p, v, 0);
        } else {
            // Deceleration phase
            double dt = t - accel_time_ - cruise_time_;
            double a = -direction * constraints_.maxAcceleration;
            double v = direction * constraints_.maxVelocity + a * dt;
            double accel_dist = 0.5 * constraints_.maxAcceleration * accel_time_ * accel_time_;
            double cruise_dist = constraints_.maxVelocity * cruise_time_;
            double p = initial_.position + direction * (accel_dist + cruise_dist) +
                      direction * constraints_.maxVelocity * dt + 0.5 * a * dt * dt;
            return State(p, v, a);
        }
    }

    double totalTime() const { return end_time_; }
    bool isFinished(double t) const { return t >= end_time_; }
};

// ============================================================================
// FILTER IMPLEMENTATIONS
// ============================================================================

// Low-Pass Filter
class LowPassFilter {
private:
    double alpha_;
    double output_;
    bool initialized_;

public:
    explicit LowPassFilter(double alpha = 0.9)
        : alpha_(numerical::clamp(alpha, 0.0, 1.0)), output_(0), initialized_(false) {}

    double update(double input) {
        if (!initialized_) {
            output_ = input;
            initialized_ = true;
        } else {
            output_ = alpha_ * output_ + (1.0 - alpha_) * input;
        }
        return output_;
    }

    double getValue() const { return output_; }
    void reset() { output_ = 0; initialized_ = false; }
    void setAlpha(double alpha) { alpha_ = numerical::clamp(alpha, 0.0, 1.0); }

    // Calculate alpha from cutoff frequency and sample rate
    static double alphaFromCutoff(double cutoff_hz, double sample_rate) {
        double rc = 1.0 / (constants::TWO_PI * cutoff_hz);
        double dt = 1.0 / sample_rate;
        return dt / (rc + dt);
    }
};

// Kalman Filter (1D)
class KalmanFilter1D {
private:
    double estimate_;
    double error_covariance_;
    double process_noise_;
    double measurement_noise_;
    bool initialized_;

public:
    KalmanFilter1D(double process_noise = 0.01, double measurement_noise = 0.1)
        : estimate_(0), error_covariance_(1.0),
          process_noise_(process_noise), measurement_noise_(measurement_noise),
          initialized_(false) {}

    double update(double measurement, double prediction = 0) {
        if (!initialized_) {
            estimate_ = measurement;
            initialized_ = true;
            return estimate_;
        }

        // Prediction step
        double predicted_estimate = estimate_ + prediction;
        double predicted_error = error_covariance_ + process_noise_;

        // Update step
        double kalman_gain = predicted_error / (predicted_error + measurement_noise_);
        estimate_ = predicted_estimate + kalman_gain * (measurement - predicted_estimate);
        error_covariance_ = (1.0 - kalman_gain) * predicted_error;

        return estimate_;
    }

    double getEstimate() const { return estimate_; }
    double getErrorCovariance() const { return error_covariance_; }
    void reset() { estimate_ = 0; error_covariance_ = 1.0; initialized_ = false; }
};

// Complementary Filter
class ComplementaryFilter {
private:
    double alpha_;
    double estimate_;
    bool initialized_;

public:
    explicit ComplementaryFilter(double alpha = 0.98)
        : alpha_(numerical::clamp(alpha, 0.0, 1.0)), estimate_(0), initialized_(false) {}

    // Combine gyro (high-freq) and accelerometer (low-freq) for angle estimation
    Radians updateAngle(const RadiansPerSecond& gyro_rate,
                        const Radians& accel_angle, double dt) {
        if (!initialized_) {
            estimate_ = accel_angle.toRadians();
            initialized_ = true;
        } else {
            // Integrate gyro rate
            double gyro_angle = estimate_ + gyro_rate.toRadiansPerSecond() * dt;
            // Combine with accelerometer measurement
            estimate_ = alpha_ * gyro_angle + (1.0 - alpha_) * accel_angle.toRadians();
        }
        return Radians::fromRadians(estimate_);
    }

    double getValue() const { return estimate_; }
    void reset() { estimate_ = 0; initialized_ = false; }
    void setAlpha(double alpha) { alpha_ = numerical::clamp(alpha, 0.0, 1.0); }
};

// Moving Average Filter
template<size_t WindowSize>
class MovingAverageFilter {
private:
    std::array<double, WindowSize> buffer_;
    size_t index_;
    double sum_;
    size_t count_;

public:
    MovingAverageFilter() : index_(0), sum_(0), count_(0) {
        buffer_.fill(0);
    }

    double update(double value) {
        if (count_ < WindowSize) {
            buffer_[count_] = value;
            sum_ += value;
            count_++;
        } else {
            sum_ -= buffer_[index_];
            buffer_[index_] = value;
            sum_ += value;
            index_ = (index_ + 1) % WindowSize;
        }
        return getValue();
    }

    double getValue() const {
        return count_ > 0 ? sum_ / count_ : 0;
    }

    void reset() {
        buffer_.fill(0);
        index_ = 0;
        sum_ = 0;
        count_ = 0;
    }
};

// Median Filter
template<size_t WindowSize>
class MedianFilter {
private:
    std::array<double, WindowSize> buffer_;
    std::array<double, WindowSize> sorted_buffer_;
    size_t index_;
    size_t count_;

public:
    MedianFilter() : index_(0), count_(0) {
        buffer_.fill(0);
        sorted_buffer_.fill(0);
    }

    double update(double value) {
        buffer_[index_] = value;
        index_ = (index_ + 1) % WindowSize;
        if (count_ < WindowSize) count_++;

        // Copy and sort for median calculation
        size_t n = count_;
        for (size_t i = 0; i < n; ++i) {
            sorted_buffer_[i] = buffer_[i];
        }
        std::sort(sorted_buffer_.begin(), sorted_buffer_.begin() + n);

        // Return median
        if (n % 2 == 0) {
            return (sorted_buffer_[n/2 - 1] + sorted_buffer_[n/2]) / 2.0;
        } else {
            return sorted_buffer_[n/2];
        }
    }

    void reset() {
        buffer_.fill(0);
        sorted_buffer_.fill(0);
        index_ = 0;
        count_ = 0;
    }
};

// Rate Limiter
class RateLimiter {
private:
    double max_rate_;
    double previous_value_;
    bool initialized_;

public:
    explicit RateLimiter(double max_rate)
        : max_rate_(std::abs(max_rate)), previous_value_(0), initialized_(false) {}

    double calculate(double input, double dt) {
        if (!initialized_) {
            previous_value_ = input;
            initialized_ = true;
            return input;
        }

        double delta = input - previous_value_;
        double max_delta = max_rate_ * dt;

        if (delta > max_delta) {
            previous_value_ += max_delta;
        } else if (delta < -max_delta) {
            previous_value_ -= max_delta;
        } else {
            previous_value_ = input;
        }

        return previous_value_;
    }

    void reset() { previous_value_ = 0; initialized_ = false; }
    void setMaxRate(double rate) { max_rate_ = std::abs(rate); }
    double getValue() const { return previous_value_; }
};

// Pure Pursuit Controller
class PurePursuitController {
private:
    double lookahead_distance_;
    double min_lookahead_;
    double max_lookahead_;

public:
    PurePursuitController(double lookahead, double min_lookahead = 0.1,
                         double max_lookahead = 5.0)
        : lookahead_distance_(lookahead),
          min_lookahead_(min_lookahead),
          max_lookahead_(max_lookahead) {}

    // Calculate angular velocity to follow path
    RadiansPerSecond calculate(const Pose2D& robot_pose,
                               const Vec2D& lookahead_point,
                               const MetersPerSecond& linear_velocity) {
        // Transform lookahead point to robot frame
        Vec2D local_point = robot_pose.toLocal(lookahead_point);

        // Calculate curvature to lookahead point
        // κ = 2y / L² where y is lateral offset and L is lookahead distance
        double curvature = 2.0 * local_point.y /
                          (lookahead_distance_ * lookahead_distance_);

        // Angular velocity = linear velocity × curvature
        return RadiansPerSecond::fromRadiansPerSecond(
            curvature * linear_velocity.toMetersPerSecond()
        );
    }

    // Adaptive lookahead based on speed
    void updateLookahead(const MetersPerSecond& velocity) {
        // Scale lookahead with velocity for better performance
        double speed = std::abs(velocity.toMetersPerSecond());
        double adaptive_distance = lookahead_distance_ * (1.0 + speed * 0.1);
        lookahead_distance_ = numerical::clamp(adaptive_distance,
                                               min_lookahead_,
                                               max_lookahead_);
    }

    void setLookahead(double distance) {
        lookahead_distance_ = numerical::clamp(distance, min_lookahead_, max_lookahead_);
    }

    double getLookahead() const { return lookahead_distance_; }
};

// Bang-Bang Controller
class BangBangController {
private:
    double threshold_;
    double hysteresis_;
    bool state_;

public:
    BangBangController(double threshold, double hysteresis = 0)
        : threshold_(threshold), hysteresis_(std::abs(hysteresis)), state_(false) {}

    bool update(double error) {
        if (state_) {
            // Currently ON - turn off if error drops below lower threshold
            if (error < threshold_ - hysteresis_) {
                state_ = false;
            }
        } else {
            // Currently OFF - turn on if error rises above upper threshold
            if (error > threshold_ + hysteresis_) {
                state_ = true;
            }
        }
        return state_;
    }

    bool getState() const { return state_; }
    void reset() { state_ = false; }
    void setThreshold(double threshold) { threshold_ = threshold; }
    void setHysteresis(double hysteresis) { hysteresis_ = std::abs(hysteresis); }
};

} // namespace robotics

// Import robotics namespace for convenience
using namespace robotics;

} // namespace units

#endif // ROBOTICS_UNITS_ROBOTICS_H
