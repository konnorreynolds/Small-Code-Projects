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

    // ========================================================================
    // DOT PRODUCT (Scalar Product)
    // ========================================================================
    // WHAT: Measures how much two vectors point in the same direction
    //
    // MATH: a · b = ax×bx + ay×by
    //
    // GEOMETRIC MEANING:
    //   a · b = |a| × |b| × cos(θ)
    //   where θ is angle between vectors
    //
    // PROPERTIES:
    //   - Parallel vectors (same direction): dot = |a|×|b| (maximum, positive)
    //   - Perpendicular vectors (90°): dot = 0
    //   - Opposite vectors (180°): dot = -|a|×|b| (minimum, negative)
    //
    // EXAMPLES:
    //   Vec2D(1,0) · Vec2D(1,0) = 1×1 + 0×0 = 1 (same direction)
    //   Vec2D(1,0) · Vec2D(0,1) = 1×0 + 0×1 = 0 (perpendicular)
    //   Vec2D(1,0) · Vec2D(-1,0) = 1×-1 + 0×0 = -1 (opposite)
    //   Vec2D(3,4) · Vec2D(1,0) = 3×1 + 4×0 = 3 (projection length)
    //
    // USE CASES:
    //   - Calculate angle between vectors: θ = acos(a·b / |a||b|)
    //   - Check if perpendicular: if (a·b ≈ 0)
    //   - Project vector onto another
    //   - Calculate work: W = Force · displacement
    constexpr double dot(const Vec2D& other) const {
        return x * other.x + y * other.y;
    }

    // ========================================================================
    // CROSS PRODUCT (2D version returns scalar, not vector)
    // ========================================================================
    // WHAT: In 2D, measures how much vectors are "rotated" relative to each other
    //
    // MATH: a × b = ax×by - ay×bx
    //
    // GEOMETRIC MEANING:
    //   a × b = |a| × |b| × sin(θ)
    //   where θ is angle from a to b (counterclockwise positive)
    //
    // SIGN MEANING:
    //   Positive: b is counterclockwise from a (left turn)
    //   Zero: vectors are parallel (same or opposite direction)
    //   Negative: b is clockwise from a (right turn)
    //
    // MAGNITUDE MEANING:
    //   |a × b| = area of parallelogram formed by vectors
    //
    // EXAMPLES:
    //   Vec2D(1,0) × Vec2D(0,1) = 1×1 - 0×0 = 1 (90° left turn)
    //   Vec2D(0,1) × Vec2D(1,0) = 0×0 - 1×1 = -1 (90° right turn)
    //   Vec2D(1,0) × Vec2D(2,0) = 1×0 - 0×2 = 0 (parallel)
    //   Vec2D(2,0) × Vec2D(0,3) = 2×3 - 0×0 = 6 (area = 6)
    //
    // USE CASES:
    //   - Determine turn direction (left vs right)
    //   - Calculate triangle/polygon area
    //   - Check if point is left/right of line
    //   - Detect clockwise/counterclockwise orientation
    constexpr double cross(const Vec2D& other) const {
        return x * other.y - y * other.x;
    }

    // ========================================================================
    // VECTOR PROJECTION
    // ========================================================================
    // WHAT: "Shadow" of this vector onto another vector
    //
    // MATH: proj_b(a) = (a·b / b·b) × b
    //
    // BREAKDOWN:
    //   1. a·b = how much of a points along b
    //   2. b·b = |b|² (length squared of b)
    //   3. (a·b / b·b) = scalar scale factor
    //   4. Multiply b by this factor
    //
    // VISUAL:
    //        a
    //       /|
    //      / |
    //     /  | (perpendicular drop)
    //    /   |
    //   /____|________  b
    //    proj (shadow)
    //
    // EXAMPLE:
    //   a = Vec2D(3, 4)
    //   b = Vec2D(1, 0) (unit vector along x-axis)
    //   a·b = 3×1 + 4×0 = 3
    //   b·b = 1×1 + 0×0 = 1
    //   proj = (3/1) × Vec2D(1,0) = Vec2D(3, 0)
    //   (The "shadow" of (3,4) on x-axis is (3,0))
    //
    // USE CASES:
    //   - Decompose velocity into components
    //   - Calculate force along a direction
    //   - Find closest point on a line
    UNITS_CONSTEXPR14 Vec2D project(const Vec2D& onto) const {
        double d = onto.dot(onto);
        return numerical::isZero(d) ? Vec2D() : onto * (this->dot(onto) / d);
    }

    // ========================================================================
    // VECTOR REFLECTION
    // ========================================================================
    // WHAT: Mirror this vector across a surface defined by normal vector
    //
    // MATH: reflected = v - 2(v·n)n
    //   where n is normalized (unit length) normal vector
    //
    // BREAKDOWN:
    //   1. v·n = how much v points into the surface
    //   2. 2(v·n)n = twice the perpendicular component
    //   3. v - 2(v·n)n = original - 2×perp = reflected
    //
    // VISUAL:
    //      v (incoming)
    //       \
    //        \
    //   ------n------- surface (normal n)
    //          \
    //           \ reflected (outgoing)
    //
    // LAW OF REFLECTION: angle_in = angle_out
    //
    // EXAMPLE (bounce off vertical wall):
    //   v = Vec2D(3, 4) (moving right-up)
    //   normal = Vec2D(1, 0) (wall faces right)
    //   v·n = 3×1 + 4×0 = 3
    //   reflected = (3,4) - 2×3×(1,0) = (3,4) - (6,0) = (-3,4)
    //   (Now moving left-up, bounced off wall)
    //
    // USE CASES:
    //   - Ball bouncing off walls
    //   - Laser/light reflection
    //   - Collision response
    UNITS_CONSTEXPR14 Vec2D reflect(const Vec2D& normal) const {
        Vec2D n = normal.normalized();
        return *this - n * (2.0 * this->dot(n));
    }

    // ========================================================================
    // 2D ROTATION
    // ========================================================================
    // WHAT: Rotate vector counterclockwise by angle
    //
    // MATH (Rotation Matrix):
    //   [x']   [cos(θ)  -sin(θ)] [x]
    //   [y'] = [sin(θ)   cos(θ)] [y]
    //
    // EXPANDED:
    //   x' = x×cos(θ) - y×sin(θ)
    //   y' = x×sin(θ) + y×cos(θ)
    //
    // WHY THIS FORMULA:
    //   Derived from polar coordinates and trig identities
    //   Preserves vector length (rotation doesn't scale)
    //
    // EXAMPLES:
    //   Vec2D(1,0).rotate(90°):
    //     x' = 1×cos(90°) - 0×sin(90°) = 1×0 - 0×1 = 0
    //     y' = 1×sin(90°) + 0×cos(90°) = 1×1 + 0×0 = 1
    //     Result: Vec2D(0, 1) ✓ (rotated from east to north)
    //
    //   Vec2D(5,0).rotate(45°):
    //     cos(45°) = 0.707, sin(45°) = 0.707
    //     x' = 5×0.707 - 0×0.707 = 3.535
    //     y' = 5×0.707 + 0×0.707 = 3.535
    //     Result: Vec2D(3.535, 3.535) (northeast, 45°)
    //
    // VISUAL:
    //        y (north)
    //        |
    //        |  / rotated
    //        | /
    //   -----+--------- x (east)
    //        |   original →
    //
    // SIGN CONVENTION:
    //   Positive angle: counterclockwise (math standard)
    //   Negative angle: clockwise
    //
    // USE CASES:
    //   - Transform robot coordinate frames
    //   - Rotate heading vectors
    //   - Orbital motion
    //   - Steering calculations
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

    // ========================================================================
    // PID CALCULATE - Main control loop
    // ========================================================================
    // WHAT: Calculate control output to minimize error
    //
    // PID EQUATION:
    //   output = kP×error + kI×∫error×dt + kD×(derror/dt) + kF×feedforward
    //
    // TERMS EXPLAINED:
    //
    // 1. PROPORTIONAL (P): Responds to current error
    //    - Formula: P = kP × error
    //    - Meaning: Bigger error → bigger correction
    //    - Problem: Can't eliminate steady-state error, oscillates
    //    - Analogy: Pressing gas pedal harder when you're further from target speed
    //
    // 2. INTEGRAL (I): Responds to accumulated past error
    //    - Formula: I = kI × ∑(error × dt)
    //    - Meaning: Sums up all past errors over time
    //    - Purpose: Eliminates steady-state error (persistent offset)
    //    - Problem: Can "wind up" (accumulate too much), causes overshoot
    //    - Analogy: Keeps pushing harder if you haven't reached target yet
    //
    // 3. DERIVATIVE (D): Responds to rate of change of error
    //    - Formula: D = kD × (Δerror / Δt)
    //    - Meaning: How fast is error changing?
    //    - Purpose: Damping (reduce oscillation, smooth motion)
    //    - Problem: Amplifies noise (small fluctuations)
    //    - Analogy: Start braking before you reach target to avoid overshoot
    //
    // 4. FEEDFORWARD (F): Pre-calculated control based on target
    //    - Formula: F = kF × setpoint
    //    - Meaning: "I know I'll need this much control for this target"
    //    - Purpose: Faster response, less reliance on feedback
    //    - Analogy: You know a heavy load needs more gas, don't wait for error
    //
    // EXAMPLE (Temperature control, target = 25°C):
    //   Current temp = 20°C
    //   Error = 25 - 20 = 5°C (too cold)
    //
    //   P term: kP×5 = immediate heating proportional to 5° difference
    //   I term: kI×(sum of all past errors) = compensate for heat loss
    //   D term: kD×(how fast temp is changing) = don't overshoot 25°
    //   Total: P + I + D = heater power
    //
    // TUNING GUIDELINES:
    //   Start: kP only, then add D, then add I
    //   - Increase kP until oscillation starts
    //   - Add kD to dampen oscillations
    //   - Add small kI to eliminate steady-state error
    //
    // ========================================================================
    double calculate(double error, double dt, double feedforward = 0) {
        // First call initialization
        if (!initialized_) {
            prev_error_ = error;
            prev_derivative_ = 0;
            initialized_ = true;
        }

        // ====================================================================
        // PROPORTIONAL TERM
        // ====================================================================
        // MATH: P = kP × error
        // EFFECT: Immediate response to current error
        // LARGER kP: Faster response, but more oscillation
        // EXAMPLE: error = 10, kP = 0.5 → P = 5
        double p = gains_.kP * error;

        // ====================================================================
        // INTEGRAL TERM (with anti-windup)
        // ====================================================================
        // MATH: I = kI × ∫error dt ≈ kI × sum(error × dt)
        //
        // DISCRETE INTEGRATION (Riemann sum):
        //   integral += error × dt
        //   This approximates the area under the error curve
        //
        // WHY CLAMPING (Anti-windup):
        //   If error persists (e.g., stuck against wall), integral grows huge
        //   When error finally decreases, huge integral causes massive overshoot
        //   Solution: Limit integral to reasonable range [-iMax, +iMax]
        //
        // EXAMPLE:
        //   dt = 0.01s, error = 5 for 10 steps
        //   integral = 5×0.01 + 5×0.01 + ... = 0.5 after 10 steps
        //   If kI = 2, then I = 2 × 0.5 = 1.0
        integral_ += error * dt;
        integral_ = numerical::clamp(integral_, -gains_.iMax, gains_.iMax);
        double i = gains_.kI * integral_;

        // ====================================================================
        // DERIVATIVE TERM (with low-pass filtering)
        // ====================================================================
        // MATH: D = kD × (derror/dt) ≈ kD × (error - prev_error) / dt
        //
        // PROBLEM: Derivative amplifies noise!
        //   If error jumps from sensor noise: 5.0 → 5.1 → 5.0
        //   Raw derivative: 0.1/0.01 = 10 (huge spike!)
        //
        // SOLUTION: Low-pass filter
        //   filtered_D = α×previous_D + (1-α)×new_D
        //   α = 0.9 means 90% old value, 10% new value (smooth)
        //
        // EXAMPLE:
        //   prev_error = 10, error = 8, dt = 0.1
        //   raw_derivative = (8-10)/0.1 = -20 (error decreasing fast)
        //   If prev filtered = -15, α = 0.9:
        //   derivative = 0.9×(-15) + 0.1×(-20) = -13.5 - 2 = -15.5
        //   D = kD × (-15.5)
        double raw_derivative = numerical::safeDivide(error - prev_error_, dt);
        double derivative = derivative_alpha_ * prev_derivative_ +
                          (1.0 - derivative_alpha_) * raw_derivative;
        double d = gains_.kD * derivative;

        // Update state for next iteration
        prev_error_ = error;
        prev_derivative_ = derivative;

        // ====================================================================
        // COMBINE ALL TERMS
        // ====================================================================
        // Total output = P + I + D + Feedforward
        //
        // P: React to current error (main driver)
        // I: Fix persistent offset
        // D: Smooth the response
        // F: Pre-compensate based on known system behavior
        double output = p + i + d + gains_.kF * feedforward;

        // ====================================================================
        // OUTPUT SATURATION
        // ====================================================================
        // PROBLEM: Can't apply infinite control
        //   Motor PWM: limited to [0, 255]
        //   Heater: can't be more than 100% on
        //
        // SOLUTION: Clamp output to physical limits
        //   This prevents damaging actuators
        double saturated_output = numerical::clamp(output, gains_.outputMin, gains_.outputMax);

        // ====================================================================
        // ANTI-WINDUP (Back-calculation method)
        // ====================================================================
        // PROBLEM: If output is saturated, integral keeps growing!
        //   Example: Motor at 100%, but error persists
        //   Integral grows huge while saturated
        //   When error finally reduces, huge integral causes overshoot
        //
        // SOLUTION: When saturated, reduce integral
        //   1. Calculate excess = output - saturated_output
        //   2. Remove this excess from integral
        //   3. Use gradual reduction (0.1 factor) for stability
        //
        // MATH:
        //   If output=120 but saturated_output=100 (limit):
        //   excess = 120 - 100 = 20
        //   Reduce integral by: 20 / kI × 0.1
        //
        // WHY: Prevents integral windup during saturation
        //      Allows faster recovery when error changes
        if (output != saturated_output && gains_.kI != 0) {
            double excess = output - saturated_output;
            integral_ -= excess / gains_.kI * 0.1;
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

    // ========================================================================
    // CALCULATE MOTION PROFILE
    // ========================================================================
    // WHAT: Plan a smooth motion from start to goal with acceleration limits
    //
    // WHY: Prevents:
    //   - Mechanical stress (instant acceleration damages motors/gears)
    //   - Wheel slip (sudden acceleration loses traction)
    //   - Jerky motion (uncomfortable, inaccurate)
    //   - Following errors (controller can't keep up with instant changes)
    //
    // TWO PROFILE TYPES:
    //
    // 1. TRAPEZOIDAL PROFILE (long distance):
    //    Velocity
    //      ^     ___________
    //      |    /           \
    //      |   /             \
    //      |  /               \
    //      | /                 \
    //      |/___________________|> Time
    //       accel cruise  decel
    //
    //    Phases:
    //    - Accelerate to max velocity
    //    - Cruise at max velocity
    //    - Decelerate to stop
    //
    // 2. TRIANGULAR PROFILE (short distance):
    //    Velocity
    //      ^      /\
    //      |     /  \
    //      |    /    \
    //      |   /      \
    //      |  /        \
    //      |_/__________|> Time
    //       accel decel
    //
    //    Phases:
    //    - Accelerate (never reach max velocity!)
    //    - Immediately decelerate
    //    - Distance too short for cruise phase
    //
    // ========================================================================
    void calculate() {
        double distance = std::abs(goal_.position - initial_.position);

        // ====================================================================
        // CALCULATE ACCELERATION DISTANCE
        // ====================================================================
        // QUESTION: How far do we travel while accelerating to max velocity?
        //
        // KINEMATIC EQUATION: v² = v₀² + 2a×d
        //   Where: v = final velocity (max velocity)
        //          v₀ = initial velocity (0)
        //          a = acceleration
        //          d = distance traveled
        //
        // SOLVE FOR d:
        //   v² = 0 + 2×a×d
        //   d = v² / (2×a)
        //
        // EXAMPLE:
        //   max_velocity = 2 m/s
        //   acceleration = 1 m/s²
        //   accel_dist = (2²) / (2×1) = 4 / 2 = 2 meters
        //   (Takes 2 meters to reach 2 m/s at 1 m/s² acceleration)
        double accel_dist = constraints_.maxVelocity * constraints_.maxVelocity /
                           (2.0 * constraints_.maxAcceleration);

        // ====================================================================
        // DETERMINE PROFILE TYPE
        // ====================================================================
        // COMPARISON: Do we have enough distance to reach max velocity?
        //
        // Total distance needed to accel + decel = 2 × accel_dist
        //
        // IF (2 × accel_dist > distance):
        //   TRIANGULAR profile - not enough room to reach max velocity
        //   Must start decelerating before reaching max!
        //
        // ELSE:
        //   TRAPEZOIDAL profile - enough room for cruise phase
        //   Accelerate, cruise at max, then decelerate
        //
        if (2.0 * accel_dist > distance) {
            // ================================================================
            // TRIANGULAR PROFILE
            // ================================================================
            // PROBLEM: Distance too short, can't reach max velocity
            // SOLUTION: Calculate peak velocity we CAN reach
            //
            // KINEMATIC EQUATION: v² = 2×a×d
            //   But distance is split: half for accel, half for decel
            //   So: d_accel = distance / 2
            //
            // TIME TO ACCEL: t = √(distance / acceleration)
            //   Derivation:
            //   d = ½×a×t² (distance during constant acceleration)
            //   Solving for t: t = √(2d/a)
            //   Since we use half distance: t = √(d/a)
            //
            // EXAMPLE:
            //   distance = 1 meter
            //   acceleration = 1 m/s²
            //   accel_time = √(1/1) = 1 second
            //   (Accelerate for 1s, decelerate for 1s, total 2s)
            accel_time_ = std::sqrt(distance / constraints_.maxAcceleration);
            cruise_time_ = 0;  // No cruise phase!

        } else {
            // ================================================================
            // TRAPEZOIDAL PROFILE
            // ================================================================
            // ENOUGH DISTANCE: Can reach max velocity and cruise
            //
            // ACCEL TIME: t = v / a
            //   How long to reach max velocity?
            //   From: v = v₀ + a×t with v₀=0
            //   t = v / a
            //
            // CRUISE DISTANCE:
            //   Total distance = accel_dist + cruise_dist + decel_dist
            //   cruise_dist = total - accel_dist - decel_dist
            //   cruise_dist = total - 2×accel_dist
            //   (Because accel_dist = decel_dist)
            //
            // CRUISE TIME:
            //   time = distance / velocity
            //   cruise_time = cruise_dist / max_velocity
            //
            // EXAMPLE:
            //   distance = 10 meters
            //   max_velocity = 2 m/s
            //   acceleration = 1 m/s²
            //
            //   accel_dist = 2²/(2×1) = 2 m
            //   accel_time = 2/1 = 2 s
            //   cruise_dist = 10 - 2×2 = 6 m
            //   cruise_time = 6/2 = 3 s
            //   decel_time = 2 s
            //   total_time = 2 + 3 + 2 = 7 s
            accel_time_ = constraints_.maxVelocity / constraints_.maxAcceleration;
            double cruise_dist = distance - 2.0 * accel_dist;
            cruise_time_ = cruise_dist / constraints_.maxVelocity;
        }

        // Total time = accel + cruise + decel
        // (decel time = accel time, symmetric profile)
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
// WHY FILTERS:
//   Sensors are noisy! Raw sensor data jumps around due to:
//   - Electrical noise
//   - Vibrations
//   - Quantization errors
//   - Environmental interference
//
//   Filters smooth out noise to get more accurate measurements
// ============================================================================

// ============================================================================
// LOW-PASS FILTER (Exponentially Weighted Moving Average)
// ============================================================================
// WHAT: Smooths rapid changes, passes through slow changes
//
// WHY "LOW-PASS":
//   - LOW frequencies (slow changes) → PASS through
//   - HIGH frequencies (fast noise) → BLOCKED/smoothed
//
// EXAMPLE:
//   Temperature slowly rising: passes through ✓
//   Electrical noise spikes: blocked ✗
//
// MATH:
//   output_new = α × output_old + (1-α) × input_new
//
//   where α ∈ [0, 1] is the smoothing factor
//
// PARAMETER α (alpha):
//   α = 0.0: No filtering (output = input, all noise)
//   α = 0.5: Medium filtering (50% old, 50% new)
//   α = 0.9: Heavy filtering (90% old, 10% new, very smooth)
//   α = 1.0: Infinite filtering (output never changes!)
//
// PROPERTIES:
//   - Simple (one line of code)
//   - Fast (constant time)
//   - Low memory (one value)
//   - Introduces lag (higher α = more lag)
//
// TIME CONSTANT:
//   τ = -Δt / ln(α)
//   where Δt = sample period
//
//   This tells you how "fast" the filter responds
//   Larger τ = slower response, smoother output
//
// ============================================================================
class LowPassFilter {
private:
    double alpha_;
    double output_;
    bool initialized_;

public:
    explicit LowPassFilter(double alpha = 0.9)
        : alpha_(numerical::clamp(alpha, 0.0, 1.0)), output_(0), initialized_(false) {}

    // ========================================================================
    // UPDATE - Apply low-pass filter to new measurement
    // ========================================================================
    // ALGORITHM:
    //   First call: output = input (initialize with first measurement)
    //   Later calls: output = α×old + (1-α)×new
    //
    // BREAKDOWN:
    //   1. Take α% of previous output (memory)
    //   2. Take (1-α)% of new input (new information)
    //   3. Sum them together
    //
    // EXAMPLE (α=0.9):
    //   Previous output: 10.0
    //   New input: 14.0 (noisy spike!)
    //
    //   Calculation:
    //   output = 0.9×10.0 + 0.1×14.0
    //         = 9.0 + 1.4
    //         = 10.4
    //
    //   Instead of jumping to 14.0, we gently move to 10.4
    //   This rejects the noise spike!
    //
    // WHY THIS WORKS:
    //   If input is noise (short spike), α keeps output stable
    //   If input is real trend (consistent), it gradually appears
    //
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

    // ========================================================================
    // CALCULATE ALPHA FROM CUTOFF FREQUENCY
    // ========================================================================
    // WHAT: Convert desired cutoff frequency to alpha parameter
    //
    // CUTOFF FREQUENCY (fc):
    //   Frequency where filter starts significantly attenuating
    //   Example: fc=10Hz means frequencies above 10Hz are reduced
    //
    // MATH:
    //   τ = 1 / (2π × fc)      [time constant from cutoff]
    //   Δt = 1 / sample_rate   [sample period]
    //   α = Δt / (τ + Δt)      [alpha from time constant]
    //
    // DERIVATION:
    //   From continuous-time RC filter: H(s) = 1 / (1 + sτ)
    //   Cutoff frequency: fc = 1 / (2πτ)
    //   Therefore: τ = 1 / (2πfc) = RC
    //
    //   Discrete approximation using Euler method:
    //   α = Δt / (τ + Δt)
    //
    // WHY Δt/(τ+Δt)?
    //   This is the "discrete-time equivalent" of the continuous filter
    //   Derived from bilinear transform (Tustin's method)
    //
    // EXAMPLE:
    //   Want to filter out noise above 5Hz
    //   Sample rate: 100Hz (Δt = 0.01s)
    //
    //   τ = 1 / (2π × 5) = 1 / 31.4 = 0.0318 seconds
    //   α = 0.01 / (0.0318 + 0.01) = 0.01 / 0.0418 = 0.239
    //
    //   Use α ≈ 0.24 for this filter
    //
    // NOTE: Returns α that should be used (not 1-α!)
    //
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

// ============================================================================
// COMPLEMENTARY FILTER (Sensor Fusion)
// ============================================================================
// WHAT: Combines two sensors that have complementary strengths/weaknesses
//
// WHY NEEDED: IMUs (Inertial Measurement Units) have two sensors:
//
//   1. GYROSCOPE:
//      ✓ Good: Accurate short-term, no noise, fast response
//      ✗ Bad: Drifts over time (integration error accumulates)
//
//   2. ACCELEROMETER:
//      ✓ Good: Accurate long-term (gravity is constant)
//      ✗ Bad: Noisy, affected by vibrations and linear acceleration
//
// PROBLEM:
//   - Trust only gyro → angle drifts away over time
//   - Trust only accel → angle jumps around from noise
//
// SOLUTION: COMPLEMENTARY FILTER!
//   - Use gyro for short-term changes (high-frequency)
//   - Use accel to correct long-term drift (low-frequency)
//
// MATH:
//   angle = α × angle_gyro + (1-α) × angle_accel
//
//   where:
//   angle_gyro = previous_angle + gyro_rate × dt  (integration)
//   angle_accel = atan2(ay, az)  (measured from gravity)
//   α ≈ 0.98 (typically 0.95-0.99)
//
// WHY "COMPLEMENTARY":
//   α acts as high-pass filter for gyro (passes high freq)
//   (1-α) acts as low-pass filter for accel (passes low freq)
//   Together they cover all frequencies with no gaps!
//
// FREQUENCY BREAKDOWN:
//   α = 0.98 means:
//   - 98% trust in gyro (short-term accurate)
//   - 2% trust in accel (long-term correction)
//
//   Over time:
//   - Fast changes (1 second): 98% gyro dominates
//   - Slow drift (1 minute): 2% accel correction adds up
//
// VISUAL:
//   Gyro:  ___/‾‾‾\___ (fast response, but drifts)
//   Accel: ~~~∿~~~∿~~~ (slow average, but noisy)
//   Result: ___/‾‾‾\___ (fast AND stable!)
//
// COMPARISON TO ALTERNATIVES:
//   - Kalman Filter: More optimal but requires tuning noise covariances
//   - Complementary: Simpler, one parameter, good enough for most robots
//
// TYPICAL VALUES:
//   α = 0.98: Standard (2% correction per step)
//   α = 0.95: More accel trust (for less gyro drift)
//   α = 0.99: More gyro trust (for noisy environment)
//
// TIME CONSTANT:
//   τ ≈ dt / (1 - α)
//
//   Example: α=0.98, dt=0.01s
//   τ = 0.01 / 0.02 = 0.5 seconds
//   (Takes ~0.5s to correct 63% of drift error)
//
// ============================================================================
class ComplementaryFilter {
private:
    double alpha_;
    double estimate_;
    bool initialized_;

public:
    explicit ComplementaryFilter(double alpha = 0.98)
        : alpha_(numerical::clamp(alpha, 0.0, 1.0)), estimate_(0), initialized_(false) {}

    // ========================================================================
    // UPDATE ANGLE - Fuse gyro and accelerometer
    // ========================================================================
    // INPUTS:
    //   gyro_rate: Angular velocity from gyroscope (rad/s)
    //   accel_angle: Angle calculated from accelerometer (radians)
    //   dt: Time since last update (seconds)
    //
    // ALGORITHM:
    //
    // STEP 1: INTEGRATE GYRO (high-frequency accurate)
    //   θ_gyro = θ_prev + ω × dt
    //
    //   This uses the gyro's angular rate to predict new angle
    //   Very accurate short-term, but small errors accumulate (drift)
    //
    // STEP 2: MEASURE FROM ACCEL (low-frequency accurate)
    //   θ_accel = atan2(ay, az)
    //
    //   Accelerometer measures gravity direction
    //   Accurate long-term, but noisy short-term
    //
    // STEP 3: FUSE BOTH
    //   θ_fused = α × θ_gyro + (1-α) × θ_accel
    //
    //   Combine both: mostly trust gyro, slightly correct with accel
    //
    // EXAMPLE (α=0.98):
    //   Previous estimate: 45°
    //   Gyro rate: 10°/s, dt=0.01s
    //   Accel angle: 46° (slightly different due to noise or drift)
    //
    //   STEP 1: Gyro integration
    //     θ_gyro = 45° + 10°/s × 0.01s = 45.1°
    //
    //   STEP 2: Already have accel measurement
    //     θ_accel = 46°
    //
    //   STEP 3: Fusion
    //     θ_fused = 0.98 × 45.1° + 0.02 × 46°
    //             = 44.198° + 0.92°
    //             = 45.118°
    //
    //   Result: Mostly follows gyro (45.1°) but slightly pulled
    //           toward accel (46°) to prevent long-term drift
    //
    // WHY THIS WORKS:
    //   - Gyro provides smooth, fast response
    //   - Small accel correction prevents drift accumulation
    //   - Over many iterations, drift gets corrected
    //
    Radians updateAngle(const RadiansPerSecond& gyro_rate,
                        const Radians& accel_angle, double dt) {
        if (!initialized_) {
            estimate_ = accel_angle.toRadians();
            initialized_ = true;
        } else {
            // STEP 1: Integrate gyro rate (dead reckoning)
            double gyro_angle = estimate_ + gyro_rate.toRadiansPerSecond() * dt;

            // STEP 2 & 3: Fuse with accelerometer measurement
            estimate_ = alpha_ * gyro_angle + (1.0 - alpha_) * accel_angle.toRadians();
        }
        return Radians::fromRadians(estimate_);
    }

    double getValue() const { return estimate_; }
    void reset() { estimate_ = 0; initialized_ = false; }
    void setAlpha(double alpha) { alpha_ = numerical::clamp(alpha, 0.0, 1.0); }
};

// ============================================================================
// MOVING AVERAGE FILTER
// ============================================================================
// WHAT: Average of the last N measurements
//
// MATH:
//   output = (x₁ + x₂ + ... + xₙ) / N
//
//   where x₁...xₙ are the N most recent measurements
//
// HOW IT WORKS:
//   Keeps a "sliding window" of the last N values
//   As new value comes in, oldest value drops out
//
// VISUAL (WindowSize = 5):
//   Time 1: [3] → average = 3
//   Time 2: [3, 5] → average = 4
//   Time 3: [3, 5, 7] → average = 5
//   Time 4: [3, 5, 7, 2] → average = 4.25
//   Time 5: [3, 5, 7, 2, 6] → average = 4.6  (window full)
//   Time 6: [5, 7, 2, 6, 8] → average = 5.6  (3 dropped out)
//            ^drop  ^new
//
// VS LOW-PASS FILTER:
//   Moving Average: All samples weighted equally
//   Low-Pass: Recent samples weighted more
//
// PROS:
//   - Predictable behavior
//   - No tuning parameter (just window size)
//   - Good for periodic noise
//
// CONS:
//   - Requires memory for N samples
//   - Old measurements still affect output
//   - Delay = WindowSize / (2 × sample_rate)
//
// EFFICIENCY:
//   This implementation uses a "circular buffer" for O(1) updates:
//   1. Keep running sum
//   2. Subtract oldest value from sum
//   3. Add new value to sum
//   4. Return sum / count
//
//   Without this trick, would need to recalculate sum each time O(N)!
//
// TEMPLATE PARAMETER:
//   WindowSize: Number of samples to average
//   - Larger window = smoother but more lag
//   - Smaller window = less lag but more noise
//   - Typical: 5-20 samples
//
// ============================================================================
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

    // ========================================================================
    // UPDATE - Add new measurement and return filtered value
    // ========================================================================
    // ALGORITHM:
    //
    // PHASE 1: Filling up (count < WindowSize)
    //   Just add new values until buffer is full
    //   Average = sum / count (grows from 1 to WindowSize)
    //
    // PHASE 2: Sliding window (count == WindowSize)
    //   1. Remove oldest value from sum: sum -= buffer[index]
    //   2. Store new value in its place: buffer[index] = value
    //   3. Add new value to sum: sum += value
    //   4. Move index forward (circular): index = (index+1) % WindowSize
    //   5. Return average: sum / WindowSize
    //
    // CIRCULAR BUFFER:
    //   index points to the OLDEST value (next to be replaced)
    //   Using modulo % makes buffer "wrap around"
    //
    // EXAMPLE (WindowSize=3):
    //   Buffer: [?, ?, ?]  index=0, sum=0
    //
    //   update(5):
    //     Buffer: [5, ?, ?]  index=0, sum=5, count=1 → returns 5
    //
    //   update(7):
    //     Buffer: [5, 7, ?]  index=0, sum=12, count=2 → returns 6
    //
    //   update(3):
    //     Buffer: [5, 7, 3]  index=0, sum=15, count=3 → returns 5
    //                                  (buffer now full!)
    //
    //   update(9):
    //     sum -= buffer[0];    // sum = 15 - 5 = 10
    //     buffer[0] = 9;       // Buffer: [9, 7, 3]
    //     sum += 9;            // sum = 10 + 9 = 19
    //     index = 1;           // Move to next position
    //     returns 19/3 = 6.33
    //
    //   update(2):
    //     sum -= buffer[1];    // sum = 19 - 7 = 12
    //     buffer[1] = 2;       // Buffer: [9, 2, 3]
    //     sum += 2;            // sum = 12 + 2 = 14
    //     index = 2;           // Move to next position
    //     returns 14/3 = 4.67
    //
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
