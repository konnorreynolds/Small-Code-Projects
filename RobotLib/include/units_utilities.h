// ============================================================================
// units_utilities.h - Utility Functions and Extensions
// ============================================================================
// Purpose: Additional utilities, streaming operators, and helper functions
// Dependencies: units_core.h, units_physics.h, units_robotics.h
//
// This file contains:
// - Streaming operators for easy printing
// - Additional vector utilities
// - Common robotics calculations
// - Debugging helpers
// ============================================================================

#ifndef ROBOTICS_UNITS_UTILITIES_H
#define ROBOTICS_UNITS_UTILITIES_H

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"

#if !UNITS_EMBEDDED
#include <iostream>
#include <iomanip>
#include <sstream>
#endif

namespace units {
namespace utilities {

// ============================================================================
// STREAMING OPERATORS (for non-embedded platforms)
// ============================================================================
#if !UNITS_EMBEDDED

// Distance streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Distance<Ratio>& d) {
    return os << std::fixed << std::setprecision(3) << d.toMeters() << " m";
}

// Time streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Time<Ratio>& t) {
    return os << std::fixed << std::setprecision(3) << t.toSeconds() << " s";
}

// Angle streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Angle<Ratio>& a) {
    return os << std::fixed << std::setprecision(2) << a.toDegrees() << "°";
}

// Mass streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Mass<Ratio>& m) {
    return os << std::fixed << std::setprecision(3) << m.toKilograms() << " kg";
}

// Temperature streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Temperature<Ratio>& t) {
    return os << std::fixed << std::setprecision(1) << t.toCelsius() << "°C";
}

// Velocity streaming
template<typename DR, typename TR>
inline std::ostream& operator<<(std::ostream& os, const Velocity<DR, TR>& v) {
    return os << std::fixed << std::setprecision(2) << v.toMetersPerSecond() << " m/s";
}

// Force streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Force<Ratio>& f) {
    return os << std::fixed << std::setprecision(2) << f.toNewtons() << " N";
}

// Power streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Power<Ratio>& p) {
    return os << std::fixed << std::setprecision(2) << p.toWatts() << " W";
}

// Voltage streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Voltage<Ratio>& v) {
    return os << std::fixed << std::setprecision(2) << v.toVolts() << " V";
}

// Current streaming
template<typename Ratio>
inline std::ostream& operator<<(std::ostream& os, const Current<Ratio>& i) {
    return os << std::fixed << std::setprecision(3) << i.toAmperes() << " A";
}

// Vec2D streaming
inline std::ostream& operator<<(std::ostream& os, const robotics::Vec2D& v) {
    return os << "Vec2D(" << std::fixed << std::setprecision(3)
              << v.x << ", " << v.y << ")";
}

// Pose2D streaming
inline std::ostream& operator<<(std::ostream& os, const robotics::Pose2D& p) {
    return os << "Pose2D(" << std::fixed << std::setprecision(3)
              << p.position.x << ", " << p.position.y << ", "
              << std::setprecision(1) << p.theta.toDegrees() << "°)";
}

#endif // !UNITS_EMBEDDED

// ============================================================================
// ADDITIONAL VEC2D UTILITIES
// ============================================================================
namespace vec2d_utils {
    // Create unit vector from angle (accepts any angle type)
    template<typename AngleRatio>
    inline robotics::Vec2D fromAngle(const Angle<AngleRatio>& angle, double magnitude = 1.0) {
        double rad = angle.toRadians();
        return robotics::Vec2D(magnitude * std::cos(rad), magnitude * std::sin(rad));
    }

    // Get perpendicular vector (90° counter-clockwise rotation)
    constexpr robotics::Vec2D perpendicular(const robotics::Vec2D& v) {
        return robotics::Vec2D(-v.y, v.x);
    }

    // Clamp magnitude
    inline robotics::Vec2D clampMagnitude(const robotics::Vec2D& v, double maxMag) {
        double mag = v.magnitude();
        if (mag > maxMag && !numerical::isZero(mag)) {
            return v * (maxMag / mag);
        }
        return v;
    }

    // Linear interpolation
    constexpr robotics::Vec2D lerp(const robotics::Vec2D& a, const robotics::Vec2D& b, double t) {
        return robotics::Vec2D(
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t
        );
    }

    // Check if two vectors are parallel
    inline bool areParallel(const robotics::Vec2D& a, const robotics::Vec2D& b,
                           double tolerance = 0.001) {
        return numerical::approxEqual(std::abs(a.cross(b)), 0.0, tolerance);
    }

    // Check if two vectors are perpendicular
    inline bool arePerpendicular(const robotics::Vec2D& a, const robotics::Vec2D& b,
                                double tolerance = 0.001) {
        return numerical::approxEqual(a.dot(b), 0.0, tolerance);
    }
}

// ============================================================================
// DIFFERENTIAL DRIVE KINEMATICS
// ============================================================================
// WHAT: Convert between robot velocity and wheel velocities
//
// DIFFERENTIAL DRIVE = Two independently driven wheels
//   - Common in robots: Two wheels on sides, casters for balance
//   - Examples: TurtleBot, many mobile robots, simplified tanks
//
// TWO COORDINATE SYSTEMS:
//   1. CHASSIS FRAME (robot's perspective):
//      - Linear velocity: v (forward/backward speed)
//      - Angular velocity: ω (turning speed)
//
//   2. WHEEL FRAME (motor controller's perspective):
//      - Left wheel velocity: v_left
//      - Right wheel velocity: v_right
//
// WHY CONVERT?
//   - High-level control: Think in terms of "drive forward at 1 m/s"
//   - Low-level control: Motors need individual wheel speeds
//   - This struct bridges the gap!
//
// ============================================================================
struct DifferentialDrive {
    MetersPerSecond leftVelocity;
    MetersPerSecond rightVelocity;

    // ========================================================================
    // FORWARD KINEMATICS: Chassis velocity → Wheel velocities
    // ========================================================================
    // WHAT: "I want to drive forward at v and turn at ω, what wheel speeds?"
    //
    // THE FORMULAS:
    //   v_left  = v - (ω × L/2)
    //   v_right = v + (ω × L/2)
    //
    //   WHERE:
    //     v = linear velocity (m/s, forward positive)
    //     ω = angular velocity (rad/s, counter-clockwise positive)
    //     L = wheelbase (distance between left and right wheels)
    //
    // DERIVATION:
    //   When robot rotates at ω rad/s:
    //   - Center of robot moves at velocity v
    //   - Left wheel is L/2 from center → moves slower
    //   - Right wheel is L/2 from center → moves faster
    //   - Velocity difference: ω × (L/2)
    //
    // VISUAL:
    //        ↑ v_left          ↑ v_right
    //        |                 |
    //     [LEFT]    v→      [RIGHT]
    //        |     ←--L--→     |
    //
    //   If turning counter-clockwise (ω > 0):
    //     - Right wheel faster: v + ω×L/2
    //     - Left wheel slower:  v - ω×L/2
    //
    // EXAMPLES:
    //
    //   Example 1: Driving straight forward
    //     v = 1.0 m/s, ω = 0 rad/s, L = 0.5m
    //     v_left  = 1.0 - 0×0.25 = 1.0 m/s
    //     v_right = 1.0 + 0×0.25 = 1.0 m/s
    //     → Both wheels same speed = straight line!
    //
    //   Example 2: Turning in place (pivot)
    //     v = 0 m/s, ω = 1.0 rad/s, L = 0.5m
    //     v_left  = 0 - 1.0×0.25 = -0.25 m/s (backward)
    //     v_right = 0 + 1.0×0.25 =  0.25 m/s (forward)
    //     → Robot spins in place!
    //
    //   Example 3: Driving in arc
    //     v = 1.0 m/s, ω = 0.5 rad/s, L = 0.5m
    //     v_left  = 1.0 - 0.5×0.25 = 0.875 m/s
    //     v_right = 1.0 + 0.5×0.25 = 1.125 m/s
    //     → Curved path!
    //
    // ========================================================================
    static DifferentialDrive fromTwist(const MetersPerSecond& linear,
                                      const RadiansPerSecond& angular,
                                      const Meters& wheelbase) {
        double v = linear.toMetersPerSecond();
        double omega = angular.toRadiansPerSecond();
        double L = wheelbase.toMeters();

        // Forward kinematics equations
        double vLeft = v - (omega * L / 2.0);
        double vRight = v + (omega * L / 2.0);

        return DifferentialDrive{
            MetersPerSecond::fromMetersPerSecond(vLeft),
            MetersPerSecond::fromMetersPerSecond(vRight)
        };
    }

    // ========================================================================
    // INVERSE KINEMATICS: Wheel velocities → Chassis velocity
    // ========================================================================
    // WHAT: "My wheels are going these speeds, how fast is the robot moving?"
    //
    // THE FORMULAS:
    //   v = (v_left + v_right) / 2
    //   ω = (v_right - v_left) / L
    //
    //   WHERE:
    //     v = linear velocity (center of robot)
    //     ω = angular velocity (rotation rate)
    //     v_left, v_right = wheel velocities
    //     L = wheelbase
    //
    // DERIVATION:
    //
    //   From forward kinematics, we had:
    //     v_left  = v - ω×L/2
    //     v_right = v + ω×L/2
    //
    //   SOLVE FOR v (add equations):
    //     v_left + v_right = (v - ω×L/2) + (v + ω×L/2)
    //     v_left + v_right = 2v
    //     v = (v_left + v_right) / 2
    //
    //   INTERPRETATION: Linear velocity = average of wheel velocities
    //
    //   SOLVE FOR ω (subtract equations):
    //     v_right - v_left = (v + ω×L/2) - (v - ω×L/2)
    //     v_right - v_left = ω×L
    //     ω = (v_right - v_left) / L
    //
    //   INTERPRETATION: Turning rate = wheel speed difference / wheelbase
    //
    // WHY THIS WORKS:
    //   - If wheels go same speed → no rotation (ω=0)
    //   - If right faster → turn left (ω positive)
    //   - Larger wheel difference → faster turning
    //   - Wider wheelbase → slower turning for same wheel difference
    //
    // EXAMPLES:
    //
    //   Example 1: Straight driving
    //     v_left = 1.0 m/s, v_right = 1.0 m/s, L = 0.5m
    //     v = (1.0 + 1.0) / 2 = 1.0 m/s
    //     ω = (1.0 - 1.0) / 0.5 = 0 rad/s
    //     → Moving forward at 1 m/s, no rotation!
    //
    //   Example 2: Turning in place
    //     v_left = -0.25 m/s, v_right = 0.25 m/s, L = 0.5m
    //     v = (-0.25 + 0.25) / 2 = 0 m/s
    //     ω = (0.25 - (-0.25)) / 0.5 = 1.0 rad/s
    //     → Spinning at 1 rad/s, not moving forward!
    //
    //   Example 3: Arc motion
    //     v_left = 0.875 m/s, v_right = 1.125 m/s, L = 0.5m
    //     v = (0.875 + 1.125) / 2 = 1.0 m/s
    //     ω = (1.125 - 0.875) / 0.5 = 0.5 rad/s
    //     → Moving forward at 1 m/s while turning at 0.5 rad/s!
    //
    // USE CASE:
    //   - Read encoder velocities from motors
    //   - Convert to robot velocity for odometry
    //   - Feed into navigation algorithms
    //
    // ========================================================================
    struct Twist {
        MetersPerSecond linear;
        RadiansPerSecond angular;
    };

    Twist toTwist(const Meters& wheelbase) const {
        double vLeft = leftVelocity.toMetersPerSecond();
        double vRight = rightVelocity.toMetersPerSecond();
        double L = wheelbase.toMeters();

        // Inverse kinematics equations
        double linear = (vLeft + vRight) / 2.0;      // Average velocity
        double angular = (vRight - vLeft) / L;       // Differential velocity

        return Twist{
            MetersPerSecond::fromMetersPerSecond(linear),
            RadiansPerSecond::fromRadiansPerSecond(angular)
        };
    }
};

// ============================================================================
// BATTERY MONITORING
// ============================================================================
class BatteryMonitor {
private:
    Volts nominalVoltage_;
    Volts minVoltage_;
    Volts maxVoltage_;
    Volts currentVoltage_;

public:
    BatteryMonitor(const Volts& nominal, const Volts& min, const Volts& max)
        : nominalVoltage_(nominal), minVoltage_(min), maxVoltage_(max),
          currentVoltage_(nominal) {}

    void update(const Volts& voltage) {
        currentVoltage_ = voltage;
    }

    // Get state of charge (0.0 to 1.0)
    double getStateOfCharge() const {
        double v = currentVoltage_.toVolts();
        double vMin = minVoltage_.toVolts();
        double vMax = maxVoltage_.toVolts();

        if (v <= vMin) return 0.0;
        if (v >= vMax) return 1.0;

        return (v - vMin) / (vMax - vMin);
    }

    // Get percentage (0-100)
    double getPercentage() const {
        return getStateOfCharge() * 100.0;
    }

    // Check if battery is low
    bool isLow(double threshold = 0.2) const {
        return getStateOfCharge() < threshold;
    }

    // Check if battery is critical
    bool isCritical(double threshold = 0.1) const {
        return getStateOfCharge() < threshold;
    }

    // Estimate remaining capacity
    template<typename EnergyRatio>
    Energy<EnergyRatio> getRemainingCapacity(const Energy<EnergyRatio>& totalCapacity) const {
        double soc = getStateOfCharge();
        double capacity = totalCapacity.toJoules() * soc;
        return Energy<EnergyRatio>::fromJoules(capacity);
    }

    Volts getCurrentVoltage() const { return currentVoltage_; }
};

// ============================================================================
// MOTOR CONTROLLER UTILITIES
// ============================================================================
// WHAT: Helper functions for motor control
//
// ============================================================================
class MotorController {
public:
    // ========================================================================
    // RPM TO PWM CONVERSION
    // ========================================================================
    // WHAT: Convert desired motor speed to PWM duty cycle
    //
    // WHY: Motor controllers accept duty cycle (-1 to 1), but we think in RPM
    //
    // FORMULA:
    //   duty_cycle = desired_rpm / max_rpm
    //
    // EXAMPLE:
    //   Max RPM = 5000, Desired = 2500
    //   duty_cycle = 2500 / 5000 = 0.5 (50% power)
    //
    // ========================================================================
    static double rpmToPWM(const RPM& desiredRPM, const RPM& maxRPM) {
        double ratio = desiredRPM.toRPM() / maxRPM.toRPM();
        return numerical::clamp(ratio, -1.0, 1.0);
    }

    // ========================================================================
    // DEADBAND (Stick Zone Compensation)
    // ========================================================================
    // WHAT: Eliminate motor "dead zone" where small inputs don't move motor
    //
    // THE PROBLEM:
    //   Real motors have static friction and minimum voltage requirements:
    //
    //   Input:  0.0  0.05  0.10  0.15  0.20  ... 1.0
    //   Output: 0.0  0.0   0.0   0.0   0.05  ... 1.0
    //           ↑_____________↑
    //           Dead zone where motor doesn't move!
    //
    //   This creates "sticky" behavior:
    //   - Joystick barely moves → nothing happens
    //   - Then sudden jump when motor engages
    //   - Poor controllability!
    //
    // THE SOLUTION (Deadband):
    //   Map inputs above deadband to full range:
    //
    //   FORMULA:
    //     If |input| < deadband:
    //       output = 0
    //     Else:
    //       output = sign(input) × (|input| - deadband) / (1 - deadband)
    //
    //   VISUAL:
    //     Input range:  [0.0 ... deadband ... 1.0]
    //                          ↓
    //     Output range: [0.0    0.0    ... 1.0]
    //                   (ignore) (rescale to fill range)
    //
    // EXAMPLE (deadband = 0.1):
    //
    //   Input  → Output
    //   0.00   → 0.00   (below deadband, zero output)
    //   0.05   → 0.00   (below deadband, zero output)
    //   0.10   → 0.00   (at deadband threshold)
    //   0.20   → 0.11   = (0.20 - 0.10) / (1.0 - 0.10) = 0.10/0.90
    //   0.50   → 0.44   = (0.50 - 0.10) / 0.90 = 0.40/0.90
    //   1.00   → 1.00   = (1.00 - 0.10) / 0.90 = 0.90/0.90
    //
    // WHY THE RESCALING?
    //   Without rescaling:
    //     - Input 1.0 → Output 0.9 (only 90% power!)
    //     - Lose top speed
    //
    //   With rescaling:
    //     - Input 1.0 → Output 1.0 (full power maintained!)
    //     - Smooth control from deadband to max
    //
    // TYPICAL DEADBAND VALUES:
    //   - 0.05 (5%): Small dead zone, most motors
    //   - 0.10 (10%): Medium dead zone
    //   - 0.15 (15%): Large dead zone, worn motors
    //
    // ========================================================================
    static double applyDeadband(double input, double deadband = 0.05) {
        // If input below deadband threshold, return zero
        if (std::abs(input) < deadband) {
            return 0.0;
        }

        // Scale the remaining range to [0, 1]
        // This ensures full range is still achievable
        double sign = (input > 0) ? 1.0 : -1.0;
        return sign * (std::abs(input) - deadband) / (1.0 - deadband);
    }

    // ========================================================================
    // VELOCITY TO RPM CONVERSION
    // ========================================================================
    // WHAT: Convert robot's linear velocity to wheel rotation speed
    //
    // WHY:
    //   - High-level: "I want to move at 1.0 m/s"
    //   - Motors need: "Spin at X RPM"
    //   - This function bridges the gap!
    //
    // THE MATH:
    //
    //   KEY INSIGHT: One wheel rotation moves robot by wheel circumference
    //
    //   FORMULA:
    //     circumference = π × diameter
    //     rotations_per_second = velocity / circumference
    //     RPM = rotations_per_second × 60
    //
    //   DERIVATION:
    //     If wheel diameter = 0.1m (10cm):
    //       circumference = π × 0.1 = 0.314 m
    //
    //     If robot moves at 1.0 m/s:
    //       It covers 1.0 meters per second
    //       Each rotation moves 0.314m
    //       Therefore: 1.0 / 0.314 = 3.18 rotations/second
    //       RPM = 3.18 × 60 = 191 RPM
    //
    // EXAMPLE 1:
    //   Velocity = 2.0 m/s, Wheel diameter = 0.15m (15cm)
    //   circumference = π × 0.15 = 0.471 m
    //   rps = 2.0 / 0.471 = 4.24 rot/s
    //   RPM = 4.24 × 60 = 255 RPM
    //
    // EXAMPLE 2 (Small wheel):
    //   Velocity = 1.0 m/s, Wheel diameter = 0.05m (5cm)
    //   circumference = π × 0.05 = 0.157 m
    //   rps = 1.0 / 0.157 = 6.37 rot/s
    //   RPM = 6.37 × 60 = 382 RPM
    //   → Smaller wheels need to spin faster for same speed!
    //
    // ========================================================================
    static RPM velocityToRPM(const MetersPerSecond& velocity,
                            const Meters& wheelDiameter) {
        double v = velocity.toMetersPerSecond();
        double d = wheelDiameter.toMeters();
        double circumference = constants::PI * d;

        // Calculate rotations per second
        double rps = v / circumference;
        // Convert to RPM (rotations per minute)
        double rpm = rps * 60.0;

        return RPM::fromRPM(rpm);
    }

    // ========================================================================
    // RPM TO VELOCITY CONVERSION
    // ========================================================================
    // WHAT: Convert wheel rotation speed to robot's linear velocity
    //
    // WHY: Read motor encoder, calculate actual robot speed
    //
    // THE MATH (inverse of velocityToRPM):
    //
    //   FORMULA:
    //     rotations_per_second = RPM / 60
    //     circumference = π × diameter
    //     velocity = rotations_per_second × circumference
    //
    //   LOGIC:
    //     Each rotation moves robot by circumference distance
    //     If N rotations per second → N × circumference meters per second
    //
    // EXAMPLE 1:
    //   RPM = 200, Wheel diameter = 0.1m
    //   rotations_per_second = 200 / 60 = 3.33 rot/s
    //   circumference = π × 0.1 = 0.314 m
    //   velocity = 3.33 × 0.314 = 1.05 m/s
    //
    // EXAMPLE 2 (verify inverse):
    //   From earlier: 1.0 m/s → 191 RPM (with 0.1m wheel)
    //   Reverse: 191 RPM → velocity
    //     rps = 191 / 60 = 3.18 rot/s
    //     circumference = π × 0.1 = 0.314 m
    //     velocity = 3.18 × 0.314 = 1.0 m/s ✓
    //
    // ========================================================================
    static MetersPerSecond rpmToVelocity(const RPM& rpm,
                                        const Meters& wheelDiameter) {
        double rotationsPerSec = rpm.toRPM() / 60.0;
        double d = wheelDiameter.toMeters();
        double circumference = constants::PI * d;

        // velocity = how many meters traveled per second
        double velocity = rotationsPerSec * circumference;
        return MetersPerSecond::fromMetersPerSecond(velocity);
    }
};

// ============================================================================
// EXPONENTIAL MOVING AVERAGE FILTER (EMA)
// ============================================================================
// WHAT: Simple, efficient low-pass filter for smoothing noisy signals
//
// WHY: Same as LowPassFilter in units_robotics.h, provided here for convenience
//   - Sensors are noisy (random fluctuations)
//   - Want smooth estimate of true value
//   - Need efficient filter (minimal memory, fast computation)
//
// THE FORMULA:
//   value_new = α × measurement + (1-α) × value_old
//
//   WHERE:
//     α = smoothing factor (0 to 1)
//     α = 0 → ignores new measurements (never updates)
//     α = 1 → ignores history (just use raw measurement)
//     α = 0.1 typical → 10% new, 90% old (smooth)
//
// HOW IT WORKS:
//   Each new measurement is blended with previous estimate
//   Recent measurements weighted more than old ones (exponential decay)
//
// EXAMPLE:
//   α = 0.2, Initial value = 100
//
//   Measurement 1 = 110:
//     value = 0.2×110 + 0.8×100 = 22 + 80 = 102
//
//   Measurement 2 = 115:
//     value = 0.2×115 + 0.8×102 = 23 + 81.6 = 104.6
//
//   Measurement 3 = 120:
//     value = 0.2×120 + 0.8×104.6 = 24 + 83.68 = 107.68
//
//   → Smoothly approaches true value!
//
// ============================================================================
class ExponentialMovingAverage {
private:
    double alpha_;        // Smoothing factor [0, 1]
    double value_;        // Current filtered value
    bool initialized_;    // Has first value been received?

public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    // alpha: smoothing factor (0-1)
    //   - Higher = more responsive (follows changes quickly)
    //   - Lower = more smooth (rejects noise better)
    // ========================================================================
    explicit ExponentialMovingAverage(double alpha = 0.1)
        : alpha_(numerical::clamp(alpha, 0.0, 1.0)),
          value_(0.0), initialized_(false) {}

    // ========================================================================
    // UPDATE - Apply filter to new measurement
    // ========================================================================
    double update(double newValue) {
        if (!initialized_) {
            // First measurement: just use it directly
            value_ = newValue;
            initialized_ = true;
        } else {
            // Blend new measurement with previous estimate
            value_ = alpha_ * newValue + (1.0 - alpha_) * value_;
        }
        return value_;
    }

    double getValue() const { return value_; }
    void reset() { value_ = 0.0; initialized_ = false; }

    // ========================================================================
    // CREATE FROM TIME CONSTANT
    // ========================================================================
    // WHAT: Create filter from desired response time (easier than picking α)
    //
    // WHY:
    //   - α is abstract: "What does 0.1 mean?"
    //   - τ (tau) is intuitive: "How long to respond to change?"
    //
    // PARAMETERS:
    //   tau = time constant (seconds)
    //     - After time τ, filter reaches ~63% of step change
    //     - Smaller τ = faster response
    //     - Larger τ = smoother filtering
    //
    //   sampleTime = time between measurements (seconds)
    //     - e.g., 0.02s for 50Hz control loop
    //
    // THE FORMULA:
    //   α = Δt / (τ + Δt)
    //
    //   WHERE:
    //     Δt = sample time (time between updates)
    //     τ = time constant (desired response time)
    //
    // DERIVATION:
    //   Exponential moving average is discrete version of:
    //     dx/dt = (input - x) / τ
    //
    //   Discretize using forward Euler:
    //     (x_new - x_old) / Δt = (input - x_old) / τ
    //     x_new = x_old + Δt/τ × (input - x_old)
    //     x_new = x_old + α × (input - x_old)    where α = Δt/(τ+Δt)
    //     x_new = α×input + (1-α)×x_old
    //
    // EXAMPLES:
    //
    //   Example 1: Fast response
    //     τ = 0.1s, Δt = 0.02s (50Hz)
    //     α = 0.02 / (0.1 + 0.02) = 0.167
    //     → Responds in ~0.1 seconds
    //
    //   Example 2: Slow response (smooth)
    //     τ = 1.0s, Δt = 0.02s
    //     α = 0.02 / (1.0 + 0.02) = 0.0196 ≈ 0.02
    //     → Responds in ~1 second, very smooth
    //
    //   Example 3: Very fast sampling
    //     τ = 0.5s, Δt = 0.001s (1000Hz)
    //     α = 0.001 / (0.5 + 0.001) = 0.002
    //     → Same response time, but smaller α due to fast sampling
    //
    // RULE OF THUMB:
    //   τ = 3-5 × sample_time for good filtering
    //   τ < sample_time means barely filtering (α → 1)
    //   τ >> sample_time means heavy filtering (α → 0)
    //
    // ========================================================================
    static ExponentialMovingAverage fromTimeConstant(double tau, double sampleTime) {
        double alpha = sampleTime / (tau + sampleTime);
        return ExponentialMovingAverage(alpha);
    }
};

// ============================================================================
// SIMPLE ODOMETRY
// ============================================================================
class SimpleOdometry {
private:
    robotics::Pose2D pose_;
    Seconds lastUpdateTime_;
    bool initialized_;

public:
    SimpleOdometry() : pose_(), lastUpdateTime_(), initialized_(false) {}

    explicit SimpleOdometry(const robotics::Pose2D& initialPose)
        : pose_(initialPose), lastUpdateTime_(), initialized_(false) {}

    // Update with velocity measurements
    void update(const MetersPerSecond& linearVel,
               const RadiansPerSecond& angularVel,
               const Seconds& currentTime) {

        if (!initialized_) {
            lastUpdateTime_ = currentTime;
            initialized_ = true;
            return;
        }

        double dt = (currentTime - lastUpdateTime_).toSeconds();
        if (dt <= 0.0) return;

        double v = linearVel.toMetersPerSecond();
        double omega = angularVel.toRadiansPerSecond();

        // Update heading first
        Radians newHeading = pose_.theta + Radians::fromRadians(omega * dt);

        // Calculate position change in global frame
        // Use average heading for more accurate integration
        Radians avgHeading = pose_.theta + Radians::fromRadians(omega * dt / 2.0);

        double dx = v * dt * avgHeading.cos();
        double dy = v * dt * avgHeading.sin();

        // Update pose
        pose_.position.x += dx;
        pose_.position.y += dy;
        pose_.theta = newHeading;

        lastUpdateTime_ = currentTime;
    }

    // Update with differential drive
    void updateDifferential(const MetersPerSecond& leftVel,
                           const MetersPerSecond& rightVel,
                           const Meters& wheelbase,
                           const Seconds& currentTime) {
        DifferentialDrive drive{leftVel, rightVel};
        auto twist = drive.toTwist(wheelbase);
        update(twist.linear, twist.angular, currentTime);
    }

    robotics::Pose2D getPose() const { return pose_; }
    void setPose(const robotics::Pose2D& pose) { pose_ = pose; }
    void reset() { pose_ = robotics::Pose2D(); initialized_ = false; }
};

} // namespace utilities

// Make utilities available in units namespace
using namespace utilities;

} // namespace units

#endif // ROBOTICS_UNITS_UTILITIES_H
