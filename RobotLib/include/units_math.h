// ============================================================================
// units_math.h - Advanced Mathematical Functions for RobotLib
// ============================================================================
// This header provides advanced mathematical functions and utilities for
// working with typed units in robotics applications.
//
// Features:
// - Square root operations for distances and other units
// - Power operations (squared, cubed, nth power)
// - Interpolation methods (linear, cubic, hermite)
// - Ramp/slew rate limiting for smooth motion
// - Statistical functions (mean, variance, std deviation)
// - Numerical utilities (sign, wrap, deadband)
//
// Requirements: C++11 or newer
// Dependencies: units_core.h, units_physics.h
// ============================================================================

#ifndef UNITS_MATH_H
#define UNITS_MATH_H

#include "units_core.h"
#include "units_physics.h"
#include <cmath>
#include <vector>
#include <algorithm>

namespace units {

// ============================================================================
// SQUARE ROOT OPERATIONS
// ============================================================================
// Note: Square root of unit types requires careful handling of dimensions
// sqrt(Distance²) = Distance
// sqrt(Area) = Distance

/// Square root of squared distance (returns Distance)
template<typename Ratio>
inline Distance<Ratio> sqrt(const Distance<Ratio>& d) {
    return Distance<Ratio>::fromMeters(std::sqrt(d.toMeters()));
}

/// Square root of area (returns meters)
inline Meters sqrtArea(double areaSquareMeters) {
    return m(std::sqrt(areaSquareMeters));
}

/// Distance from squared distance (d = √(x² + y²))
template<typename Ratio>
inline Distance<Ratio> distanceFromSquared(double squaredMeters) {
    return Distance<Ratio>::fromMeters(std::sqrt(squaredMeters));
}

// ============================================================================
// POWER OPERATIONS
// ============================================================================

/// Square a distance (returns area in m²)
template<typename Ratio>
inline double squared(const Distance<Ratio>& d) {
    double m = d.toMeters();
    return m * m;
}

/// Square a velocity (returns m²/s²)
template<typename Ratio>
inline double squared(const Velocity<Ratio>& v) {
    double mps = v.toMetersPerSecond();
    return mps * mps;
}

/// Square a time (returns s²)
template<typename Ratio>
inline double squared(const Time<Ratio>& t) {
    double s = t.toSeconds();
    return s * s;
}

/// Cube a distance (returns volume in m³)
template<typename Ratio>
inline double cubed(const Distance<Ratio>& d) {
    double m = d.toMeters();
    return m * m * m;
}

/// Absolute value
template<typename Ratio>
inline Distance<Ratio> abs(const Distance<Ratio>& d) {
    return Distance<Ratio>::fromMeters(std::abs(d.toMeters()));
}

template<typename Ratio>
inline Velocity<Ratio> abs(const Velocity<Ratio>& v) {
    return Velocity<Ratio>::fromMetersPerSecond(std::abs(v.toMetersPerSecond()));
}

template<typename AngleRatio>
inline Angle<AngleRatio> abs(const Angle<AngleRatio>& a) {
    return Angle<AngleRatio>::fromRadians(std::abs(a.toRadians()));
}

// ============================================================================
// SIGN FUNCTION
// ============================================================================

/// Sign function: returns -1, 0, or +1
template<typename T>
inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

/// Sign function returning double
template<typename T>
inline double signDouble(T val) {
    return static_cast<double>(sign(val));
}

// ============================================================================
// INTERPOLATION FUNCTIONS
// ============================================================================

/// Linear interpolation between two values
/// @param t Interpolation parameter [0, 1]
/// @return a when t=0, b when t=1
template<typename T>
inline T lerp(const T& a, const T& b, double t) {
    return a + (b - a) * t;
}

/// Inverse lerp: find t such that lerp(a, b, t) = value
/// @return Parameter t, or 0 if a == b
inline double inverseLerp(double a, double b, double value) {
    if (std::abs(b - a) < 1e-10) return 0.0;
    return (value - a) / (b - a);
}

/// Cubic Hermite interpolation (smooth interpolation with velocity control)
/// @param p0 Start value
/// @param v0 Start velocity (derivative)
/// @param p1 End value
/// @param v1 End velocity (derivative)
/// @param t Interpolation parameter [0, 1]
template<typename T>
inline T cubicHermite(const T& p0, const T& v0, const T& p1, const T& v1, double t) {
    double t2 = t * t;
    double t3 = t2 * t;

    // Hermite basis functions
    double h00 = 2*t3 - 3*t2 + 1;
    double h10 = t3 - 2*t2 + t;
    double h01 = -2*t3 + 3*t2;
    double h11 = t3 - t2;

    return p0 * h00 + v0 * h10 + p1 * h01 + v1 * h11;
}

/// Smooth step function (ease in/out)
/// Returns smooth interpolation from 0 to 1 as x goes from 0 to 1
inline double smoothstep(double x) {
    x = numerical::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

/// Smoother step function (even smoother than smoothstep)
inline double smootherstep(double x) {
    x = numerical::clamp(x, 0.0, 1.0);
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
}

// ============================================================================
// RAMP / SLEW RATE LIMITER
// ============================================================================

/// Limits the rate of change of a value (slew rate limiter)
/// Useful for smooth acceleration/deceleration
class RampLimiter {
private:
    double currentValue_;
    double risingRate_;   // Maximum increase per second
    double fallingRate_;  // Maximum decrease per second (positive value)

public:
    /// Constructor
    /// @param risingRate Maximum rate of increase (units/second)
    /// @param fallingRate Maximum rate of decrease (units/second, positive)
    RampLimiter(double risingRate, double fallingRate = -1.0)
        : currentValue_(0.0),
          risingRate_(risingRate),
          fallingRate_(fallingRate < 0 ? risingRate : fallingRate) {}

    /// Update the limiter with a new target value
    /// @param target Desired value
    /// @param dt Time step (seconds)
    /// @return Limited value
    double update(double target, double dt) {
        double delta = target - currentValue_;
        double maxDelta = delta > 0 ? risingRate_ * dt : -fallingRate_ * dt;

        if (std::abs(delta) > std::abs(maxDelta)) {
            currentValue_ += maxDelta;
        } else {
            currentValue_ = target;
        }

        return currentValue_;
    }

    /// Get current value
    double getValue() const { return currentValue_; }

    /// Set current value (for initialization)
    void setValue(double value) { currentValue_ = value; }

    /// Reset to zero
    void reset() { currentValue_ = 0.0; }
};

// ============================================================================
// DEADBAND FUNCTION
// ============================================================================

/// Apply deadband to a value
/// Values within [-threshold, +threshold] become zero
/// Values outside are scaled to start at zero
inline double applyDeadband(double value, double threshold) {
    if (std::abs(value) < threshold) {
        return 0.0;
    }

    if (value > 0) {
        return (value - threshold) / (1.0 - threshold);
    } else {
        return (value + threshold) / (1.0 - threshold);
    }
}

/// Apply deadband without rescaling
inline double applyDeadbandSimple(double value, double threshold) {
    if (std::abs(value) < threshold) {
        return 0.0;
    }
    return value;
}

// ============================================================================
// ANGLE WRAPPING
// ============================================================================

/// Wrap angle to [-PI, +PI] range
template<typename AngleRatio>
inline Angle<AngleRatio> wrapToPi(const Angle<AngleRatio>& angle) {
    double rad = angle.toRadians();
    // Normalize to [-PI, PI]
    while (rad > constants::PI) rad -= constants::TWO_PI;
    while (rad < -constants::PI) rad += constants::TWO_PI;
    return Angle<AngleRatio>::fromRadians(rad);
}

/// Wrap angle to [0, 2*PI] range
template<typename AngleRatio>
inline Angle<AngleRatio> wrapTo2Pi(const Angle<AngleRatio>& angle) {
    double rad = angle.toRadians();
    // Normalize to [0, 2*PI]
    while (rad < 0) rad += constants::TWO_PI;
    while (rad >= constants::TWO_PI) rad -= constants::TWO_PI;
    return Angle<AngleRatio>::fromRadians(rad);
}

/// Compute shortest angular distance from a to b
/// Result is in range [-PI, +PI]
template<typename AngleRatio1, typename AngleRatio2>
inline Radians shortestAngularDistance(const Angle<AngleRatio1>& from,
                                       const Angle<AngleRatio2>& to) {
    double diff = to.toRadians() - from.toRadians();

    // Normalize to [-PI, PI]
    while (diff > constants::PI) diff -= 2.0 * constants::PI;
    while (diff < -constants::PI) diff += 2.0 * constants::PI;

    return rad(diff);
}

// ============================================================================
// STATISTICAL FUNCTIONS
// ============================================================================

/// Calculate mean of a vector of values
template<typename T>
inline double mean(const std::vector<T>& values) {
    if (values.empty()) return 0.0;

    double sum = 0.0;
    for (const auto& v : values) {
        sum += v;
    }
    return sum / values.size();
}

/// Calculate variance of a vector of values
template<typename T>
inline double variance(const std::vector<T>& values) {
    if (values.size() < 2) return 0.0;

    double m = mean(values);
    double sumSquares = 0.0;

    for (const auto& v : values) {
        double diff = v - m;
        sumSquares += diff * diff;
    }

    return sumSquares / (values.size() - 1);  // Sample variance
}

/// Calculate standard deviation
template<typename T>
inline double standardDeviation(const std::vector<T>& values) {
    return std::sqrt(variance(values));
}

/// Calculate median of a vector of values (modifies input!)
template<typename T>
inline T median(std::vector<T>& values) {
    if (values.empty()) return T(0);

    size_t n = values.size();
    std::nth_element(values.begin(), values.begin() + n/2, values.end());

    if (n % 2 == 1) {
        return values[n/2];
    } else {
        T mid1 = values[n/2];
        T mid2 = *std::max_element(values.begin(), values.begin() + n/2);
        return (mid1 + mid2) / 2.0;
    }
}

// ============================================================================
// RANGE MAPPING
// ============================================================================

/// Map value from one range to another
/// Example: map(512, 0, 1023, 0.0, 5.0) → 2.5
inline double map(double value, double fromMin, double fromMax,
                 double toMin, double toMax) {
    return toMin + (value - fromMin) * (toMax - toMin) / (fromMax - fromMin);
}

/// Map value from one range to another with clamping
inline double mapClamped(double value, double fromMin, double fromMax,
                        double toMin, double toMax) {
    double result = map(value, fromMin, fromMax, toMin, toMax);
    return numerical::clamp(result, numerical::min(toMin, toMax), numerical::max(toMin, toMax));
}

// ============================================================================
// HYSTERESIS
// ============================================================================

/// Hysteresis comparator (Schmitt trigger)
/// Prevents oscillation when value is near threshold
class Hysteresis {
private:
    double lowThreshold_;
    double highThreshold_;
    bool state_;

public:
    /// Constructor
    /// @param lowThreshold When value falls below this, output becomes false
    /// @param highThreshold When value rises above this, output becomes true
    Hysteresis(double lowThreshold, double highThreshold)
        : lowThreshold_(lowThreshold),
          highThreshold_(highThreshold),
          state_(false) {
        // Ensure proper ordering
        if (lowThreshold_ > highThreshold_) {
            std::swap(lowThreshold_, highThreshold_);
        }
    }

    /// Update with new value
    /// @return Current state (true if above high threshold, false if below low)
    bool update(double value) {
        if (value > highThreshold_) {
            state_ = true;
        } else if (value < lowThreshold_) {
            state_ = false;
        }
        // If value is between thresholds, state doesn't change
        return state_;
    }

    bool getState() const { return state_; }
    void reset(bool initialState = false) { state_ = initialState; }
};

// ============================================================================
// SATURATION FUNCTION
// ============================================================================

/// Saturation function with different upper and lower limits
template<typename T>
inline T saturate(const T& value, const T& lower, const T& upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/// Symmetric saturation
template<typename T>
inline T saturateSymmetric(const T& value, const T& limit) {
    return saturate(value, -limit, limit);
}

// ============================================================================
// NUMERICAL DERIVATIVES
// ============================================================================

/// Simple numerical derivative (backward difference)
template<typename T>
class NumericalDerivative {
private:
    T lastValue_;
    bool initialized_;

public:
    NumericalDerivative() : lastValue_(T(0)), initialized_(false) {}

    /// Calculate derivative
    /// @param value Current value
    /// @param dt Time step
    /// @return Rate of change (value/time)
    T calculate(const T& value, double dt) {
        if (!initialized_) {
            lastValue_ = value;
            initialized_ = true;
            return T(0);
        }

        if (dt <= 0) return T(0);

        T derivative = (value - lastValue_) / dt;
        lastValue_ = value;
        return derivative;
    }

    void reset() {
        initialized_ = false;
        lastValue_ = T(0);
    }
};

// ============================================================================
// FIRST-ORDER LAG FILTER
// ============================================================================

/// First-order lag filter (exponential smoothing)
/// Similar to low-pass filter but formulated differently
template<typename T>
class FirstOrderLag {
private:
    T output_;
    double timeConstant_;
    bool initialized_;

public:
    explicit FirstOrderLag(double timeConstant)
        : output_(T(0)), timeConstant_(timeConstant), initialized_(false) {}

    /// Update filter
    /// @param input New input value
    /// @param dt Time step
    /// @return Filtered output
    T update(const T& input, double dt) {
        if (!initialized_) {
            output_ = input;
            initialized_ = true;
            return output_;
        }

        // First-order differential equation: dy/dt = (input - y) / tau
        double alpha = dt / (timeConstant_ + dt);
        output_ = output_ + alpha * (input - output_);
        return output_;
    }

    T getValue() const { return output_; }
    void reset() { initialized_ = false; }
    void setValue(const T& value) { output_ = value; initialized_ = true; }
};

} // namespace units

#endif // UNITS_MATH_H

/*
Usage Examples:

1. Square root of distance:
   auto area = squared(m(5));  // 25 m²
   auto dist = sqrtArea(area);  // 5 m

2. Interpolation:
   auto pos = lerp(m(0), m(10), 0.5);  // 5 m
   auto smooth = smoothstep(0.5);      // Smooth curve

3. Ramp limiting (smooth acceleration):
   RampLimiter accelLimit(2.0);  // Max 2 m/s² change
   auto limitedSpeed = accelLimit.update(targetSpeed, dt);

4. Angle wrapping:
   auto normalized = wrapToPi(deg(450));  // Wraps to 90°
   auto diff = shortestAngularDistance(deg(10), deg(350));  // -20°

5. Statistical analysis:
   std::vector<double> measurements = {1.0, 2.0, 3.0, 4.0, 5.0};
   double avg = mean(measurements);
   double stddev = standardDeviation(measurements);

6. Hysteresis (prevent oscillation):
   Hysteresis batteryLow(3.2, 3.4);  // Low at 3.2V, OK at 3.4V
   bool isLow = batteryLow.update(voltage);

Extensions:
- Add Bezier curves for trajectory planning
- Add FFT for frequency analysis
- Add more statistical functions (correlation, etc.)
- Add optimization routines (gradient descent, etc.)
*/
