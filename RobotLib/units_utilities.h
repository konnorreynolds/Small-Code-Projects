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
struct DifferentialDrive {
    MetersPerSecond leftVelocity;
    MetersPerSecond rightVelocity;

    // Create from chassis velocity
    static DifferentialDrive fromTwist(const MetersPerSecond& linear,
                                      const RadiansPerSecond& angular,
                                      const Meters& wheelbase) {
        double v = linear.toMetersPerSecond();
        double omega = angular.toRadiansPerSecond();
        double L = wheelbase.toMeters();

        double vLeft = v - (omega * L / 2.0);
        double vRight = v + (omega * L / 2.0);

        return DifferentialDrive{
            MetersPerSecond::fromMetersPerSecond(vLeft),
            MetersPerSecond::fromMetersPerSecond(vRight)
        };
    }

    // Get chassis velocity from wheel velocities
    struct Twist {
        MetersPerSecond linear;
        RadiansPerSecond angular;
    };

    Twist toTwist(const Meters& wheelbase) const {
        double vLeft = leftVelocity.toMetersPerSecond();
        double vRight = rightVelocity.toMetersPerSecond();
        double L = wheelbase.toMeters();

        double linear = (vLeft + vRight) / 2.0;
        double angular = (vRight - vLeft) / L;

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
class MotorController {
public:
    // Convert desired RPM to PWM duty cycle (0-1)
    static double rpmToPWM(const RPM& desiredRPM, const RPM& maxRPM) {
        double ratio = desiredRPM.toRPM() / maxRPM.toRPM();
        return numerical::clamp(ratio, -1.0, 1.0);
    }

    // Apply deadband to eliminate stick zone
    static double applyDeadband(double input, double deadband = 0.05) {
        if (std::abs(input) < deadband) {
            return 0.0;
        }

        // Scale the remaining range
        double sign = (input > 0) ? 1.0 : -1.0;
        return sign * (std::abs(input) - deadband) / (1.0 - deadband);
    }

    // Convert linear velocity to wheel RPM
    static RPM velocityToRPM(const MetersPerSecond& velocity,
                            const Meters& wheelDiameter) {
        double v = velocity.toMetersPerSecond();
        double d = wheelDiameter.toMeters();
        double circumference = constants::PI * d;

        // rotations per second = velocity / circumference
        double rps = v / circumference;
        double rpm = rps * 60.0;

        return RPM::fromRPM(rpm);
    }

    // Convert wheel RPM to linear velocity
    static MetersPerSecond rpmToVelocity(const RPM& rpm,
                                        const Meters& wheelDiameter) {
        double rotationsPerSec = rpm.toRPM() / 60.0;
        double d = wheelDiameter.toMeters();
        double circumference = constants::PI * d;

        double velocity = rotationsPerSec * circumference;
        return MetersPerSecond::fromMetersPerSecond(velocity);
    }
};

// ============================================================================
// EXPONENTIAL MOVING AVERAGE FILTER
// ============================================================================
class ExponentialMovingAverage {
private:
    double alpha_;
    double value_;
    bool initialized_;

public:
    // alpha: smoothing factor (0-1), higher = more responsive
    explicit ExponentialMovingAverage(double alpha = 0.1)
        : alpha_(numerical::clamp(alpha, 0.0, 1.0)),
          value_(0.0), initialized_(false) {}

    double update(double newValue) {
        if (!initialized_) {
            value_ = newValue;
            initialized_ = true;
        } else {
            value_ = alpha_ * newValue + (1.0 - alpha_) * value_;
        }
        return value_;
    }

    double getValue() const { return value_; }
    void reset() { value_ = 0.0; initialized_ = false; }

    // Create from time constant and sample rate
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
