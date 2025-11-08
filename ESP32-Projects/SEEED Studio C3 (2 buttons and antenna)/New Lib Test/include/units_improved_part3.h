// ============================================================================
// units_improved_part3.h - Helper Functions, Control Systems & Robotics
// ============================================================================

#ifndef UNITS_PART3_H
#define UNITS_PART3_H

#include "units_improved.h"
#include "units_improved_part2.h"

namespace units {

// ============================================================================
// HELPER FUNCTIONS (Improved with better type safety)
// ============================================================================
namespace helpers {
    
    // Distance helpers
    inline constexpr Meters m(double meters) { return Meters::fromMeters(meters); }
    inline constexpr Meters meters(double meters) { return Meters::fromMeters(meters); }
    inline constexpr Feet ft(double feet) { return Feet::fromFeet(feet); }
    inline constexpr Feet feet(double feet) { return Feet::fromFeet(feet); }
    inline constexpr Inches in(double inches) { return Inches::fromInches(inches); }
    inline constexpr Inches inches(double inches) { return Inches::fromInches(inches); }
    inline constexpr Centimeters cm(double cm) { return Centimeters::fromCentimeters(cm); }
    inline constexpr Centimeters centimeters(double cm) { return Centimeters::fromCentimeters(cm); }
    inline constexpr Millimeters mm(double mm) { return Millimeters::fromMillimeters(mm); }
    inline constexpr Millimeters millimeters(double mm) { return Millimeters::fromMillimeters(mm); }
    inline constexpr Kilometers km(double km) { return Kilometers::fromKilometers(km); }
    inline constexpr Kilometers kilometers(double km) { return Kilometers::fromKilometers(km); }
    inline constexpr Miles mi(double miles) { return Miles::fromMiles(miles); }
    inline constexpr Miles miles(double miles) { return Miles::fromMiles(miles); }
    
    // Time helpers
    inline constexpr Seconds s(double seconds) { return Seconds::fromSeconds(seconds); }
    inline constexpr Seconds seconds(double seconds) { return Seconds::fromSeconds(seconds); }
    inline constexpr Milliseconds ms(double ms) { return Milliseconds::fromMilliseconds(ms); }
    inline constexpr Milliseconds milliseconds(double ms) { return Milliseconds::fromMilliseconds(ms); }
    inline constexpr Microseconds us(double us) { return Microseconds::fromMicroseconds(us); }
    inline constexpr Microseconds microseconds(double us) { return Microseconds::fromMicroseconds(us); }
    inline constexpr Minutes min(double minutes) { return Minutes::fromMinutes(minutes); }
    inline constexpr Minutes minutes(double minutes) { return Minutes::fromMinutes(minutes); }
    inline constexpr Hours hr(double hours) { return Hours::fromHours(hours); }
    inline constexpr Hours hours(double hours) { return Hours::fromHours(hours); }
    
    // Angle helpers
    inline constexpr Radians rad(double radians) { return Radians::fromRadians(radians); }
    inline constexpr Radians radians(double radians) { return Radians::fromRadians(radians); }
    inline constexpr Degrees deg(double degrees) { return Degrees::fromDegrees(degrees); }
    inline constexpr Degrees degrees(double degrees) { return Degrees::fromDegrees(degrees); }
    inline constexpr Rotations rot(double rotations) { return Rotations::fromRotations(rotations); }
    inline constexpr Rotations rotations(double rotations) { return Rotations::fromRotations(rotations); }
    
    // Velocity helpers
    inline constexpr MetersPerSecond mps(double mps) { return MetersPerSecond::fromMetersPerSecond(mps); }
    inline constexpr KilometersPerHour kph(double kph) { return KilometersPerHour::fromKilometersPerHour(kph); }
    inline constexpr KilometersPerHour kmh(double kmh) { return KilometersPerHour::fromKilometersPerHour(kmh); }
    inline constexpr MilesPerHour mph(double mph) { return MilesPerHour::fromMilesPerHour(mph); }
    inline constexpr FeetPerSecond fps(double fps) { return FeetPerSecond::fromFeetPerSecond(fps); }
    inline constexpr Knots kts(double knots) { return Knots::fromKnots(knots); }
    
    // Mass helpers
    inline constexpr Kilograms kg(double kg) { return Kilograms::fromKilograms(kg); }
    inline constexpr Grams g(double g) { return Grams::fromGrams(g); }
    inline constexpr Pounds lb(double lb) { return Pounds::fromPounds(lb); }
    inline constexpr Ounces oz(double oz) { return Ounces::fromOunces(oz); }
    
    // Force helpers
    inline constexpr Newtons N(double n) { return Newtons::fromNewtons(n); }
    inline constexpr Pounds lbf(double lbf) { return Pounds::fromPounds(lbf); }
    
    // Torque helpers
    inline constexpr NewtonMeters Nm(double nm) { return NewtonMeters::fromNewtonMeters(nm); }
    inline constexpr PoundFeet lbft(double lbft) { return PoundFeet::fromPoundFeet(lbft); }
    
    // Power helpers
    inline constexpr Watts W(double w) { return Watts::fromWatts(w); }
    inline constexpr Kilowatts kW(double kw) { return Kilowatts::fromKilowatts(kw); }
    inline constexpr Horsepower hp(double hp) { return Horsepower::fromHorsepower(hp); }
    
    // Electrical helpers
    inline constexpr Volts V(double v) { return Volts::fromVolts(v); }
    inline constexpr Millivolts mV(double mv) { return Millivolts::fromMillivolts(mv); }
    inline constexpr Amperes A(double a) { return Amperes::fromAmperes(a); }
    inline constexpr Milliamperes mA(double ma) { return Milliamperes::fromMilliamperes(ma); }
    inline constexpr Ohms ohm(double ohms) { return Ohms::fromOhms(ohms); }
    inline constexpr Kiloohms kohm(double kohms) { return Kiloohms::fromKiloohms(kohms); }
    
    // Temperature helpers
    inline constexpr Celsius degC(double c) { return Celsius::fromCelsius(c); }
    inline constexpr Fahrenheit degF(double f) { return Fahrenheit::fromFahrenheit(f); }
    inline constexpr Kelvin K(double k) { return Kelvin::fromKelvin(k); }
    
    // Scalar helpers
    inline constexpr Scalar scalar(double value) { return Scalar::fromValue(value); }
    inline constexpr Scalar percent(double percent) { return Scalar::fromPercent(percent); }
    inline constexpr Scalar ratio(double ratio) { return Scalar::fromRatio(ratio); }
    inline constexpr Scalar db(double db) { return Scalar::fromDecibels(db); }
}

// Import helpers into units namespace
using namespace helpers;

// ============================================================================
// 2D VECTOR (Enhanced)
// ============================================================================
struct Vec2D {
    double x, y;
    
    constexpr Vec2D() : x(0), y(0) {}
    constexpr Vec2D(double x, double y) : x(x), y(y) {}
    
    // Polar coordinates
    static Vec2D fromPolar(double magnitude, const Radians& angle) {
        return Vec2D(magnitude * angle.cos(), magnitude * angle.sin());
    }
    
    // Basic operations
    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    constexpr double magnitudeSquared() const {
        return x * x + y * y;
    }
    
    Vec2D normalized() const {
        double mag = magnitude();
        return numerical::isZero(mag) ? Vec2D(0, 0) : Vec2D(x / mag, y / mag);
    }
    
    constexpr double dot(const Vec2D& other) const {
        return x * other.x + y * other.y;
    }
    
    constexpr double cross(const Vec2D& other) const {
        return x * other.y - y * other.x;
    }
    
    Vec2D rotate(const Radians& angle) const {
        double c, s;
        angle.sincos(s, c);
        return Vec2D(x * c - y * s, x * s + y * c);
    }
    
    Radians angleTo(const Vec2D& other) const {
        return Radians::atan2(other.y - y, other.x - x);
    }
    
    Radians angle() const {
        return Radians::atan2(y, x);
    }
    
    double distanceTo(const Vec2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    constexpr double distanceSquaredTo(const Vec2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return dx * dx + dy * dy;
    }
    
    Vec2D project(const Vec2D& onto) const {
        double d = onto.dot(onto);
        return numerical::isZero(d) ? Vec2D(0, 0) : onto * (this->dot(onto) / d);
    }
    
    Vec2D reflect(const Vec2D& normal) const {
        return *this - normal * (2.0 * this->dot(normal));
    }
    
    Vec2D lerp(const Vec2D& target, double t) const {
        return Vec2D(x + (target.x - x) * t, y + (target.y - y) * t);
    }
    
    // Operators
    constexpr Vec2D operator+(const Vec2D& other) const { return Vec2D(x + other.x, y + other.y); }
    constexpr Vec2D operator-(const Vec2D& other) const { return Vec2D(x - other.x, y - other.y); }
    constexpr Vec2D operator*(double scalar) const { return Vec2D(x * scalar, y * scalar); }
    constexpr Vec2D operator/(double scalar) const { 
        return Vec2D(numerical::safeDivide(x, scalar), numerical::safeDivide(y, scalar)); 
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
        return numerical::approxEqual(x, other.x) && numerical::approxEqual(y, other.y);
    }
    constexpr bool operator!=(const Vec2D& other) const { return !(*this == other); }
};

// ============================================================================
// POSE2D (Enhanced)
// ============================================================================
struct Pose2D {
    Vec2D position;
    Radians theta;
    
    constexpr Pose2D() : position(), theta() {}
    constexpr Pose2D(double x, double y, const Radians& theta) : position(x, y), theta(theta) {}
    constexpr Pose2D(const Vec2D& pos, const Radians& theta) : position(pos), theta(theta) {}
    
    // Transform operations
    Vec2D toGlobal(const Vec2D& local) const {
        double c, s;
        theta.sincos(s, c);
        return Vec2D(
            position.x + local.x * c - local.y * s,
            position.y + local.x * s + local.y * c
        );
    }
    
    Vec2D toLocal(const Vec2D& global) const {
        Vec2D diff = global - position;
        double c, s;
        (-theta).sincos(s, c);
        return Vec2D(diff.x * c - diff.y * s, diff.x * s + diff.y * c);
    }
    
    Pose2D operator*(const Pose2D& other) const {
        return Pose2D(toGlobal(other.position), theta + other.theta);
    }
    
    Pose2D inverse() const {
        double c, s;
        (-theta).sincos(s, c);
        return Pose2D(
            -(position.x * c - position.y * s),
            -(position.x * s + position.y * c),
            -theta
        );
    }
    
    Pose2D relativeTo(const Pose2D& target) const {
        return inverse() * target;
    }
    
    double distanceTo(const Pose2D& other) const {
        return position.distanceTo(other.position);
    }
    
    Pose2D lerp(const Pose2D& target, double t) const {
        Radians angleDiff = (target.theta - theta).normalizeSigned();
        return Pose2D(
            position.lerp(target.position, t),
            theta + angleDiff * t
        );
    }
    
    Pose2D exp(double delta) const {
        // Exponential map for SE(2)
        if (numerical::isZero(theta.toRadians())) {
            return Pose2D(position + Vec2D(delta, 0).rotate(theta), theta);
        }
        double r = delta / theta.toRadians();
        Vec2D translation(r * theta.sin(), r * (1.0 - theta.cos()));
        return *this * Pose2D(translation, theta * delta);
    }
};

// ============================================================================
// PID CONTROLLER (Enhanced with anti-windup and derivative filtering)
// ============================================================================
class PIDController {
public:
    struct Gains {
        double kP, kI, kD;
        double kF;  // Feedforward gain
        double iMax;  // Integral windup limit
        double outputMin, outputMax;  // Output saturation
        
        constexpr Gains(double p, double i, double d) 
            : kP(p), kI(i), kD(d), kF(0), 
              iMax(std::numeric_limits<double>::max()),
              outputMin(-std::numeric_limits<double>::max()),
              outputMax(std::numeric_limits<double>::max()) {}
    };
    
private:
    Gains gains;
    double prevError;
    double integral;
    double prevDerivative;
    double alpha;  // Derivative filter coefficient
    bool initialized;
    
public:
    explicit PIDController(const Gains& g, double derivativeFilter = 0.9)
        : gains(g), prevError(0), integral(0), prevDerivative(0),
          alpha(derivativeFilter), initialized(false) {}
    
    explicit PIDController(double kP, double kI, double kD)
        : PIDController(Gains(kP, kI, kD)) {}
    
    double calculate(double error, double dt, double feedforward = 0) {
        if (!initialized) {
            prevError = error;
            prevDerivative = 0;
            initialized = true;
        }
        
        // Proportional term
        double p = gains.kP * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        if (integral > gains.iMax) integral = gains.iMax;
        if (integral < -gains.iMax) integral = -gains.iMax;
        double i = gains.kI * integral;
        
        // Derivative term with filtering
        double rawDerivative = numerical::safeDivide(error - prevError, dt);
        double derivative = alpha * prevDerivative + (1.0 - alpha) * rawDerivative;
        double d = gains.kD * derivative;
        
        prevError = error;
        prevDerivative = derivative;
        
        // Calculate total output
        double output = p + i + d + gains.kF * feedforward;
        
        // Apply saturation
        output = numerical::clamp(output, gains.outputMin, gains.outputMax);
        
        // Back-calculation anti-windup
        if (output != p + i + d + gains.kF * feedforward) {
            // Output was saturated, prevent integral windup
            integral -= error * dt * 0.5;  // Reduce integral accumulation
        }
        
        return output;
    }
    
    void reset() {
        prevError = 0;
        integral = 0;
        prevDerivative = 0;
        initialized = false;
    }
    
    void setGains(const Gains& g) { gains = g; }
    const Gains& getGains() const { return gains; }
    double getIntegral() const { return integral; }
    
    void setIntegralLimit(double limit) { gains.iMax = limit; }
    void setOutputLimits(double min, double max) {
        gains.outputMin = min;
        gains.outputMax = max;
    }
};

// ============================================================================
// FEEDFORWARD CONTROLLER
// ============================================================================
class FeedforwardController {
public:
    struct Model {
        double kS;  // Static friction (voltage to overcome stiction)
        double kV;  // Velocity constant (voltage per unit velocity)
        double kA;  // Acceleration constant (voltage per unit acceleration)
        double kG;  // Gravity compensation (for arms/elevators)
        
        constexpr Model(double s = 0, double v = 0, double a = 0, double g = 0)
            : kS(s), kV(v), kA(a), kG(g) {}
    };
    
private:
    Model model;
    
public:
    explicit FeedforwardController(const Model& m) : model(m) {}
    
    FeedforwardController(double kS, double kV, double kA = 0, double kG = 0)
        : model(kS, kV, kA, kG) {}
    
    double calculate(double velocity, double acceleration = 0, 
                    const Radians& angle = Radians()) const {
        double sign = velocity > 0 ? 1.0 : (velocity < 0 ? -1.0 : 0.0);
        return model.kS * sign + 
               model.kV * velocity + 
               model.kA * acceleration + 
               model.kG * angle.cos();
    }
    
    void setModel(const Model& m) { model = m; }
    const Model& getModel() const { return model; }
};

// ============================================================================
// TRAPEZOID PROFILE GENERATOR (Enhanced)
// ============================================================================
class TrapezoidProfile {
public:
    struct Constraints {
        double maxVelocity;
        double maxAcceleration;
        double maxJerk;  // Optional jerk limiting
        
        Constraints(double v, double a, double j = std::numeric_limits<double>::max())
            : maxVelocity(std::abs(v)), maxAcceleration(std::abs(a)), maxJerk(std::abs(j)) {}
    };
    
    struct State {
        double position;
        double velocity;
        double acceleration;
        
        State(double p = 0, double v = 0, double a = 0)
            : position(p), velocity(v), acceleration(a) {}
    };
    
private:
    Constraints constraints;
    State initialState;
    State goalState;
    double startTime;
    double endTime;
    double accelTime;
    double cruiseTime;
    double decelTime;
    
public:
    TrapezoidProfile(const Constraints& c, const State& goal, const State& initial = State())
        : constraints(c), initialState(initial), goalState(goal) {
        calculate();
    }
    
    State calculate(double t) const {
        if (t <= 0) return initialState;
        if (t >= endTime) return goalState;
        
        double distance = goalState.position - initialState.position;
        double direction = distance >= 0 ? 1.0 : -1.0;
        
        if (t < accelTime) {
            // Acceleration phase
            double a = direction * constraints.maxAcceleration;
            double v = initialState.velocity + a * t;
            double p = initialState.position + initialState.velocity * t + 0.5 * a * t * t;
            return State(p, v, a);
        } else if (t < accelTime + cruiseTime) {
            // Cruise phase
            double dt = t - accelTime;
            double v = direction * constraints.maxVelocity;
            double accelDist = 0.5 * constraints.maxAcceleration * accelTime * accelTime;
            double p = initialState.position + direction * accelDist + v * dt;
            return State(p, v, 0);
        } else {
            // Deceleration phase
            double dt = t - accelTime - cruiseTime;
            double a = -direction * constraints.maxAcceleration;
            double v = direction * constraints.maxVelocity + a * dt;
            double accelDist = 0.5 * constraints.maxAcceleration * accelTime * accelTime;
            double cruiseDist = constraints.maxVelocity * cruiseTime;
            double p = initialState.position + direction * (accelDist + cruiseDist) + 
                      direction * constraints.maxVelocity * dt + 0.5 * a * dt * dt;
            return State(p, v, a);
        }
    }
    
    double totalTime() const { return endTime; }
    bool isFinished(double t) const { return t >= endTime; }
    
private:
    void calculate() {
        double distance = std::abs(goalState.position - initialState.position);
        
        // Check if we can reach max velocity
        double accelDist = constraints.maxVelocity * constraints.maxVelocity / 
                          (2.0 * constraints.maxAcceleration);
        
        if (2.0 * accelDist > distance) {
            // Triangular profile (no cruise)
            accelTime = std::sqrt(distance / constraints.maxAcceleration);
            decelTime = accelTime;
            cruiseTime = 0;
        } else {
            // Trapezoidal profile
            accelTime = constraints.maxVelocity / constraints.maxAcceleration;
            decelTime = accelTime;
            cruiseTime = (distance - 2.0 * accelDist) / constraints.maxVelocity;
        }
        
        endTime = accelTime + cruiseTime + decelTime;
        startTime = 0;
    }
};

// ============================================================================
// LOW-PASS FILTER (Enhanced with multiple types)
// ============================================================================
class LowPassFilter {
private:
    double alpha;
    double output;
    bool initialized;
    
public:
    explicit LowPassFilter(double alpha = 0.9)
        : alpha(numerical::clamp(alpha, 0.0, 1.0)), output(0), initialized(false) {}
    
    double update(double input) {
        if (!initialized) {
            output = input;
            initialized = true;
        } else {
            output = alpha * output + (1.0 - alpha) * input;
        }
        return output;
    }
    
    double getValue() const { return output; }
    void reset() { output = 0; initialized = false; }
    void setAlpha(double a) { alpha = numerical::clamp(a, 0.0, 1.0); }
    double getAlpha() const { return alpha; }
    
    // Calculate alpha from cutoff frequency
    static double alphaFromCutoff(double cutoffHz, double sampleRate) {
        double rc = 1.0 / (constants::TWO_PI * cutoffHz);
        double dt = 1.0 / sampleRate;
        return dt / (rc + dt);
    }
};

// ============================================================================
// MOVING AVERAGE FILTER
// ============================================================================
template<size_t WindowSize>
class MovingAverageFilter {
private:
    std::array<double, WindowSize> buffer;
    size_t index;
    double sum;
    size_t count;
    
public:
    MovingAverageFilter() : index(0), sum(0), count(0) {
        buffer.fill(0);
    }
    
    double update(double value) {
        if (count < WindowSize) {
            buffer[count] = value;
            sum += value;
            count++;
        } else {
            sum -= buffer[index];
            buffer[index] = value;
            sum += value;
            index = (index + 1) % WindowSize;
        }
        
        return getValue();
    }
    
    double getValue() const {
        return count > 0 ? sum / count : 0;
    }
    
    void reset() {
        buffer.fill(0);
        index = 0;
        sum = 0;
        count = 0;
    }
    
    size_t getWindowSize() const { return WindowSize; }
    size_t getSampleCount() const { return count; }
};

// ============================================================================
// RATE LIMITER
// ============================================================================
class RateLimiter {
private:
    double maxRate;
    double previousValue;
    bool initialized;
    
public:
    explicit RateLimiter(double rate)
        : maxRate(std::abs(rate)), previousValue(0), initialized(false) {}
    
    double calculate(double input, double dt) {
        if (!initialized) {
            previousValue = input;
            initialized = true;
            return input;
        }
        
        double delta = input - previousValue;
        double maxDelta = maxRate * dt;
        
        if (delta > maxDelta) {
            previousValue += maxDelta;
        } else if (delta < -maxDelta) {
            previousValue -= maxDelta;
        } else {
            previousValue = input;
        }
        
        return previousValue;
    }
    
    void reset() {
        previousValue = 0;
        initialized = false;
    }
    
    void setMaxRate(double rate) { maxRate = std::abs(rate); }
    double getMaxRate() const { return maxRate; }
    double getValue() const { return previousValue; }
};

} // namespace units

#endif // UNITS_PART3_H