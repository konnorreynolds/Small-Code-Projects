// ============================================================================
// Professional Robotics Units Library v2.0 (Enhanced)
// ============================================================================
// A comprehensive, type-safe units system with integrated robotics control.
// Improvements in v2.0:
// - Better constexpr support for C++14/17/20
// - Enhanced error handling and safety features
// - Improved numerical stability
// - Added more unit types and conversions
// - Better template metaprogramming
// - Thread-safe operations where needed
// - More comprehensive documentation
//
// Platform: ESP32, Arduino, Raspberry Pi, STM32, any C++11+ compatible
// License: MIT
// ============================================================================

#ifndef UNITS_H
#define UNITS_H

// Version macros
#define UNITS_VERSION_MAJOR 2
#define UNITS_VERSION_MINOR 0
#define UNITS_VERSION_PATCH 0

// Standard C++ headers
#include <ratio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>
#include <array>
#include <memory>
#include <chrono>

// Platform detection and configuration
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
    #define UNITS_EMBEDDED_PLATFORM 1
    #ifndef M_PI
        #define M_PI 3.14159265358979323846
    #endif
    #ifndef M_E
        #define M_E 2.71828182845904523536
    #endif
#else
    #define UNITS_EMBEDDED_PLATFORM 0
#endif

// C++ version detection
#if __cplusplus >= 202002L
    #define UNITS_CPP20 1
    #define UNITS_CONSTEXPR constexpr
    #define UNITS_CONSTEXPR14 constexpr
    #define UNITS_CONSTEXPR17 constexpr
    #define UNITS_CONSTEXPR20 constexpr
#elif __cplusplus >= 201703L
    #define UNITS_CPP17 1
    #define UNITS_CONSTEXPR constexpr
    #define UNITS_CONSTEXPR14 constexpr
    #define UNITS_CONSTEXPR17 constexpr
    #define UNITS_CONSTEXPR20
#elif __cplusplus >= 201402L
    #define UNITS_CPP14 1
    #define UNITS_CONSTEXPR constexpr
    #define UNITS_CONSTEXPR14 constexpr
    #define UNITS_CONSTEXPR17
    #define UNITS_CONSTEXPR20
#else
    #define UNITS_CPP11 1
    #define UNITS_CONSTEXPR constexpr
    #define UNITS_CONSTEXPR14
    #define UNITS_CONSTEXPR17
    #define UNITS_CONSTEXPR20
#endif

namespace units {

// ============================================================================
// MATHEMATICAL CONSTANTS (High Precision)
// ============================================================================
namespace constants {
    //constexpr double PI = 3.14159265358979323846264338327950288;
    //constexpr double TWO_PI = 6.28318530717958647692528676655900577;
    //constexpr double HALF_PI = 1.57079632679489661923132169163975144;
    constexpr double E = 2.71828182845904523536028747135266250;
    constexpr double SQRT2 = 1.41421356237309504880168872420969808;
    constexpr double SQRT3 = 1.73205080756887729352744634150587237;
    
    // Conversion factors
    //constexpr double DEG_TO_RAD = PI / 180.0;
    //constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double FEET_TO_METERS = 0.3048;
    constexpr double INCHES_TO_METERS = 0.0254;
    constexpr double METERS_TO_FEET = 3.280839895013123;
    constexpr double METERS_TO_INCHES = 39.37007874015748;
    
    // Physical constants
    constexpr double GRAVITY_EARTH = 9.80665;  // m/s² (standard gravity)
    constexpr double GRAVITY_MOON = 1.625;     // m/s²
    constexpr double GRAVITY_MARS = 3.721;     // m/s²
}

// ============================================================================
// NUMERICAL UTILITIES
// ============================================================================
namespace numerical {
    
    // Epsilon for floating point comparisons
    template<typename T>
    constexpr T epsilon() {
        return std::numeric_limits<T>::epsilon();
    }
    
    // Safe floating point comparison
    template<typename T>
    constexpr bool approxEqual(T a, T b, T tolerance = epsilon<T>() * 100) {
        return std::abs(a - b) < tolerance;
    }
    
    // Check if value is effectively zero
    template<typename T>
    constexpr bool isZero(T value, T tolerance = epsilon<T>() * 100) {
        return std::abs(value) < tolerance;
    }
    
    // Safe division with zero check
    template<typename T>
    constexpr T safeDivide(T numerator, T denominator, T defaultValue = T(0)) {
        return isZero(denominator) ? defaultValue : numerator / denominator;
    }
    
    // Constexpr abs for all C++ versions
    template<typename T>
    constexpr T abs(T value) {
        return value < T(0) ? -value : value;
    }
    
    // Constexpr min/max for all C++ versions
    template<typename T>
    constexpr T min(T a, T b) {
        return a < b ? a : b;
    }
    
    template<typename T>
    constexpr T max(T a, T b) {
        return a > b ? a : b;
    }
    
    // Constexpr clamp
    template<typename T>
    constexpr T clamp(T value, T low, T high) {
        return value < low ? low : (value > high ? high : value);
    }
}

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
template<typename Ratio> class Distance;
template<typename Ratio> class Time;
template<typename Ratio> class Angle;
template<typename Ratio> class Mass;
template<typename Ratio> class Temperature;
template<typename DistanceRatio, typename TimeRatio> class Velocity;
template<typename DistanceRatio, typename TimeRatio> class Acceleration;
template<typename Ratio> class Frequency;
template<typename Ratio> class Force;
template<typename Ratio> class Energy;
template<typename Ratio> class Power;
template<typename Ratio> class Torque;
template<typename Ratio> class AngularVelocity;
template<typename Ratio> class AngularAcceleration;
template<typename Ratio> class Voltage;
template<typename Ratio> class Current;
template<typename Ratio> class Resistance;
class Scalar;

// ============================================================================
// TRAITS AND METAPROGRAMMING UTILITIES
// ============================================================================
namespace traits {
    
    // Check if type is a units type
    template<typename T>
    struct is_unit : std::false_type {};
    
    template<typename R>
    struct is_unit<Distance<R>> : std::true_type {};
    
    template<typename R>
    struct is_unit<Time<R>> : std::true_type {};
    
    template<typename R>
    struct is_unit<Angle<R>> : std::true_type {};
    
    template<>
    struct is_unit<Scalar> : std::true_type {};
    
    // Enable if for SFINAE
    template<typename T>
    using enable_if_unit_t = typename std::enable_if<is_unit<T>::value, T>::type;
    
    // Ratio arithmetic helpers
    template<typename R1, typename R2>
    using ratio_multiply = std::ratio_multiply<R1, R2>;
    
    template<typename R1, typename R2>
    using ratio_divide = std::ratio_divide<R1, R2>;
    
    template<typename R>
    using ratio_inverse = std::ratio<R::den, R::num>;
}

// ============================================================================
// BASE UNIT CLASS (CRTP Pattern for Code Reuse)
// ============================================================================
template<typename Derived, typename Ratio = std::ratio<1, 1>>
class Unit {
public:
    using ratio = Ratio;
    using value_type = double;
    
    value_type value;
    
    constexpr Unit() : value(0) {}
    constexpr explicit Unit(value_type v) : value(v) {}
    
    // Common arithmetic operations (CRTP)
    constexpr Derived plus(const Derived& other) const {
        return Derived(value + other.value);
    }
    
    constexpr Derived minus(const Derived& other) const {
        return Derived(value - other.value);
    }
    
    constexpr Derived times(value_type scalar) const {
        return Derived(value * scalar);
    }
    
    constexpr Derived div(value_type scalar) const {
        return Derived(numerical::safeDivide(value, scalar));
    }
    
    constexpr Derived negate() const {
        return Derived(-value);
    }
    
    constexpr Derived abs() const {
        return Derived(numerical::abs(value));
    }
    
    // Comparison operations
    constexpr bool isZero(value_type tolerance = numerical::epsilon<value_type>() * 100) const {
        return numerical::isZero(value, tolerance);
    }
    
    constexpr bool isPositive() const {
        return value > 0;
    }
    
    constexpr bool isNegative() const {
        return value < 0;
    }
    
    constexpr bool isFinite() const {
        return std::isfinite(value);
    }
    
    constexpr bool isNaN() const {
        return std::isnan(value);
    }
    
    // Min/max operations
    constexpr Derived min(const Derived& other) const {
        return Derived(numerical::min(value, other.value));
    }
    
    constexpr Derived max(const Derived& other) const {
        return Derived(numerical::max(value, other.value));
    }
    
    constexpr Derived clamp(const Derived& low, const Derived& high) const {
        return Derived(numerical::clamp(value, low.value, high.value));
    }
    
    // Linear interpolation
    constexpr Derived lerp(const Derived& target, value_type t) const {
        return Derived(value + (target.value - value) * t);
    }
    
    // Smooth step interpolation
    constexpr Derived smoothstep(const Derived& target, value_type t) const {
        t = numerical::clamp(t, value_type(0), value_type(1));
        t = t * t * (3 - 2 * t);
        return lerp(target, t);
    }
    
    // Get raw value
    constexpr value_type raw() const { return value; }
    
    // Operators
    constexpr Derived operator+(const Derived& other) const { return plus(other); }
    constexpr Derived operator-(const Derived& other) const { return minus(other); }
    constexpr Derived operator*(value_type scalar) const { return times(scalar); }
    constexpr Derived operator/(value_type scalar) const { return div(scalar); }
    constexpr value_type operator/(const Derived& other) const { 
        return numerical::safeDivide(value, other.value); 
    }
    constexpr Derived operator-() const { return negate(); }
    
    // Mutable operators
    Derived& operator+=(const Derived& other) { value += other.value; return static_cast<Derived&>(*this); }
    Derived& operator-=(const Derived& other) { value -= other.value; return static_cast<Derived&>(*this); }
    Derived& operator*=(value_type scalar) { value *= scalar; return static_cast<Derived&>(*this); }
    Derived& operator/=(value_type scalar) { value = numerical::safeDivide(value, scalar); return static_cast<Derived&>(*this); }
    
    // Comparison operators
    constexpr bool operator==(const Derived& other) const { 
        return numerical::approxEqual(value, other.value); 
    }
    constexpr bool operator!=(const Derived& other) const { return !(*this == other); }
    constexpr bool operator<(const Derived& other) const { return value < other.value; }
    constexpr bool operator<=(const Derived& other) const { return value <= other.value; }
    constexpr bool operator>(const Derived& other) const { return value > other.value; }
    constexpr bool operator>=(const Derived& other) const { return value >= other.value; }
};

// ============================================================================
// DISTANCE UNITS (Enhanced)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Distance : public Unit<Distance<Ratio>, Ratio> {
    using Base = Unit<Distance<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    // Conversion methods
    constexpr double toMeters() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toFeet() const {
        return toMeters() * constants::METERS_TO_FEET;
    }
    
    constexpr double toInches() const {
        return toMeters() * constants::METERS_TO_INCHES;
    }
    
    constexpr double toCentimeters() const {
        return toMeters() * 100.0;
    }
    
    constexpr double toMillimeters() const {
        return toMeters() * 1000.0;
    }
    
    constexpr double toKilometers() const {
        return toMeters() * 0.001;
    }
    
    constexpr double toMiles() const {
        return toMeters() * 0.000621371192237334;
    }
    
    constexpr double toYards() const {
        return toMeters() * 1.0936132983377078;
    }
    
    constexpr double toNauticalMiles() const {
        return toMeters() * 0.000539956803455724;
    }
    
    // Factory methods
    static constexpr Distance fromMeters(double meters) {
        return Distance(meters * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Distance fromFeet(double feet) {
        return fromMeters(feet * constants::FEET_TO_METERS);
    }
    
    static constexpr Distance fromInches(double inches) {
        return fromMeters(inches * constants::INCHES_TO_METERS);
    }
    
    static constexpr Distance fromCentimeters(double cm) {
        return fromMeters(cm * 0.01);
    }
    
    static constexpr Distance fromMillimeters(double mm) {
        return fromMeters(mm * 0.001);
    }
    
    static constexpr Distance fromKilometers(double km) {
        return fromMeters(km * 1000.0);
    }
    
    static constexpr Distance fromMiles(double miles) {
        return fromMeters(miles * 1609.344);
    }
    
    static constexpr Distance fromYards(double yards) {
        return fromMeters(yards * 0.9144);
    }
    
    static constexpr Distance fromNauticalMiles(double nm) {
        return fromMeters(nm * 1852.0);
    }
};

// Common distance types
using Meters = Distance<std::ratio<1, 1>>;
using Feet = Distance<std::ratio<3048, 10000>>;
using Inches = Distance<std::ratio<254, 10000>>;
using Centimeters = Distance<std::ratio<1, 100>>;
using Millimeters = Distance<std::ratio<1, 1000>>;
using Kilometers = Distance<std::ratio<1000, 1>>;
using Miles = Distance<std::ratio<1609344, 1000>>;
using Yards = Distance<std::ratio<9144, 10000>>;
using NauticalMiles = Distance<std::ratio<1852, 1>>;

// ============================================================================
// TIME UNITS (Enhanced)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Time : public Unit<Time<Ratio>, Ratio> {
    using Base = Unit<Time<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    // Conversion methods
    constexpr double toSeconds() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toMilliseconds() const {
        return toSeconds() * 1000.0;
    }
    
    constexpr double toMicroseconds() const {
        return toSeconds() * 1000000.0;
    }
    
    constexpr double toNanoseconds() const {
        return toSeconds() * 1000000000.0;
    }
    
    constexpr double toMinutes() const {
        return toSeconds() / 60.0;
    }
    
    constexpr double toHours() const {
        return toSeconds() / 3600.0;
    }
    
    constexpr double toDays() const {
        return toSeconds() / 86400.0;
    }
    
    constexpr double toWeeks() const {
        return toSeconds() / 604800.0;
    }
    
    // Factory methods
    static constexpr Time fromSeconds(double seconds) {
        return Time(seconds * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Time fromMilliseconds(double ms) {
        return fromSeconds(ms * 0.001);
    }
    
    static constexpr Time fromMicroseconds(double us) {
        return fromSeconds(us * 0.000001);
    }
    
    static constexpr Time fromNanoseconds(double ns) {
        return fromSeconds(ns * 0.000000001);
    }
    
    static constexpr Time fromMinutes(double minutes) {
        return fromSeconds(minutes * 60.0);
    }
    
    static constexpr Time fromHours(double hours) {
        return fromSeconds(hours * 3600.0);
    }
    
    static constexpr Time fromDays(double days) {
        return fromSeconds(days * 86400.0);
    }
    
    static constexpr Time fromWeeks(double weeks) {
        return fromSeconds(weeks * 604800.0);
    }
    
    // Conversion to std::chrono
#if !UNITS_EMBEDDED_PLATFORM
    template<typename ChronoDuration>
    ChronoDuration toChrono() const {
        using namespace std::chrono;
        return duration_cast<ChronoDuration>(duration<double>(toSeconds()));
    }
    
    template<typename ChronoDuration>
    static Time fromChrono(const ChronoDuration& d) {
        using namespace std::chrono;
        return fromSeconds(duration_cast<duration<double>>(d).count());
    }
#endif
};

// Common time types
using Seconds = Time<std::ratio<1, 1>>;
using Milliseconds = Time<std::ratio<1, 1000>>;
using Microseconds = Time<std::ratio<1, 1000000>>;
using Nanoseconds = Time<std::ratio<1, 1000000000>>;
using Minutes = Time<std::ratio<60, 1>>;
using Hours = Time<std::ratio<3600, 1>>;
using Days = Time<std::ratio<86400, 1>>;
using Weeks = Time<std::ratio<604800, 1>>;

// ============================================================================
// ANGLE UNITS (Enhanced with better normalization)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Angle : public Unit<Angle<Ratio>, Ratio> {
    using Base = Unit<Angle<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    // Conversion methods
    constexpr double toRadians() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toDegrees() const {
        return toRadians() * constants::RAD_TO_DEG;
    }
    
    constexpr double toRotations() const {
        return toRadians() / constants::TWO_PI;
    }
    
    constexpr double toGradians() const {
        return toRadians() * 200.0 / constants::PI;
    }
    
    constexpr double toArcMinutes() const {
        return toDegrees() * 60.0;
    }
    
    constexpr double toArcSeconds() const {
        return toDegrees() * 3600.0;
    }
    
    // Factory methods
    static constexpr Angle fromRadians(double radians) {
        return Angle(radians * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Angle fromDegrees(double degrees) {
        return fromRadians(degrees * constants::DEG_TO_RAD);
    }
    
    static constexpr Angle fromRotations(double rotations) {
        return fromRadians(rotations * constants::TWO_PI);
    }
    
    static constexpr Angle fromGradians(double gradians) {
        return fromRadians(gradians * constants::PI / 200.0);
    }
    
    static constexpr Angle fromArcMinutes(double arcmin) {
        return fromDegrees(arcmin / 60.0);
    }
    
    static constexpr Angle fromArcSeconds(double arcsec) {
        return fromDegrees(arcsec / 3600.0);
    }
    
    // Normalization methods
    Angle normalizePositive() const {
        double rads = toRadians();
        rads = std::fmod(rads, constants::TWO_PI);
        if (rads < 0) rads += constants::TWO_PI;
        return fromRadians(rads);
    }
    
    Angle normalizeSigned() const {
        double rads = toRadians();
        rads = std::fmod(rads + constants::PI, constants::TWO_PI);
        if (rads < 0) rads += constants::TWO_PI;
        return fromRadians(rads - constants::PI);
    }
    
    // Trigonometric functions
    double sin() const { return std::sin(toRadians()); }
    double cos() const { return std::cos(toRadians()); }
    double tan() const { return std::tan(toRadians()); }
    double sec() const { return 1.0 / std::cos(toRadians()); }
    double csc() const { return 1.0 / std::sin(toRadians()); }
    double cot() const { return 1.0 / std::tan(toRadians()); }
    
    // Inverse trigonometric static functions
    static Angle asin(double value) {
        return fromRadians(std::asin(numerical::clamp(value, -1.0, 1.0)));
    }
    
    static Angle acos(double value) {
        return fromRadians(std::acos(numerical::clamp(value, -1.0, 1.0)));
    }
    
    static Angle atan(double value) {
        return fromRadians(std::atan(value));
    }
    
    static Angle atan2(double y, double x) {
        return fromRadians(std::atan2(y, x));
    }
    
    // Efficient sin/cos computation
    void sincos(double& sinOut, double& cosOut) const {
        const double rads = toRadians();
#if defined(__GNUC__) && !UNITS_EMBEDDED_PLATFORM
        __builtin_sincos(rads, &sinOut, &cosOut);
#else
        sinOut = std::sin(rads);
        cosOut = std::cos(rads);
#endif
    }
    
    // Angle difference (shortest path)
    Angle shortestDistanceTo(const Angle& other) const {
        Angle diff = other - *this;
        return diff.normalizeSigned();
    }
    
    // Check if angle is in range [start, end] (considering wrap-around)
    bool isInRange(const Angle& start, const Angle& end) const {
        Angle norm = this->normalizePositive();
        Angle s = start.normalizePositive();
        Angle e = end.normalizePositive();
        
        if (s.value <= e.value) {
            return norm.value >= s.value && norm.value <= e.value;
        } else {
            return norm.value >= s.value || norm.value <= e.value;
        }
    }
};

// Common angle types
using Radians = Angle<std::ratio<1, 1>>;
using Degrees = Angle<std::ratio<31415926535897932, 1800000000000000000>>;  // π/180 high precision
using Rotations = Angle<std::ratio<62831853071795865, 10000000000000000>>;  // 2π high precision
using Gradians = Angle<std::ratio<31415926535897932, 2000000000000000000>>;  // π/200

// ============================================================================
// SCALAR (Enhanced with more operations)
// ============================================================================
class Scalar {
public:
    double value;
    
    constexpr Scalar() : value(0) {}
    constexpr explicit Scalar(double v) : value(v) {}
    
    // Conversion methods
    constexpr double toValue() const { return value; }
    constexpr double toPercent() const { return value * 100.0; }
    constexpr double toRatio() const { return value; }
    constexpr double toDecibels() const { return 20.0 * std::log10(numerical::abs(value)); }
    
    // Factory methods
    static constexpr Scalar fromValue(double val) { return Scalar(val); }
    static constexpr Scalar fromPercent(double percent) { return Scalar(percent * 0.01); }
    static constexpr Scalar fromRatio(double ratio) { return Scalar(ratio); }
    static constexpr Scalar fromDecibels(double db) { return Scalar(std::pow(10.0, db / 20.0)); }
    
    // Constants
    static constexpr Scalar zero() { return Scalar(0); }
    static constexpr Scalar one() { return Scalar(1); }
    static constexpr Scalar pi() { return Scalar(constants::PI); }
    static constexpr Scalar e() { return Scalar(constants::E); }
    
    // Arithmetic operations
    constexpr Scalar operator+(const Scalar& other) const { return Scalar(value + other.value); }
    constexpr Scalar operator-(const Scalar& other) const { return Scalar(value - other.value); }
    constexpr Scalar operator*(const Scalar& other) const { return Scalar(value * other.value); }
    constexpr Scalar operator*(double scalar) const { return Scalar(value * scalar); }
    constexpr Scalar operator/(const Scalar& other) const { 
        return Scalar(numerical::safeDivide(value, other.value)); 
    }
    constexpr Scalar operator/(double scalar) const { 
        return Scalar(numerical::safeDivide(value, scalar)); 
    }
    constexpr Scalar operator-() const { return Scalar(-value); }
    
    // Mathematical functions
    Scalar abs() const { return Scalar(numerical::abs(value)); }
    Scalar sqrt() const { return Scalar(std::sqrt(numerical::abs(value))); }
    Scalar cbrt() const { return Scalar(std::cbrt(value)); }
    Scalar pow(double exponent) const { return Scalar(std::pow(value, exponent)); }
    Scalar square() const { return Scalar(value * value); }
    Scalar cube() const { return Scalar(value * value * value); }
    Scalar reciprocal() const { return Scalar(numerical::safeDivide(1.0, value)); }
    
    // Exponential and logarithmic
    Scalar exp() const { return Scalar(std::exp(value)); }
    Scalar exp2() const { return Scalar(std::exp2(value)); }
    Scalar ln() const { return Scalar(std::log(numerical::abs(value))); }
    Scalar log10() const { return Scalar(std::log10(numerical::abs(value))); }
    Scalar log2() const { return Scalar(std::log2(numerical::abs(value))); }
    
    // Trigonometric (treating value as radians)
    Scalar sin() const { return Scalar(std::sin(value)); }
    Scalar cos() const { return Scalar(std::cos(value)); }
    Scalar tan() const { return Scalar(std::tan(value)); }
    Scalar asin() const { return Scalar(std::asin(numerical::clamp(value, -1.0, 1.0))); }
    Scalar acos() const { return Scalar(std::acos(numerical::clamp(value, -1.0, 1.0))); }
    Scalar atan() const { return Scalar(std::atan(value)); }
    
    // Hyperbolic functions
    Scalar sinh() const { return Scalar(std::sinh(value)); }
    Scalar cosh() const { return Scalar(std::cosh(value)); }
    Scalar tanh() const { return Scalar(std::tanh(value)); }
    Scalar asinh() const { return Scalar(std::asinh(value)); }
    Scalar acosh() const { return Scalar(std::acosh(std::max(value, 1.0))); }
    Scalar atanh() const { return Scalar(std::atanh(numerical::clamp(value, -0.999, 0.999))); }
    
    // Rounding functions
    Scalar floor() const { return Scalar(std::floor(value)); }
    Scalar ceil() const { return Scalar(std::ceil(value)); }
    Scalar round() const { return Scalar(std::round(value)); }
    Scalar trunc() const { return Scalar(std::trunc(value)); }
    
    // Utility functions
    constexpr Scalar min(const Scalar& other) const { 
        return Scalar(numerical::min(value, other.value)); 
    }
    
    constexpr Scalar max(const Scalar& other) const { 
        return Scalar(numerical::max(value, other.value)); 
    }
    
    constexpr Scalar clamp(const Scalar& low, const Scalar& high) const {
        return Scalar(numerical::clamp(value, low.value, high.value));
    }
    
    constexpr Scalar lerp(const Scalar& target, double t) const {
        return Scalar(value + (target.value - value) * t);
    }
    
    Scalar smoothstep(const Scalar& target, double t) const {
        t = numerical::clamp(t, 0.0, 1.0);
        t = t * t * (3.0 - 2.0 * t);
        return lerp(target, t);
    }
    
    // Map from one range to another
    constexpr Scalar map(double inMin, double inMax, double outMin, double outMax) const {
        double t = numerical::safeDivide(value - inMin, inMax - inMin);
        return Scalar(outMin + t * (outMax - outMin));
    }
    
    // Comparison
    constexpr bool isZero(double tolerance = numerical::epsilon<double>() * 100) const { 
        return numerical::isZero(value, tolerance); 
    }
    constexpr bool isPositive() const { return value > 0; }
    constexpr bool isNegative() const { return value < 0; }
    constexpr bool isFinite() const { return std::isfinite(value); }
    constexpr bool isNaN() const { return std::isnan(value); }
    
    // Comparison operators
    constexpr bool operator==(const Scalar& other) const { 
        return numerical::approxEqual(value, other.value); 
    }
    constexpr bool operator!=(const Scalar& other) const { return !(*this == other); }
    constexpr bool operator<(const Scalar& other) const { return value < other.value; }
    constexpr bool operator<=(const Scalar& other) const { return value <= other.value; }
    constexpr bool operator>(const Scalar& other) const { return value > other.value; }
    constexpr bool operator>=(const Scalar& other) const { return value >= other.value; }
    
    // Implicit conversion to double
    constexpr operator double() const { return value; }
};

// Scalar literals (C++11 compatible)
inline Scalar operator""_percent(long double val) { return Scalar::fromPercent(static_cast<double>(val)); }
inline Scalar operator""_percent(unsigned long long val) { return Scalar::fromPercent(static_cast<double>(val)); }
inline Scalar operator""_ratio(long double val) { return Scalar::fromRatio(static_cast<double>(val)); }
inline Scalar operator""_db(long double val) { return Scalar::fromDecibels(static_cast<double>(val)); }

// Reverse multiplication for Scalar
inline Scalar operator*(double scalar, const Scalar& s) { return s * scalar; }

// ============================================================================
// TEMPERATURE UNITS (New)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Temperature : public Unit<Temperature<Ratio>, Ratio> {
    using Base = Unit<Temperature<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    // Note: Temperature uses offset scales, not just ratios
    // Internal storage is in Kelvin
    
    constexpr double toKelvin() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toCelsius() const {
        return toKelvin() - 273.15;
    }
    
    constexpr double toFahrenheit() const {
        return toCelsius() * 9.0/5.0 + 32.0;
    }
    
    constexpr double toRankine() const {
        return toKelvin() * 9.0/5.0;
    }
    
    // Factory methods
    static constexpr Temperature fromKelvin(double kelvin) {
        return Temperature(kelvin * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Temperature fromCelsius(double celsius) {
        return fromKelvin(celsius + 273.15);
    }
    
    static constexpr Temperature fromFahrenheit(double fahrenheit) {
        return fromCelsius((fahrenheit - 32.0) * 5.0/9.0);
    }
    
    static constexpr Temperature fromRankine(double rankine) {
        return fromKelvin(rankine * 5.0/9.0);
    }
    
    // Physical constants
    static constexpr Temperature absoluteZero() { return fromKelvin(0); }
    static constexpr Temperature waterFreezing() { return fromCelsius(0); }
    static constexpr Temperature waterBoiling() { return fromCelsius(100); }
    static constexpr Temperature roomTemperature() { return fromCelsius(20); }
    static constexpr Temperature bodyTemperature() { return fromCelsius(37); }
};

using Kelvin = Temperature<std::ratio<1, 1>>;
using Celsius = Temperature<std::ratio<1, 1>>;  // Stored as Kelvin internally
using Fahrenheit = Temperature<std::ratio<1, 1>>;  // Stored as Kelvin internally
using Rankine = Temperature<std::ratio<5, 9>>;

// ============================================================================
// MASS UNITS (New)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Mass : public Unit<Mass<Ratio>, Ratio> {
    using Base = Unit<Mass<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toKilograms() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toGrams() const {
        return toKilograms() * 1000.0;
    }
    
    constexpr double toMilligrams() const {
        return toKilograms() * 1000000.0;
    }
    
    constexpr double toMetricTons() const {
        return toKilograms() * 0.001;
    }
    
    constexpr double toPounds() const {
        return toKilograms() * 2.20462262185;
    }
    
    constexpr double toOunces() const {
        return toKilograms() * 35.27396195;
    }
    
    constexpr double toStones() const {
        return toKilograms() * 0.157473044;
    }
    
    // Factory methods
    static constexpr Mass fromKilograms(double kg) {
        return Mass(kg * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Mass fromGrams(double g) {
        return fromKilograms(g * 0.001);
    }
    
    static constexpr Mass fromMilligrams(double mg) {
        return fromKilograms(mg * 0.000001);
    }
    
    static constexpr Mass fromMetricTons(double t) {
        return fromKilograms(t * 1000.0);
    }
    
    static constexpr Mass fromPounds(double lb) {
        return fromKilograms(lb * 0.45359237);
    }
    
    static constexpr Mass fromOunces(double oz) {
        return fromKilograms(oz * 0.028349523125);
    }
    
    static constexpr Mass fromStones(double st) {
        return fromKilograms(st * 6.35029318);
    }
};

using Kilograms = Mass<std::ratio<1, 1>>;
using Grams = Mass<std::ratio<1, 1000>>;
using Milligrams = Mass<std::ratio<1, 1000000>>;
using MetricTons = Mass<std::ratio<1000, 1>>;
using Pounds = Mass<std::ratio<45359237, 100000000>>;
using Ounces = Mass<std::ratio<28349523125, 1000000000000>>;
using Stones = Mass<std::ratio<635029318, 100000000>>;

// ============================================================================
// FREQUENCY UNITS (New)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Frequency : public Unit<Frequency<Ratio>, Ratio> {
    using Base = Unit<Frequency<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toHertz() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toKilohertz() const {
        return toHertz() * 0.001;
    }
    
    constexpr double toMegahertz() const {
        return toHertz() * 0.000001;
    }
    
    constexpr double toGigahertz() const {
        return toHertz() * 0.000000001;
    }
    
    constexpr double toRPM() const {
        return toHertz() * 60.0;
    }
    
    constexpr double toRadiansPerSecond() const {
        return toHertz() * constants::TWO_PI;
    }
    
    // Factory methods
    static constexpr Frequency fromHertz(double hz) {
        return Frequency(hz * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Frequency fromKilohertz(double khz) {
        return fromHertz(khz * 1000.0);
    }
    
    static constexpr Frequency fromMegahertz(double mhz) {
        return fromHertz(mhz * 1000000.0);
    }
    
    static constexpr Frequency fromGigahertz(double ghz) {
        return fromHertz(ghz * 1000000000.0);
    }
    
    static constexpr Frequency fromRPM(double rpm) {
        return fromHertz(rpm / 60.0);
    }
    
    static constexpr Frequency fromRadiansPerSecond(double radps) {
        return fromHertz(radps / constants::TWO_PI);
    }
    
    // Convert to period
    Time<std::ratio<1, 1>> toPeriod() const {
        return Time<std::ratio<1, 1>>::fromSeconds(numerical::safeDivide(1.0, toHertz()));
    }
};

using Hertz = Frequency<std::ratio<1, 1>>;
using Kilohertz = Frequency<std::ratio<1000, 1>>;
using Megahertz = Frequency<std::ratio<1000000, 1>>;
using Gigahertz = Frequency<std::ratio<1000000000, 1>>;
using RPM = Frequency<std::ratio<1, 60>>;
using RadiansPerSecond = Frequency<std::ratio<1000000000, 6283185307>>;  // 1/(2π)

// Continue in next part...
} // namespace units

#endif // UNITS_H