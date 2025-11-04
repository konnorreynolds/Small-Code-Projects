// ============================================================================
// units_core.h - Core Units System Foundation
// ============================================================================
// Purpose: Fundamental unit types, type safety, and basic mathematical operations
// Dependencies: None (standalone)
// Platform: C++11 or higher, embedded-friendly
//
// This file contains:
// - Mathematical constants and utilities
// - Base unit template with CRTP
// - Fundamental units (Distance, Time, Angle, Mass, Temperature)
// - Type traits and metaprogramming utilities
// ============================================================================

#ifndef ROBOTICS_UNITS_CORE_H
#define ROBOTICS_UNITS_CORE_H

#include <ratio>
#include <cmath>
#include <limits>
#include <type_traits>

// Version information
#define ROBOTICS_UNITS_VERSION_MAJOR 2
#define ROBOTICS_UNITS_VERSION_MINOR 1
#define ROBOTICS_UNITS_VERSION_PATCH 0

// Platform detection
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM) || defined(STM32)
    #define UNITS_EMBEDDED 1
#else
    #define UNITS_EMBEDDED 0
#endif

// C++ version detection for constexpr support
#if __cplusplus >= 201402L
    #define UNITS_CONSTEXPR14 constexpr
#else
    #define UNITS_CONSTEXPR14
#endif

namespace units {

// ============================================================================
// MATHEMATICAL CONSTANTS
// ============================================================================
namespace constants {
    // Mathematical constants (high precision)
    constexpr double PI = 3.14159265358979323846264338327950288;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double HALF_PI = 0.5 * PI;
    constexpr double E = 2.71828182845904523536028747135266250;
    constexpr double SQRT2 = 1.41421356237309504880168872420969808;
    constexpr double SQRT3 = 1.73205080756887729352744634150587237;

    // Conversion factors (exact where possible)
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double FEET_TO_METERS = 0.3048;  // Exact by definition
    constexpr double INCHES_TO_METERS = 0.0254;  // Exact by definition

    // Physical constants
    constexpr double GRAVITY_EARTH = 9.80665;  // m/s² (standard gravity)
    constexpr double SPEED_OF_LIGHT = 299792458.0;  // m/s (exact)
}

// ============================================================================
// NUMERICAL UTILITIES
// ============================================================================
// Why: These utilities prevent common numerical errors and provide consistent
// behavior across the library. They're all inline/constexpr for zero overhead.
// ============================================================================
namespace numerical {

    // Get machine epsilon for type
    template<typename T>
    inline constexpr T epsilon() {
        return std::numeric_limits<T>::epsilon();
    }

    // Safe floating-point comparison
    // Why: Direct == comparison of floats is unreliable due to rounding
    template<typename T>
    inline constexpr bool approxEqual(T a, T b, T tolerance = epsilon<T>() * 100) {
        return (a - b) < tolerance && (b - a) < tolerance;  // Avoids std::abs in constexpr
    }

    // Check if effectively zero
    template<typename T>
    inline constexpr bool isZero(T value, T tolerance = epsilon<T>() * 100) {
        return value < tolerance && -value < tolerance;
    }

    // Safe division with zero check
    // Why: Prevents undefined behavior from division by zero
    template<typename T>
    inline constexpr T safeDivide(T numerator, T denominator, T defaultValue = T(0)) {
        return isZero(denominator) ? defaultValue : numerator / denominator;
    }

    // Constexpr min/max/clamp for C++11 compatibility
    template<typename T>
    inline constexpr T min(T a, T b) { return a < b ? a : b; }

    template<typename T>
    inline constexpr T max(T a, T b) { return a > b ? a : b; }

    template<typename T>
    inline constexpr T clamp(T value, T low, T high) {
        return value < low ? low : (value > high ? high : value);
    }

    // Constexpr abs for all C++ versions
    template<typename T>
    inline constexpr T abs(T value) { return value < T(0) ? -value : value; }
}

// ============================================================================
// TYPE TRAITS
// ============================================================================
// Why: These traits enable compile-time checking and SFINAE to ensure
// only valid operations are allowed between units.
// ============================================================================
namespace traits {
    // Primary template - not a unit
    template<typename T>
    struct is_unit : std::false_type {};

    // Specializations will be added for each unit type

    // Helper for enabling functions only for unit types
    template<typename T>
    using enable_if_unit_t = typename std::enable_if<is_unit<T>::value>::type;

    // Ratio operations
    template<typename R1, typename R2>
    using ratio_multiply = std::ratio_multiply<R1, R2>;

    template<typename R1, typename R2>
    using ratio_divide = std::ratio_divide<R1, R2>;
}

// ============================================================================
// BASE UNIT CLASS (CRTP)
// ============================================================================
// Why use CRTP (Curiously Recurring Template Pattern)?
// 1. Avoids virtual function overhead (important for embedded)
// 2. Enables compile-time polymorphism
// 3. Allows shared implementation without runtime cost
// 4. Each unit type gets its own instantiation (no type confusion)
// ============================================================================
template<typename Derived, typename Ratio = std::ratio<1, 1>>
class UnitBase {
public:
    using ratio = Ratio;
    using value_type = double;

    value_type value;

    // Constructors
    constexpr UnitBase() : value(0) {}
    constexpr explicit UnitBase(value_type v) : value(v) {}

    // Arithmetic operations return Derived type (CRTP magic)
    constexpr Derived operator+(const Derived& other) const {
        return Derived(value + other.value);
    }

    constexpr Derived operator-(const Derived& other) const {
        return Derived(value - other.value);
    }

    constexpr Derived operator*(value_type scalar) const {
        return Derived(value * scalar);
    }

    constexpr Derived operator/(value_type scalar) const {
        return Derived(numerical::safeDivide(value, scalar));
    }

    constexpr value_type operator/(const Derived& other) const {
        return numerical::safeDivide(value, other.value);
    }

    constexpr Derived operator-() const {
        return Derived(-value);
    }

    // Mutable arithmetic
    Derived& operator+=(const Derived& other) {
        value += other.value;
        return static_cast<Derived&>(*this);
    }

    Derived& operator-=(const Derived& other) {
        value -= other.value;
        return static_cast<Derived&>(*this);
    }

    Derived& operator*=(value_type scalar) {
        value *= scalar;
        return static_cast<Derived&>(*this);
    }

    Derived& operator/=(value_type scalar) {
        value = numerical::safeDivide(value, scalar);
        return static_cast<Derived&>(*this);
    }

    // Comparison operators
    constexpr bool operator==(const Derived& other) const {
        return numerical::approxEqual(value, other.value);
    }

    constexpr bool operator!=(const Derived& other) const {
        return !(*this == other);
    }

    constexpr bool operator<(const Derived& other) const { return value < other.value; }
    constexpr bool operator<=(const Derived& other) const { return value <= other.value; }
    constexpr bool operator>(const Derived& other) const { return value > other.value; }
    constexpr bool operator>=(const Derived& other) const { return value >= other.value; }

    // Utility methods
    constexpr Derived abs() const {
        return Derived(numerical::abs(value));
    }

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

    // Smooth step interpolation (S-curve)
    UNITS_CONSTEXPR14 Derived smoothstep(const Derived& target, value_type t) const {
        t = numerical::clamp(t, value_type(0), value_type(1));
        t = t * t * (3 - 2 * t);
        return lerp(target, t);
    }

    // State queries
    constexpr bool isZero(value_type tolerance = numerical::epsilon<value_type>() * 100) const {
        return numerical::isZero(value, tolerance);
    }

    constexpr bool isPositive() const { return value > 0; }
    constexpr bool isNegative() const { return value < 0; }
    constexpr bool isFinite() const { return std::isfinite(value); }

    // Raw value access
    constexpr value_type raw() const { return value; }
};

// Scalar multiplication from left side
template<typename Derived, typename Ratio>
inline constexpr Derived operator*(typename UnitBase<Derived, Ratio>::value_type scalar,
                                   const UnitBase<Derived, Ratio>& unit) {
    return static_cast<const Derived&>(unit) * scalar;
}

// ============================================================================
// DISTANCE UNIT
// ============================================================================
// Why this design:
// - Template ratio allows compile-time unit tracking
// - All conversions go through meters (SI base unit)
// - Factory methods provide type safety
// - Conversion methods are explicit (no implicit conversions)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Distance : public UnitBase<Distance<Ratio>, Ratio> {
    using Base = UnitBase<Distance<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversion to other units (always explicit)
    constexpr double toMeters() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toKilometers() const { return toMeters() * 0.001; }
    constexpr double toFeet() const { return toMeters() / constants::FEET_TO_METERS; }
    constexpr double toInches() const { return toMeters() / constants::INCHES_TO_METERS; }
    constexpr double toCentimeters() const { return toMeters() * 100.0; }
    constexpr double toMillimeters() const { return toMeters() * 1000.0; }
    constexpr double toMiles() const { return toMeters() / 1609.344; }

    // Factory methods (type-safe construction)
    static constexpr Distance fromMeters(double m) {
        return Distance(m * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Distance fromKilometers(double km) {
        return fromMeters(km * 1000.0);
    }

    static constexpr Distance fromFeet(double ft) {
        return fromMeters(ft * constants::FEET_TO_METERS);
    }

    static constexpr Distance fromInches(double in) {
        return fromMeters(in * constants::INCHES_TO_METERS);
    }

    static constexpr Distance fromCentimeters(double cm) {
        return fromMeters(cm * 0.01);
    }

    static constexpr Distance fromMillimeters(double mm) {
        return fromMeters(mm * 0.001);
    }

    static constexpr Distance fromMiles(double mi) {
        return fromMeters(mi * 1609.344);
    }
};

// Type aliases for common units
using Meters = Distance<std::ratio<1, 1>>;
using Kilometers = Distance<std::ratio<1000, 1>>;
using Feet = Distance<std::ratio<3048, 10000>>;  // Exact: 0.3048
using Inches = Distance<std::ratio<254, 10000>>;  // Exact: 0.0254
using Centimeters = Distance<std::ratio<1, 100>>;
using Millimeters = Distance<std::ratio<1, 1000>>;
using Miles = Distance<std::ratio<1609344, 1000>>;

// Register Distance as a unit type
namespace traits {
    template<typename R> struct is_unit<Distance<R>> : std::true_type {};
}

// ============================================================================
// TIME UNIT
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Time : public UnitBase<Time<Ratio>, Ratio> {
    using Base = UnitBase<Time<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions
    constexpr double toSeconds() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toMilliseconds() const { return toSeconds() * 1000.0; }
    constexpr double toMicroseconds() const { return toSeconds() * 1000000.0; }
    constexpr double toMinutes() const { return toSeconds() / 60.0; }
    constexpr double toHours() const { return toSeconds() / 3600.0; }

    // Factory methods
    static constexpr Time fromSeconds(double s) {
        return Time(s * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Time fromMilliseconds(double ms) {
        return fromSeconds(ms * 0.001);
    }

    static constexpr Time fromMicroseconds(double us) {
        return fromSeconds(us * 0.000001);
    }

    static constexpr Time fromMinutes(double min) {
        return fromSeconds(min * 60.0);
    }

    static constexpr Time fromHours(double hr) {
        return fromSeconds(hr * 3600.0);
    }
};

// Type aliases
using Seconds = Time<std::ratio<1, 1>>;
using Milliseconds = Time<std::ratio<1, 1000>>;
using Microseconds = Time<std::ratio<1, 1000000>>;
using Minutes = Time<std::ratio<60, 1>>;
using Hours = Time<std::ratio<3600, 1>>;

namespace traits {
    template<typename R> struct is_unit<Time<R>> : std::true_type {};
}

// ============================================================================
// ANGLE UNIT
// ============================================================================
// Why this design:
// - Internally stores as ratio of radians (mathematical standard)
// - Normalization methods for different use cases
// - Efficient sincos for rotation matrices
// - Static factory methods for inverse trig (bounds checking)
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Angle : public UnitBase<Angle<Ratio>, Ratio> {
    using Base = UnitBase<Angle<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions
    constexpr double toRadians() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toDegrees() const {
        return toRadians() * constants::RAD_TO_DEG;
    }

    constexpr double toRotations() const {
        return toRadians() / constants::TWO_PI;
    }

    // Factory methods
    static constexpr Angle fromRadians(double rad) {
        return Angle(rad * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Angle fromDegrees(double deg) {
        return fromRadians(deg * constants::DEG_TO_RAD);
    }

    static constexpr Angle fromRotations(double rot) {
        return fromRadians(rot * constants::TWO_PI);
    }

    // Trigonometric functions
    UNITS_CONSTEXPR14 double sin() const { return std::sin(toRadians()); }
    UNITS_CONSTEXPR14 double cos() const { return std::cos(toRadians()); }
    UNITS_CONSTEXPR14 double tan() const { return std::tan(toRadians()); }

    // Efficient combined sin/cos (useful for rotations)
    UNITS_CONSTEXPR14 void sincos(double& s, double& c) const {
        const double rad = toRadians();
        s = std::sin(rad);
        c = std::cos(rad);
    }

    // Inverse trig with bounds checking
    static UNITS_CONSTEXPR14 Angle asin(double value) {
        return fromRadians(std::asin(numerical::clamp(value, -1.0, 1.0)));
    }

    static UNITS_CONSTEXPR14 Angle acos(double value) {
        return fromRadians(std::acos(numerical::clamp(value, -1.0, 1.0)));
    }

    static UNITS_CONSTEXPR14 Angle atan(double value) {
        return fromRadians(std::atan(value));
    }

    static UNITS_CONSTEXPR14 Angle atan2(double y, double x) {
        return fromRadians(std::atan2(y, x));
    }

    // Normalize to [0, 2π)
    UNITS_CONSTEXPR14 Angle normalizePositive() const {
        double rad = std::fmod(toRadians(), constants::TWO_PI);
        if (rad < 0) rad += constants::TWO_PI;
        return fromRadians(rad);
    }

    // Normalize to [-π, π)
    UNITS_CONSTEXPR14 Angle normalizeSigned() const {
        double rad = std::fmod(toRadians() + constants::PI, constants::TWO_PI);
        if (rad < 0) rad += constants::TWO_PI;
        return fromRadians(rad - constants::PI);
    }

    // Shortest angular distance to another angle
    UNITS_CONSTEXPR14 Angle shortestDistanceTo(const Angle& other) const {
        return (other - *this).normalizeSigned();
    }
};

// Type aliases
using Radians = Angle<std::ratio<1, 1>>;
using Degrees = Angle<std::ratio<17453293, 1000000000>>;  // π/180 approximation
using Rotations = Angle<std::ratio<6283185307, 1000000000>>;  // 2π approximation

namespace traits {
    template<typename R> struct is_unit<Angle<R>> : std::true_type {};
}

// ============================================================================
// MASS UNIT
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Mass : public UnitBase<Mass<Ratio>, Ratio> {
    using Base = UnitBase<Mass<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions (SI base: kilogram)
    constexpr double toKilograms() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toGrams() const { return toKilograms() * 1000.0; }
    constexpr double toMilligrams() const { return toKilograms() * 1000000.0; }
    constexpr double toPounds() const { return toKilograms() * 2.20462262185; }
    constexpr double toOunces() const { return toKilograms() * 35.27396195; }
    constexpr double toTons() const { return toKilograms() * 0.001; }

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

    static constexpr Mass fromPounds(double lb) {
        return fromKilograms(lb * 0.45359237);
    }

    static constexpr Mass fromOunces(double oz) {
        return fromKilograms(oz * 0.028349523125);
    }

    static constexpr Mass fromTons(double t) {
        return fromKilograms(t * 1000.0);
    }
};

// Type aliases
using Kilograms = Mass<std::ratio<1, 1>>;
using Grams = Mass<std::ratio<1, 1000>>;
using Milligrams = Mass<std::ratio<1, 1000000>>;
using Pounds = Mass<std::ratio<45359237, 100000000>>;
using Ounces = Mass<std::ratio<28349523125, 1000000000000>>;
using MetricTons = Mass<std::ratio<1000, 1>>;

namespace traits {
    template<typename R> struct is_unit<Mass<R>> : std::true_type {};
}

// ============================================================================
// TEMPERATURE UNIT
// ============================================================================
// Why special handling:
// - Temperature has offset scales (0°C ≠ 0K)
// - Internal storage in Kelvin (absolute scale)
// - Special care needed for temperature differences vs absolute temperatures
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Temperature : public UnitBase<Temperature<Ratio>, Ratio> {
    using Base = UnitBase<Temperature<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // All stored as Kelvin internally
    constexpr double toKelvin() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toCelsius() const {
        return toKelvin() - 273.15;
    }

    constexpr double toFahrenheit() const {
        return toCelsius() * 9.0/5.0 + 32.0;
    }

    // Factory methods
    static constexpr Temperature fromKelvin(double k) {
        return Temperature(k * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Temperature fromCelsius(double c) {
        return fromKelvin(c + 273.15);
    }

    static constexpr Temperature fromFahrenheit(double f) {
        return fromCelsius((f - 32.0) * 5.0/9.0);
    }

    // Common temperature constants
    static constexpr Temperature absoluteZero() { return fromKelvin(0); }
    static constexpr Temperature waterFreezing() { return fromCelsius(0); }
    static constexpr Temperature waterBoiling() { return fromCelsius(100); }
    static constexpr Temperature roomTemperature() { return fromCelsius(20); }
};

using Kelvin = Temperature<std::ratio<1, 1>>;
using Celsius = Temperature<std::ratio<1, 1>>;  // Stored as Kelvin
using Fahrenheit = Temperature<std::ratio<1, 1>>;  // Stored as Kelvin

namespace traits {
    template<typename R> struct is_unit<Temperature<R>> : std::true_type {};
}

// ============================================================================
// SCALAR (Dimensionless)
// ============================================================================
// Why not inherit from UnitBase:
// - Scalar needs different behavior (no unit conversions)
// - Can implicitly convert to/from double
// - Represents pure numbers, ratios, percentages
// ============================================================================
class Scalar {
public:
    double value;

    constexpr Scalar() : value(0) {}
    constexpr explicit Scalar(double v) : value(v) {}

    // Allow implicit conversion to double for convenience
    constexpr operator double() const { return value; }

    // Factory methods for common representations
    static constexpr Scalar fromValue(double v) { return Scalar(v); }
    static constexpr Scalar fromPercent(double p) { return Scalar(p * 0.01); }
    static constexpr Scalar fromRatio(double r) { return Scalar(r); }
    static constexpr Scalar fromDecibels(double db) {
        return Scalar(std::pow(10.0, db / 20.0));
    }

    // Conversions
    constexpr double toValue() const { return value; }
    constexpr double toPercent() const { return value * 100.0; }
    constexpr double toRatio() const { return value; }
    UNITS_CONSTEXPR14 double toDecibels() const {
        return 20.0 * std::log10(numerical::abs(value));
    }

    // Arithmetic
    constexpr Scalar operator+(const Scalar& other) const { return Scalar(value + other.value); }
    constexpr Scalar operator-(const Scalar& other) const { return Scalar(value - other.value); }
    constexpr Scalar operator*(const Scalar& other) const { return Scalar(value * other.value); }
    constexpr Scalar operator/(const Scalar& other) const {
        return Scalar(numerical::safeDivide(value, other.value));
    }
    constexpr Scalar operator-() const { return Scalar(-value); }

    // Mathematical functions
    UNITS_CONSTEXPR14 Scalar abs() const { return Scalar(numerical::abs(value)); }
    UNITS_CONSTEXPR14 Scalar sqrt() const { return Scalar(std::sqrt(numerical::abs(value))); }
    UNITS_CONSTEXPR14 Scalar pow(double exp) const { return Scalar(std::pow(value, exp)); }
    UNITS_CONSTEXPR14 Scalar exp() const { return Scalar(std::exp(value)); }
    UNITS_CONSTEXPR14 Scalar log() const { return Scalar(std::log(numerical::abs(value))); }

    // Comparison
    constexpr bool operator==(const Scalar& other) const {
        return numerical::approxEqual(value, other.value);
    }
    constexpr bool operator!=(const Scalar& other) const { return !(*this == other); }
    constexpr bool operator<(const Scalar& other) const { return value < other.value; }
    constexpr bool operator<=(const Scalar& other) const { return value <= other.value; }
    constexpr bool operator>(const Scalar& other) const { return value > other.value; }
    constexpr bool operator>=(const Scalar& other) const { return value >= other.value; }
};

namespace traits {
    template<> struct is_unit<Scalar> : std::true_type {};
}

// ============================================================================
// HELPER FUNCTIONS FOR UNIT CREATION
// ============================================================================
// Why: Provides intuitive, terse syntax for creating units
// These are in the core because they're fundamental to usability
// ============================================================================
namespace literals {
    // Distance literals
    inline constexpr Meters m(double val) { return Meters::fromMeters(val); }
    inline constexpr Kilometers km(double val) { return Kilometers::fromKilometers(val); }
    inline constexpr Feet ft(double val) { return Feet::fromFeet(val); }
    inline constexpr Inches in(double val) { return Inches::fromInches(val); }
    inline constexpr Centimeters cm(double val) { return Centimeters::fromCentimeters(val); }
    inline constexpr Millimeters mm(double val) { return Millimeters::fromMillimeters(val); }
    inline constexpr Miles mi(double val) { return Miles::fromMiles(val); }

    // Time literals
    inline constexpr Seconds s(double val) { return Seconds::fromSeconds(val); }
    inline constexpr Milliseconds ms(double val) { return Milliseconds::fromMilliseconds(val); }
    inline constexpr Microseconds us(double val) { return Microseconds::fromMicroseconds(val); }
    inline constexpr Minutes min(double val) { return Minutes::fromMinutes(val); }
    inline constexpr Hours hr(double val) { return Hours::fromHours(val); }

    // Angle literals
    inline constexpr Radians rad(double val) { return Radians::fromRadians(val); }
    inline constexpr Degrees deg(double val) { return Degrees::fromDegrees(val); }
    inline constexpr Rotations rot(double val) { return Rotations::fromRotations(val); }

    // Mass literals
    inline constexpr Kilograms kg(double val) { return Kilograms::fromKilograms(val); }
    inline constexpr Grams g(double val) { return Grams::fromGrams(val); }
    inline constexpr Pounds lb(double val) { return Pounds::fromPounds(val); }
    inline constexpr Ounces oz(double val) { return Ounces::fromOunces(val); }

    // Temperature literals
    inline constexpr Kelvin K(double val) { return Kelvin::fromKelvin(val); }
    inline constexpr Celsius degC(double val) { return Celsius::fromCelsius(val); }
    inline constexpr Fahrenheit degF(double val) { return Fahrenheit::fromFahrenheit(val); }

    // Scalar literals
    inline constexpr Scalar scalar(double val) { return Scalar::fromValue(val); }
    inline constexpr Scalar percent(double val) { return Scalar::fromPercent(val); }
    inline constexpr Scalar ratio(double val) { return Scalar::fromRatio(val); }
}

// Make literals available in units namespace
using namespace literals;

} // namespace units

#endif // ROBOTICS_UNITS_CORE_H
