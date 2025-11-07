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
// These constants are used throughout the library for conversions and
// calculations. High precision values prevent accumulation of rounding errors.
// ============================================================================
namespace constants {
    // ========================================================================
    // PI (π) ≈ 3.14159...
    // ========================================================================
    // WHAT: The ratio of a circle's circumference to its diameter
    // WHY: Used for circular motion, angle conversions, and trigonometry
    // FORMULA: C = 2πr (circumference), A = πr² (area)
    // EXAMPLE: A wheel with radius 0.5m has circumference = 2π(0.5) = 3.14m
    constexpr double PI = 3.14159265358979323846264338327950288;

    // 2π = Full circle in radians (360 degrees)
    // WHY: One complete rotation = 2π radians
    // EXAMPLE: Motor spinning at 60 RPM = 60 × 2π rad/min = 377 rad/min
    constexpr double TWO_PI = 2.0 * PI;

    // π/2 = Quarter circle (90 degrees)
    // WHY: Common angle for perpendicular directions
    // EXAMPLE: Robot turning 90° right rotates π/2 radians
    constexpr double HALF_PI = 0.5 * PI;

    // ========================================================================
    // E (Euler's number) ≈ 2.71828...
    // ========================================================================
    // WHAT: Base of natural logarithms
    // WHY: Used in exponential growth/decay, filters, and control systems
    // FORMULA: y = e^(kt) for exponential growth
    // EXAMPLE: Battery discharge, sensor filters, PID controllers
    constexpr double E = 2.71828182845904523536028747135266250;

    // ========================================================================
    // √2 ≈ 1.41421...
    // ========================================================================
    // WHAT: Square root of 2
    // WHY: Diagonal distance, signal normalization
    // EXAMPLE: Moving diagonally 1m in X and 1m in Y = √2 m total distance
    constexpr double SQRT2 = 1.41421356237309504880168872420969808;

    // ========================================================================
    // √3 ≈ 1.73205...
    // ========================================================================
    // WHAT: Square root of 3
    // WHY: Used in 120° symmetry (3-phase systems, triangle calculations)
    // EXAMPLE: Omnidirectional robots with 3 wheels at 120° apart
    constexpr double SQRT3 = 1.73205080756887729352744634150587237;

    // ========================================================================
    // ANGLE CONVERSION FACTORS
    // ========================================================================

    // DEGREES → RADIANS
    // MATH: radians = degrees × (π / 180)
    // WHY: A full circle is 360° or 2π radians
    //      Therefore: 1° = 2π/360 = π/180 radians
    // EXAMPLE: 90° × (π/180) = π/2 ≈ 1.5708 radians
    constexpr double DEG_TO_RAD = PI / 180.0;

    // RADIANS → DEGREES
    // MATH: degrees = radians × (180 / π)
    // WHY: Inverse of above conversion
    // EXAMPLE: π radians × (180/π) = 180°
    constexpr double RAD_TO_DEG = 180.0 / PI;

    // ========================================================================
    // DISTANCE CONVERSION FACTORS (Exact by international definition)
    // ========================================================================

    // FEET → METERS
    // EXACT DEFINITION: 1 foot = exactly 0.3048 meters (since 1959)
    // HISTORY: Defined to match imperial and metric systems
    // EXAMPLE: 10 feet × 0.3048 = 3.048 meters
    constexpr double FEET_TO_METERS = 0.3048;

    // INCHES → METERS
    // EXACT DEFINITION: 1 inch = exactly 2.54 cm = 0.0254 meters (since 1959)
    // DERIVATION: 1 foot = 12 inches, so 1 inch = 0.3048/12 = 0.0254 m
    // EXAMPLE: 6 inches × 0.0254 = 0.1524 meters
    constexpr double INCHES_TO_METERS = 0.0254;

    // ========================================================================
    // PHYSICAL CONSTANTS
    // ========================================================================

    // STANDARD GRAVITY (g)
    // EXACT DEFINITION: 9.80665 m/s² (defined standard, not measured)
    // WHY: Used for weight calculations, accelerometer calibration
    // FORMULA: Weight = mass × g (Newtons = kg × m/s²)
    // EXAMPLE: 10 kg mass weighs 10 × 9.80665 = 98.0665 N on Earth
    // NOTE: Actual gravity varies by location (9.78 to 9.83 m/s²)
    constexpr double GRAVITY_EARTH = 9.80665;

    // SPEED OF LIGHT (c)
    // EXACT DEFINITION: 299,792,458 m/s (defines the meter since 1983)
    // WHY: Used in relativity, time-of-flight sensors, GPS calculations
    // EXAMPLE: Light travels ~30 cm in 1 nanosecond
    constexpr double SPEED_OF_LIGHT = 299792458.0;
}

// ============================================================================
// NUMERICAL UTILITIES
// ============================================================================
// These utilities prevent common numerical errors in floating-point math.
// All are inline/constexpr for zero runtime overhead.
//
// WHY NEEDED: Floating-point numbers have precision limits that cause issues:
// - 0.1 + 0.2 ≠ 0.3 exactly in binary floating point
// - Division by zero is undefined (crashes or returns NaN/Inf)
// - Direct comparison with == is unreliable for computed values
// ============================================================================
namespace numerical {

    // ========================================================================
    // MACHINE EPSILON - Smallest distinguishable difference
    // ========================================================================
    // WHAT: The smallest number ε where 1.0 + ε ≠ 1.0 in floating point
    // WHY: Used to determine if numbers are "close enough" to be equal
    // VALUE: For double, ε ≈ 2.22 × 10⁻¹⁶
    // EXAMPLE: Can't distinguish 1.0 from 1.0 + 1×10⁻¹⁷
    template<typename T>
    inline constexpr T epsilon() {
        return std::numeric_limits<T>::epsilon();
    }

    // ========================================================================
    // APPROXIMATE EQUALITY - Safe floating-point comparison
    // ========================================================================
    // PROBLEM: 0.1 + 0.2 == 0.3 returns FALSE in floating point!
    //          Actual value: 0.30000000000000004...
    //
    // SOLUTION: Check if |a - b| < tolerance
    //
    // ALGORITHM:
    //   if (a - b < tolerance AND b - a < tolerance)
    //       then a ≈ b (approximately equal)
    //
    // WHY TWO CHECKS: Avoids calling std::abs() for constexpr compatibility
    //   (a - b) < tolerance  checks if a is not much bigger than b
    //   (b - a) < tolerance  checks if b is not much bigger than a
    //   Both true means |a - b| < tolerance
    //
    // TOLERANCE: Default is 100 × machine epsilon (very strict)
    //
    // EXAMPLE:
    //   double a = 0.1 + 0.2;              // 0.30000000000000004
    //   double b = 0.3;                    // 0.29999999999999999
    //   a == b → FALSE (wrong!)
    //   approxEqual(a, b) → TRUE (correct!)
    template<typename T>
    inline constexpr bool approxEqual(T a, T b, T tolerance = epsilon<T>() * 100) {
        return (a - b) < tolerance && (b - a) < tolerance;
    }

    // ========================================================================
    // IS ZERO - Check if number is effectively zero
    // ========================================================================
    // PROBLEM: After many calculations, 0.0 might become 1×10⁻¹⁵
    //
    // ALGORITHM:
    //   if (value < tolerance AND -value < tolerance)
    //       then value ≈ 0
    //
    // WHY: Same as approxEqual(value, 0) but faster
    //
    // EXAMPLE:
    //   double x = 1.0 - 1.0;              // Might be 1×10⁻¹⁶, not exactly 0
    //   x == 0.0 → Might be FALSE
    //   isZero(x) → TRUE
    template<typename T>
    inline constexpr bool isZero(T value, T tolerance = epsilon<T>() * 100) {
        return value < tolerance && -value < tolerance;
    }

    // ========================================================================
    // SAFE DIVISION - Prevent divide-by-zero crashes
    // ========================================================================
    // PROBLEM: Division by zero is undefined
    //   - Can crash program
    //   - Returns NaN (Not a Number) or Inf (Infinity)
    //   - Propagates errors through calculations
    //
    // SOLUTION: Check denominator before dividing
    //
    // ALGORITHM:
    //   if (denominator ≈ 0)
    //       return defaultValue (usually 0)
    //   else
    //       return numerator / denominator
    //
    // EXAMPLE:
    //   safeDivide(10.0, 0.0, 0.0) → 0.0 (safe)
    //   10.0 / 0.0 → Inf or crash (unsafe)
    //
    // USE CASE: Robot velocity = distance / time
    //   If time = 0, safeDivide returns 0 instead of crashing
    template<typename T>
    inline constexpr T safeDivide(T numerator, T denominator, T defaultValue = T(0)) {
        return isZero(denominator) ? defaultValue : numerator / denominator;
    }

    // ========================================================================
    // MIN - Return smaller of two values
    // ========================================================================
    // ALGORITHM: if (a < b) return a; else return b;
    // EXAMPLE: min(5, 3) → 3
    template<typename T>
    inline constexpr T min(T a, T b) { return a < b ? a : b; }

    // ========================================================================
    // MAX - Return larger of two values
    // ========================================================================
    // ALGORITHM: if (a > b) return a; else return b;
    // EXAMPLE: max(5, 3) → 5
    template<typename T>
    inline constexpr T max(T a, T b) { return a > b ? a : b; }

    // ========================================================================
    // CLAMP - Constrain value to range [low, high]
    // ========================================================================
    // ALGORITHM:
    //   if (value < low) return low;
    //   else if (value > high) return high;
    //   else return value;
    //
    // VISUAL:
    //        low        high
    //    ----[==========]----
    //    ^              ^   ^
    //    |              |   |
    //   too low        OK  too high
    //
    // EXAMPLE: clamp(15, 0, 10) → 10 (limited to max)
    //          clamp(-5, 0, 10) → 0  (limited to min)
    //          clamp(7, 0, 10) → 7   (within range)
    //
    // USE CASE: Motor PWM must be in range [0, 255]
    //   pwm = clamp(calculated_pwm, 0, 255)
    template<typename T>
    inline constexpr T clamp(T value, T low, T high) {
        return value < low ? low : (value > high ? high : value);
    }

    // ========================================================================
    // ABSOLUTE VALUE - Remove sign, get magnitude
    // ========================================================================
    // ALGORITHM: if (value < 0) return -value; else return value;
    //
    // MATH: |x| = { x if x ≥ 0
    //             {-x if x < 0
    //
    // EXAMPLE: abs(-5) → 5
    //          abs(3) → 3
    //          abs(0) → 0
    //
    // USE CASE: Distance is always positive: distance = abs(final - initial)
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
// BASE UNIT CLASS (CRTP - Curiously Recurring Template Pattern)
// ============================================================================
// WHAT IS CRTP?
//   A design pattern where a class X inherits from a template Base<X>
//   Example: class Distance : public UnitBase<Distance>
//
// WHY USE CRTP INSTEAD OF NORMAL INHERITANCE?
//
// 1. ZERO RUNTIME OVERHEAD
//    Normal inheritance: Uses virtual functions (vtable pointer, runtime lookup)
//    CRTP: All resolved at compile time (no vtable, no runtime cost)
//    Critical for embedded systems with limited memory!
//
// 2. TYPE SAFETY
//    Prevents mixing incompatible units:
//    ✓ Distance + Distance = Distance  (allowed)
//    ✗ Distance + Time = ???           (compile error!)
//
// 3. COMPILE-TIME POLYMORPHISM
//    Operations return the correct derived type automatically
//    Distance + Distance returns Distance, not UnitBase
//
// 4. SHARED IMPLEMENTATION
//    Common operations (addition, comparison, etc.) defined once
//    All unit types get these operations for free
//
// EXAMPLE:
//   Meters m1(5.0);
//   Meters m2(3.0);
//   Meters m3 = m1 + m2;  // Returns Meters, not UnitBase!
//
// RATIO PARAMETER:
//   Stores unit scale at compile time using std::ratio
//   Examples:
//   - std::ratio<1,1>     = 1/1 = 1.0      (meters)
//   - std::ratio<1,1000>  = 1/1000 = 0.001 (millimeters)
//   - std::ratio<1000,1>  = 1000/1 = 1000  (kilometers)
// ============================================================================
template<typename Derived, typename Ratio = std::ratio<1, 1>>
class UnitBase {
public:
    using ratio = Ratio;
    using value_type = double;

    value_type value;

    // ========================================================================
    // CONSTRUCTORS
    // ========================================================================
    constexpr UnitBase() : value(0) {}
    constexpr explicit UnitBase(value_type v) : value(v) {}

    // ========================================================================
    // ARITHMETIC OPERATIONS - Dimensional Analysis
    // ========================================================================
    // These operations follow the laws of dimensional analysis (physics units)
    //
    // KEY PRINCIPLE: Units must match for addition/subtraction
    //   ✓ 5 meters + 3 meters = 8 meters
    //   ✗ 5 meters + 3 seconds = ??? (compile error!)
    //
    // NOTE: All operations return Derived type (CRTP magic)
    //   This ensures Distance + Distance returns Distance, not UnitBase
    // ========================================================================

    // ADDITION: Same units can be added
    // MATH: [Distance] + [Distance] = [Distance]
    // EXAMPLE: 5m + 3m = 8m
    // ALGORITHM: Simply add the numeric values (units already match)
    constexpr Derived operator+(const Derived& other) const {
        return Derived(value + other.value);
    }

    // SUBTRACTION: Same units can be subtracted
    // MATH: [Distance] - [Distance] = [Distance]
    // EXAMPLE: 10m - 4m = 6m
    // ALGORITHM: Simply subtract the numeric values
    constexpr Derived operator-(const Derived& other) const {
        return Derived(value - other.value);
    }

    // SCALAR MULTIPLICATION: Unit × number = same unit scaled
    // MATH: [Distance] × scalar = [Distance]
    // EXAMPLE: 5m × 2 = 10m
    // WHY: Multiplying by a pure number doesn't change the dimension
    // ALGORITHM: Multiply the numeric value
    constexpr Derived operator*(value_type scalar) const {
        return Derived(value * scalar);
    }

    // SCALAR DIVISION: Unit ÷ number = same unit scaled
    // MATH: [Distance] ÷ scalar = [Distance]
    // EXAMPLE: 10m ÷ 2 = 5m
    // WHY: Dividing by a pure number doesn't change the dimension
    // ALGORITHM: Safely divide the numeric value (check for zero)
    constexpr Derived operator/(value_type scalar) const {
        return Derived(numerical::safeDivide(value, scalar));
    }

    // UNIT DIVISION: Unit ÷ same unit = pure number (ratio)
    // MATH: [Distance] ÷ [Distance] = dimensionless
    // EXAMPLE: 10m ÷ 2m = 5 (pure number, no units)
    // WHY: Units cancel out (m/m = 1)
    // RETURNS: double (not a unit type)
    //
    // USE CASE: Calculate scale factor
    //   double scale = actual_distance / expected_distance;
    constexpr value_type operator/(const Derived& other) const {
        return numerical::safeDivide(value, other.value);
    }

    // NEGATION: Reverse the sign
    // MATH: -[Distance] = [Distance]
    // EXAMPLE: -(5m) = -5m
    // USE CASE: Reverse direction
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

    // ========================================================================
    // LINEAR INTERPOLATION (LERP)
    // ========================================================================
    // WHAT: Find a point between two values
    //
    // MATH: lerp(a, b, t) = a + (b - a) × t
    //   where t ∈ [0, 1]
    //
    // ALGORITHM BREAKDOWN:
    //   1. Calculate difference: (b - a)
    //   2. Scale by t: (b - a) × t
    //   3. Add to start: a + (b - a) × t
    //
    // PARAMETER t:
    //   t = 0.0 → returns start value (a)
    //   t = 0.5 → returns midpoint (a + b)/2
    //   t = 1.0 → returns target value (b)
    //
    // VISUAL:
    //   start=10        target=20
    //      |--------------|
    //   t=0  t=0.25  t=0.5  t=0.75  t=1
    //   10     12.5    15     17.5   20
    //
    // EXAMPLES:
    //   lerp(10m, 20m, 0.0) = 10m (start)
    //   lerp(10m, 20m, 0.5) = 15m (halfway)
    //   lerp(10m, 20m, 1.0) = 20m (end)
    //   lerp(10m, 20m, 0.25) = 12.5m (quarter way)
    //
    // USE CASES:
    //   - Smooth robot motion from point A to B
    //   - Animation and transitions
    //   - Path planning
    //   - Gradual acceleration
    constexpr Derived lerp(const Derived& target, value_type t) const {
        return Derived(value + (target.value - value) * t);
    }

    // ========================================================================
    // SMOOTH STEP INTERPOLATION (S-CURVE)
    // ========================================================================
    // WHAT: Smooth acceleration/deceleration between two values
    //
    // WHY BETTER THAN LERP:
    //   - LERP: Constant velocity (abrupt start/stop)
    //   - SMOOTHSTEP: Gradual start, constant middle, gradual stop
    //
    // MATH: smoothstep(t) = 3t² - 2t³
    //   This creates an S-shaped curve
    //
    // ALGORITHM:
    //   1. Clamp t to [0, 1]
    //   2. Apply smoothing: t' = t² × (3 - 2t)
    //   3. Interpolate with smoothed t': lerp(start, target, t')
    //
    // VISUAL COMPARISON:
    //   LERP:       /        (straight line, constant speed)
    //              /
    //
    //   SMOOTHSTEP: ╱        (S-curve, smooth start/stop)
    //              ╱
    //
    // VELOCITY PROFILE:
    //   Time:     0 -------- 0.5 -------- 1.0
    //   LERP:     |========|========|    (constant)
    //   SMOOTHSTEP: ╱╲_____╱╲    (smooth)
    //
    // EXAMPLE:
    //   smoothstep(0m, 10m, 0.0) = 0m   (slow start)
    //   smoothstep(0m, 10m, 0.5) = 5m   (full speed)
    //   smoothstep(0m, 10m, 1.0) = 10m  (slow stop)
    //
    // USE CASES:
    //   - Smooth robot movements
    //   - Comfortable elevator motion
    //   - Natural-feeling animations
    UNITS_CONSTEXPR14 Derived smoothstep(const Derived& target, value_type t) const {
        t = numerical::clamp(t, value_type(0), value_type(1));
        t = t * t * (3 - 2 * t);  // S-curve smoothing formula
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
// DESIGN PRINCIPLES:
//
// 1. COMPILE-TIME UNIT TRACKING
//    Template ratio stores the unit scale (e.g., 1/1000 for mm)
//    Compiler knows unit type at compile time (no runtime overhead!)
//
// 2. METERS AS BASE UNIT (SI Standard)
//    All conversions go through meters internally
//    Why: International standard, simplifies conversion logic
//
// 3. FACTORY METHODS FOR TYPE SAFETY
//    Can't accidentally create Distance with wrong value
//    Must use Distance::fromMeters(5.0), not Distance(5.0)
//
// 4. EXPLICIT CONVERSIONS ONLY
//    Prevents silent bugs from implicit conversions
//    Must explicitly call toMeters(), can't auto-convert
//
// INTERNAL STORAGE:
//   Value is stored in units matching the Ratio parameter
//   - Distance<ratio<1,1>>(5.0) stores 5.0 meters
//   - Distance<ratio<1,1000>>(5.0) stores 5.0 millimeters
//
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Distance : public UnitBase<Distance<Ratio>, Ratio> {
    using Base = UnitBase<Distance<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // ========================================================================
    // CONVERSION TO OTHER UNITS (always explicit, prevents silent bugs)
    // ========================================================================

    // CONVERT TO METERS (SI base unit)
    //
    // MATH: meters = stored_value × (numerator / denominator)
    //
    // EXAMPLE 1: Millimeters to meters
    //   Ratio = std::ratio<1, 1000> = 1/1000 = 0.001
    //   Value = 5000 (5000 millimeters)
    //   Result = 5000 × (1/1000) = 5.0 meters ✓
    //
    // EXAMPLE 2: Kilometers to meters
    //   Ratio = std::ratio<1000, 1> = 1000/1 = 1000
    //   Value = 2 (2 kilometers)
    //   Result = 2 × (1000/1) = 2000 meters ✓
    //
    // WHY THIS FORMULA:
    //   Ratio represents "this unit / meter"
    //   To get meters, multiply by the ratio
    constexpr double toMeters() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    // CONVERT TO KILOMETERS
    // MATH: kilometers = meters × 0.001
    // WHY: 1 km = 1000 m, so m → km requires dividing by 1000
    // EXAMPLE: 5000 meters = 5000 × 0.001 = 5 kilometers
    constexpr double toKilometers() const { return toMeters() * 0.001; }

    // CONVERT TO FEET
    // MATH: feet = meters / 0.3048
    // WHY: 1 foot = 0.3048 meters (exact definition)
    //      So to convert m → ft, divide by 0.3048
    // DERIVATION: feet = meters × (1 ft / 0.3048 m) = meters / 0.3048
    // EXAMPLE: 3.048 meters = 3.048 / 0.3048 = 10 feet
    constexpr double toFeet() const { return toMeters() / constants::FEET_TO_METERS; }

    // CONVERT TO INCHES
    // MATH: inches = meters / 0.0254
    // WHY: 1 inch = 0.0254 meters = 2.54 cm (exact definition)
    // DERIVATION: inches = meters × (1 in / 0.0254 m) = meters / 0.0254
    // EXAMPLE: 0.254 meters = 0.254 / 0.0254 = 10 inches
    constexpr double toInches() const { return toMeters() / constants::INCHES_TO_METERS; }

    // CONVERT TO CENTIMETERS
    // MATH: centimeters = meters × 100
    // WHY: 1 meter = 100 centimeters (centi = 1/100)
    // EXAMPLE: 1.5 meters = 1.5 × 100 = 150 centimeters
    constexpr double toCentimeters() const { return toMeters() * 100.0; }

    // CONVERT TO MILLIMETERS
    // MATH: millimeters = meters × 1000
    // WHY: 1 meter = 1000 millimeters (milli = 1/1000)
    // EXAMPLE: 0.5 meters = 0.5 × 1000 = 500 millimeters
    constexpr double toMillimeters() const { return toMeters() * 1000.0; }

    // CONVERT TO MILES
    // MATH: miles = meters / 1609.344
    // WHY: 1 mile = 1609.344 meters (exact definition)
    // DERIVATION: 1 mile = 5280 feet × 0.3048 m/ft = 1609.344 m
    // EXAMPLE: 1609.344 meters = 1609.344 / 1609.344 = 1 mile
    constexpr double toMiles() const { return toMeters() / 1609.344; }

    // ========================================================================
    // FACTORY METHODS (type-safe construction)
    // ========================================================================
    // WHY FACTORY METHODS:
    //   Prevent confusion about what value represents
    //   Distance::fromMeters(5.0) is clear
    //   Distance(5.0) is ambiguous - meters? feet? millimeters?
    //
    // ALL CONVERSIONS GO THROUGH METERS (reduces code duplication)
    // ========================================================================

    // CREATE FROM METERS
    //
    // MATH: stored_value = meters × (denominator / numerator)
    //   This is the inverse of toMeters()
    //
    // EXAMPLE 1: Create 5 meters as Millimeters type
    //   Ratio = std::ratio<1, 1000>
    //   Input = 5.0 meters
    //   stored_value = 5.0 × (1000/1) = 5000
    //   Stores 5000 (which represents 5000 millimeters) ✓
    //
    // EXAMPLE 2: Create 2000 meters as Kilometers type
    //   Ratio = std::ratio<1000, 1>
    //   Input = 2000 meters
    //   stored_value = 2000 × (1/1000) = 2
    //   Stores 2 (which represents 2 kilometers) ✓
    static constexpr Distance fromMeters(double m) {
        return Distance(m * Ratio::den / static_cast<double>(Ratio::num));
    }

    // CREATE FROM KILOMETERS
    // STEP 1: Convert km to meters: km × 1000
    // STEP 2: Use fromMeters to create Distance
    // EXAMPLE: 2.5 km = 2.5 × 1000 = 2500 meters
    static constexpr Distance fromKilometers(double km) {
        return fromMeters(km * 1000.0);
    }

    // CREATE FROM FEET
    // STEP 1: Convert feet to meters: feet × 0.3048
    // STEP 2: Use fromMeters to create Distance
    // EXAMPLE: 10 feet = 10 × 0.3048 = 3.048 meters
    static constexpr Distance fromFeet(double ft) {
        return fromMeters(ft * constants::FEET_TO_METERS);
    }

    // CREATE FROM INCHES
    // STEP 1: Convert inches to meters: inches × 0.0254
    // STEP 2: Use fromMeters to create Distance
    // EXAMPLE: 10 inches = 10 × 0.0254 = 0.254 meters
    static constexpr Distance fromInches(double in) {
        return fromMeters(in * constants::INCHES_TO_METERS);
    }

    // CREATE FROM CENTIMETERS
    // STEP 1: Convert cm to meters: cm ÷ 100
    // STEP 2: Use fromMeters to create Distance
    // EXAMPLE: 150 cm = 150 × 0.01 = 1.5 meters
    static constexpr Distance fromCentimeters(double cm) {
        return fromMeters(cm * 0.01);
    }

    // CREATE FROM MILLIMETERS
    // STEP 1: Convert mm to meters: mm ÷ 1000
    // STEP 2: Use fromMeters to create Distance
    // EXAMPLE: 500 mm = 500 × 0.001 = 0.5 meters
    static constexpr Distance fromMillimeters(double mm) {
        return fromMeters(mm * 0.001);
    }

    // CREATE FROM MILES
    // STEP 1: Convert miles to meters: miles × 1609.344
    // STEP 2: Use fromMeters to create Distance
    // EXAMPLE: 1 mile = 1 × 1609.344 = 1609.344 meters
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

    // ========================================================================
    // NORMALIZE TO [0, 2π) - Positive angles only
    // ========================================================================
    // PROBLEM: Angles can be > 360° or negative
    //   Example: Robot turned 370° or -30°
    //   We want equivalent angle in standard range [0°, 360°)
    //
    // MATH: Use modulo operation
    //   angle_normalized = angle mod 2π
    //
    // ALGORITHM:
    //   1. Calculate remainder: angle mod 2π (fmod function)
    //   2. If negative, add 2π to make positive
    //   3. Result is in range [0, 2π)
    //
    // EXAMPLES:
    //   370° → 370 mod 360 = 10°  (one full rotation + 10°)
    //   -30° → -30 mod 360 = 330° (same as turning right 30°)
    //   720° → 720 mod 360 = 0°   (two full rotations = 0°)
    //   90°  → 90 mod 360 = 90°   (already in range, unchanged)
    //
    // VISUAL (0° is right, counterclockwise):
    //          90° (up)
    //           |
    //  180° ----+---- 0° (right)
    //           |
    //         270° (down)
    //
    //   450° = 360° + 90° → normalizes to 90°
    //   -90° = 360° - 90° → normalizes to 270°
    //
    // USE CASES:
    //   - Display heading in standard format
    //   - Compass bearings (0° = North, 90° = East)
    //   - Robot orientation tracking
    UNITS_CONSTEXPR14 Angle normalizePositive() const {
        double rad = std::fmod(toRadians(), constants::TWO_PI);
        if (rad < 0) rad += constants::TWO_PI;
        return fromRadians(rad);
    }

    // ========================================================================
    // NORMALIZE TO [-π, π) - Signed angles
    // ========================================================================
    // PROBLEM: Sometimes we want angles in range [-180°, +180°)
    //   More intuitive for:
    //   - Turn left (negative) vs right (positive)
    //   - Error calculations in control systems
    //   - Shortest rotation direction
    //
    // MATH: Shift, modulo, shift back
    //   1. Add π (shift range)
    //   2. Mod 2π
    //   3. Subtract π (shift back)
    //
    // ALGORITHM:
    //   1. Shift: (angle + π)
    //   2. Normalize: mod 2π
    //   3. Handle negative: if < 0, add 2π
    //   4. Shift back: - π
    //   Result is in range [-π, π)
    //
    // EXAMPLES:
    //   370° → 10°    (small positive turn)
    //   -30° → -30°   (small negative turn)
    //   270° → -90°   (right turn, not 270° left!)
    //   180° → -180° or 180° (on boundary)
    //
    // VISUAL:
    //     +90° (left/up)
    //         |
    //  +/-180°+---- 0° (straight)
    //         |
    //     -90° (right/down)
    //
    // WHY THIS IS BETTER FOR CONTROL:
    //   Positive range:  190° means "turn 190° left" (very far)
    //   Signed range:    190° → -170° means "turn 170° right" (closer!)
    //
    // USE CASES:
    //   - PID control (minimize turn distance)
    //   - Robot path planning (shortest rotation)
    //   - Joystick input (-180° to +180°)
    UNITS_CONSTEXPR14 Angle normalizeSigned() const {
        double rad = std::fmod(toRadians() + constants::PI, constants::TWO_PI);
        if (rad < 0) rad += constants::TWO_PI;
        return fromRadians(rad - constants::PI);
    }

    // ========================================================================
    // SHORTEST ANGULAR DISTANCE - Which way to turn?
    // ========================================================================
    // PROBLEM: To rotate from angle A to angle B, which direction is shorter?
    //
    // EXAMPLE PROBLEM:
    //   Current: 10°
    //   Target: 350°
    //   Long way: 10° → 350° = +340° (turn left almost full circle)
    //   Short way: 10° → 350° = -20° (turn right just a bit!)
    //
    // SOLUTION: Use signed normalization
    //
    // ALGORITHM:
    //   1. Calculate difference: target - current
    //   2. Normalize to [-180°, +180°]
    //   3. Sign indicates direction:
    //      Positive = turn left (counterclockwise)
    //      Negative = turn right (clockwise)
    //   4. Magnitude = degrees to turn
    //
    // EXAMPLES:
    //   Current=10°, Target=350°
    //     Difference = 350° - 10° = 340°
    //     Normalized = -20° (turn right 20°) ✓
    //
    //   Current=350°, Target=10°
    //     Difference = 10° - 350° = -340°
    //     Normalized = +20° (turn left 20°) ✓
    //
    //   Current=0°, Target=90°
    //     Difference = 90° - 0° = 90°
    //     Normalized = +90° (turn left 90°) ✓
    //
    // VISUAL EXAMPLE:
    //       current(10°)
    //           |
    //      0° --+------------
    //           |     \
    //           |      \  SHORT PATH (-20°)
    //           |       \
    //    270° --+-------- target(350°)
    //     ^                ^
    //     |                |
    //   LONG PATH: 10°→90°→180°→270°→350° = 340° (don't do this!)
    //   SHORT PATH: 10°→0°→350° = -20° (do this!)
    //
    // USE CASES:
    //   - Robot turning to target
    //   - Servo moving to position
    //   - Turret aiming
    //   - Gimbal stabilization
    //
    // RETURNS: Signed angle
    //   Positive: turn counterclockwise (left)
    //   Negative: turn clockwise (right)
    //   Magnitude: degrees to turn
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
