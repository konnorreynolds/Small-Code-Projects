// ============================================================================
// units_physics.h - Physics and Derived Units
// ============================================================================
// Purpose: Derived units for physics calculations and engineering
// Dependencies: units_core.h
//
// This file contains:
// - Velocity, Acceleration (linear and angular)
// - Force, Energy, Power, Torque
// - Electrical units (Voltage, Current, Resistance)
// - Frequency and related units
// - Unit operation overloads (Force = Mass × Acceleration, etc.)
// ============================================================================

#ifndef ROBOTICS_UNITS_PHYSICS_H
#define ROBOTICS_UNITS_PHYSICS_H

#include "units_core.h"

namespace units {

// ============================================================================
// VELOCITY (Derived: Distance / Time)
// ============================================================================
// Why template approach:
// - Preserves unit information at compile time
// - Allows different combinations (m/s, ft/min, etc.)
// - Zero runtime overhead for conversions
// ============================================================================
template<typename DistanceRatio = std::ratio<1,1>, typename TimeRatio = std::ratio<1,1>>
class Velocity {
public:
    using distance_ratio = DistanceRatio;
    using time_ratio = TimeRatio;
    double value;

    constexpr Velocity() : value(0) {}
    constexpr explicit Velocity(double v) : value(v) {}

    // Convert to standard SI units (m/s)
    // C++11 constexpr: must be single return statement
    constexpr double toMetersPerSecond() const {
        return value * (distance_ratio::num / static_cast<double>(distance_ratio::den))
                     * (time_ratio::den / static_cast<double>(time_ratio::num));
    }

    // Other common conversions
    constexpr double toKilometersPerHour() const { return toMetersPerSecond() * 3.6; }
    constexpr double toMilesPerHour() const { return toMetersPerSecond() * 2.23693629205; }
    constexpr double toFeetPerSecond() const { return toMetersPerSecond() / constants::FEET_TO_METERS; }
    constexpr double toKnots() const { return toMetersPerSecond() * 1.94384449244; }

    // Factory methods
    // C++11 constexpr: must be single return statement
    static constexpr Velocity fromMetersPerSecond(double mps) {
        return Velocity(mps * (distance_ratio::den / static_cast<double>(distance_ratio::num))
                            * (time_ratio::num / static_cast<double>(time_ratio::den)));
    }

    static constexpr Velocity fromKilometersPerHour(double kmh) {
        return fromMetersPerSecond(kmh / 3.6);
    }

    static constexpr Velocity fromMilesPerHour(double mph) {
        return fromMetersPerSecond(mph / 2.23693629205);
    }

    // Arithmetic operations
    constexpr Velocity operator+(const Velocity& other) const { return Velocity(value + other.value); }
    constexpr Velocity operator-(const Velocity& other) const { return Velocity(value - other.value); }
    constexpr Velocity operator*(double scalar) const { return Velocity(value * scalar); }
    constexpr Velocity operator/(double scalar) const {
        return Velocity(numerical::safeDivide(value, scalar));
    }
    constexpr Velocity operator-() const { return Velocity(-value); }

    // Comparisons
    constexpr bool operator==(const Velocity& other) const {
        return numerical::approxEqual(value, other.value);
    }
    constexpr bool operator!=(const Velocity& other) const { return !(*this == other); }
    constexpr bool operator<(const Velocity& other) const { return value < other.value; }
    constexpr bool operator<=(const Velocity& other) const { return value <= other.value; }
    constexpr bool operator>(const Velocity& other) const { return value > other.value; }
    constexpr bool operator>=(const Velocity& other) const { return value >= other.value; }

    // Physics calculations
    template<typename MassRatio>
    constexpr double kineticEnergy(const Mass<MassRatio>& mass) const {
        double v = toMetersPerSecond();
        return 0.5 * mass.toKilograms() * v * v;  // Joules
    }

    template<typename MassRatio>
    constexpr double momentum(const Mass<MassRatio>& mass) const {
        return mass.toKilograms() * toMetersPerSecond();  // kg⋅m/s
    }
};

// Common velocity types
using MetersPerSecond = Velocity<std::ratio<1,1>, std::ratio<1,1>>;
using KilometersPerHour = Velocity<std::ratio<1000,1>, std::ratio<3600,1>>;
using MilesPerHour = Velocity<std::ratio<1609344,1000>, std::ratio<3600,1>>;
using FeetPerSecond = Velocity<std::ratio<3048,10000>, std::ratio<1,1>>;

// ============================================================================
// ACCELERATION (Derived: Velocity / Time)
// ============================================================================
template<typename DistanceRatio = std::ratio<1,1>, typename TimeRatio = std::ratio<1,1>>
class Acceleration {
public:
    using distance_ratio = DistanceRatio;
    using time_ratio = TimeRatio;
    double value;

    constexpr Acceleration() : value(0) {}
    constexpr explicit Acceleration(double v) : value(v) {}

    // Convert to SI units (m/s²)
    // C++11 constexpr: must be single return statement
    constexpr double toMetersPerSecondSquared() const {
        return value * (distance_ratio::num / static_cast<double>(distance_ratio::den))
                     * (time_ratio::den / static_cast<double>(time_ratio::num))
                     * (time_ratio::den / static_cast<double>(time_ratio::num));
    }

    constexpr double toGravities() const {
        return toMetersPerSecondSquared() / constants::GRAVITY_EARTH;
    }

    // Factory methods
    // C++11 constexpr: must be single return statement
    static constexpr Acceleration fromMetersPerSecondSquared(double mps2) {
        return Acceleration(mps2 * (distance_ratio::den / static_cast<double>(distance_ratio::num))
                                  / ((time_ratio::num / static_cast<double>(time_ratio::den))
                                  * (time_ratio::num / static_cast<double>(time_ratio::den))));
    }

    static constexpr Acceleration fromGravities(double g) {
        return fromMetersPerSecondSquared(g * constants::GRAVITY_EARTH);
    }

    // Standard arithmetic
    constexpr Acceleration operator+(const Acceleration& other) const {
        return Acceleration(value + other.value);
    }
    constexpr Acceleration operator-(const Acceleration& other) const {
        return Acceleration(value - other.value);
    }
    constexpr Acceleration operator*(double scalar) const {
        return Acceleration(value * scalar);
    }
    constexpr Acceleration operator/(double scalar) const {
        return Acceleration(numerical::safeDivide(value, scalar));
    }
    constexpr Acceleration operator-() const { return Acceleration(-value); }
};

using MetersPerSecondSquared = Acceleration<std::ratio<1,1>, std::ratio<1,1>>;
using FeetPerSecondSquared = Acceleration<std::ratio<3048,10000>, std::ratio<1,1>>;
using StandardGravity = Acceleration<std::ratio<980665,100000>, std::ratio<1,1>>;

// ============================================================================
// FORCE
// ============================================================================
// Why separate class instead of derived:
// - Force has its own units and conversions
// - Cleaner API than Force<MassRatio, AccelRatio>
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class Force : public UnitBase<Force<Ratio>, Ratio> {
    using Base = UnitBase<Force<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions (SI: Newton = kg⋅m/s²)
    constexpr double toNewtons() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toPounds() const { return toNewtons() * 0.224808943; }
    constexpr double toKilogramForce() const { return toNewtons() / constants::GRAVITY_EARTH; }
    constexpr double toDynes() const { return toNewtons() * 100000.0; }

    // Factory methods
    static constexpr Force fromNewtons(double n) {
        return Force(n * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Force fromPounds(double lbf) {
        return fromNewtons(lbf / 0.224808943);
    }

    static constexpr Force fromKilogramForce(double kgf) {
        return fromNewtons(kgf * constants::GRAVITY_EARTH);
    }

    // Physics calculations
    template<typename DistRatio>
    constexpr double work(const Distance<DistRatio>& distance) const {
        return toNewtons() * distance.toMeters();  // Joules
    }

    template<typename DistRatio>
    constexpr double torque(const Distance<DistRatio>& leverArm) const {
        return toNewtons() * leverArm.toMeters();  // N⋅m
    }
};

using Newtons = Force<std::ratio<1,1>>;
using PoundsForce = Force<std::ratio<4448222,1000000>>;
using KilogramForce = Force<std::ratio<980665,100000>>;

namespace traits {
    template<typename R> struct is_unit<Force<R>> : std::true_type {};
}

// ============================================================================
// ENERGY
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class Energy : public UnitBase<Energy<Ratio>, Ratio> {
    using Base = UnitBase<Energy<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions (SI: Joule = N⋅m = kg⋅m²/s²)
    constexpr double toJoules() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toKilojoules() const { return toJoules() * 0.001; }
    constexpr double toCalories() const { return toJoules() * 0.239005736; }
    constexpr double toKilocalories() const { return toJoules() * 0.000239005736; }
    constexpr double toWattHours() const { return toJoules() / 3600.0; }
    constexpr double toKilowattHours() const { return toJoules() / 3600000.0; }

    // Factory methods
    static constexpr Energy fromJoules(double j) {
        return Energy(j * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Energy fromKilojoules(double kj) {
        return fromJoules(kj * 1000.0);
    }

    static constexpr Energy fromCalories(double cal) {
        return fromJoules(cal / 0.239005736);
    }

    static constexpr Energy fromWattHours(double wh) {
        return fromJoules(wh * 3600.0);
    }

    static constexpr Energy fromKilowattHours(double kwh) {
        return fromJoules(kwh * 3600000.0);
    }
};

using Joules = Energy<std::ratio<1,1>>;
using Kilojoules = Energy<std::ratio<1000,1>>;
using WattHours = Energy<std::ratio<3600,1>>;
using KilowattHours = Energy<std::ratio<3600000,1>>;

namespace traits {
    template<typename R> struct is_unit<Energy<R>> : std::true_type {};
}

// ============================================================================
// POWER
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class Power : public UnitBase<Power<Ratio>, Ratio> {
    using Base = UnitBase<Power<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions (SI: Watt = J/s = kg⋅m²/s³)
    constexpr double toWatts() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toKilowatts() const { return toWatts() * 0.001; }
    constexpr double toMegawatts() const { return toWatts() * 0.000001; }
    constexpr double toHorsepower() const { return toWatts() * 0.00134102209; }
    constexpr double toMilliwatts() const { return toWatts() * 1000.0; }

    // Factory methods
    static constexpr Power fromWatts(double w) {
        return Power(w * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Power fromKilowatts(double kw) {
        return fromWatts(kw * 1000.0);
    }

    static constexpr Power fromHorsepower(double hp) {
        return fromWatts(hp / 0.00134102209);
    }
};

using Watts = Power<std::ratio<1,1>>;
using Kilowatts = Power<std::ratio<1000,1>>;
using Megawatts = Power<std::ratio<1000000,1>>;
using Horsepower = Power<std::ratio<7456999,10000>>;

namespace traits {
    template<typename R> struct is_unit<Power<R>> : std::true_type {};
}

// ============================================================================
// TORQUE
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class Torque : public UnitBase<Torque<Ratio>, Ratio> {
    using Base = UnitBase<Torque<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions (SI: Newton-meter)
    constexpr double toNewtonMeters() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toPoundFeet() const { return toNewtonMeters() * 0.737562149; }
    constexpr double toPoundInches() const { return toNewtonMeters() * 8.85074579; }
    constexpr double toKilogramCentimeters() const { return toNewtonMeters() * 10.19716213; }

    // Factory methods
    static constexpr Torque fromNewtonMeters(double nm) {
        return Torque(nm * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Torque fromPoundFeet(double lbft) {
        return fromNewtonMeters(lbft / 0.737562149);
    }

    static constexpr Torque fromPoundInches(double lbin) {
        return fromNewtonMeters(lbin / 8.85074579);
    }
};

using NewtonMeters = Torque<std::ratio<1,1>>;
using PoundFeet = Torque<std::ratio<1355818,1000000>>;
using PoundInches = Torque<std::ratio<112985,1000000>>;

namespace traits {
    template<typename R> struct is_unit<Torque<R>> : std::true_type {};
}

// ============================================================================
// ANGULAR VELOCITY
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class AngularVelocity : public UnitBase<AngularVelocity<Ratio>, Ratio> {
    using Base = UnitBase<AngularVelocity<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    // Conversions (SI: rad/s)
    constexpr double toRadiansPerSecond() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toDegreesPerSecond() const {
        return toRadiansPerSecond() * constants::RAD_TO_DEG;
    }

    constexpr double toRPM() const {
        return toRadiansPerSecond() * 60.0 / constants::TWO_PI;
    }

    // Factory methods
    static constexpr AngularVelocity fromRadiansPerSecond(double radps) {
        return AngularVelocity(radps * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr AngularVelocity fromDegreesPerSecond(double degps) {
        return fromRadiansPerSecond(degps * constants::DEG_TO_RAD);
    }

    static constexpr AngularVelocity fromRPM(double rpm) {
        return fromRadiansPerSecond(rpm * constants::TWO_PI / 60.0);
    }

    // Calculate tangential velocity
    template<typename DistRatio>
    constexpr MetersPerSecond tangentialVelocity(const Distance<DistRatio>& radius) const {
        double v = toRadiansPerSecond() * radius.toMeters();
        return MetersPerSecond::fromMetersPerSecond(v);
    }
};

using RadiansPerSecond = AngularVelocity<std::ratio<1,1>>;
using DegreesPerSecond = AngularVelocity<std::ratio<17453293,1000000000>>;
using RPM = AngularVelocity<std::ratio<10471976,1000000000>>;  // 2π/60

namespace traits {
    template<typename R> struct is_unit<AngularVelocity<R>> : std::true_type {};
}

// ============================================================================
// ANGULAR ACCELERATION
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class AngularAcceleration : public UnitBase<AngularAcceleration<Ratio>, Ratio> {
    using Base = UnitBase<AngularAcceleration<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    constexpr double toRadiansPerSecondSquared() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toDegreesPerSecondSquared() const {
        return toRadiansPerSecondSquared() * constants::RAD_TO_DEG;
    }

    static constexpr AngularAcceleration fromRadiansPerSecondSquared(double radps2) {
        return AngularAcceleration(radps2 * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr AngularAcceleration fromDegreesPerSecondSquared(double degps2) {
        return fromRadiansPerSecondSquared(degps2 * constants::DEG_TO_RAD);
    }
};

using RadiansPerSecondSquared = AngularAcceleration<std::ratio<1,1>>;
using DegreesPerSecondSquared = AngularAcceleration<std::ratio<17453293,1000000000>>;

namespace traits {
    template<typename R> struct is_unit<AngularAcceleration<R>> : std::true_type {};
}

// ============================================================================
// FREQUENCY
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class Frequency : public UnitBase<Frequency<Ratio>, Ratio> {
    using Base = UnitBase<Frequency<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    constexpr double toHertz() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toKilohertz() const { return toHertz() * 0.001; }
    constexpr double toMegahertz() const { return toHertz() * 0.000001; }
    constexpr double toGigahertz() const { return toHertz() * 0.000000001; }

    static constexpr Frequency fromHertz(double hz) {
        return Frequency(hz * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Frequency fromKilohertz(double khz) {
        return fromHertz(khz * 1000.0);
    }

    static constexpr Frequency fromMegahertz(double mhz) {
        return fromHertz(mhz * 1000000.0);
    }

    // Convert to period
    constexpr Seconds toPeriod() const {
        return Seconds::fromSeconds(numerical::safeDivide(1.0, toHertz()));
    }
};

using Hertz = Frequency<std::ratio<1,1>>;
using Kilohertz = Frequency<std::ratio<1000,1>>;
using Megahertz = Frequency<std::ratio<1000000,1>>;

namespace traits {
    template<typename R> struct is_unit<Frequency<R>> : std::true_type {};
}

// ============================================================================
// ELECTRICAL UNITS
// ============================================================================
template<typename Ratio = std::ratio<1,1>>
class Voltage : public UnitBase<Voltage<Ratio>, Ratio> {
    using Base = UnitBase<Voltage<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    constexpr double toVolts() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toMillivolts() const { return toVolts() * 1000.0; }
    constexpr double toKilovolts() const { return toVolts() * 0.001; }

    static constexpr Voltage fromVolts(double v) {
        return Voltage(v * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Voltage fromMillivolts(double mv) {
        return fromVolts(mv * 0.001);
    }

    static constexpr Voltage fromKilovolts(double kv) {
        return fromVolts(kv * 1000.0);
    }
};

template<typename Ratio = std::ratio<1,1>>
class Current : public UnitBase<Current<Ratio>, Ratio> {
    using Base = UnitBase<Current<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    constexpr double toAmperes() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toMilliamperes() const { return toAmperes() * 1000.0; }
    constexpr double toMicroamperes() const { return toAmperes() * 1000000.0; }

    static constexpr Current fromAmperes(double a) {
        return Current(a * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Current fromMilliamperes(double ma) {
        return fromAmperes(ma * 0.001);
    }

    static constexpr Current fromMicroamperes(double ua) {
        return fromAmperes(ua * 0.000001);
    }
};

template<typename Ratio = std::ratio<1,1>>
class Resistance : public UnitBase<Resistance<Ratio>, Ratio> {
    using Base = UnitBase<Resistance<Ratio>, Ratio>;

public:
    using Base::Base;
    using Base::value;

    constexpr double toOhms() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }

    constexpr double toKiloohms() const { return toOhms() * 0.001; }
    constexpr double toMegaohms() const { return toOhms() * 0.000001; }

    static constexpr Resistance fromOhms(double ohms) {
        return Resistance(ohms * Ratio::den / static_cast<double>(Ratio::num));
    }

    static constexpr Resistance fromKiloohms(double kohms) {
        return fromOhms(kohms * 1000.0);
    }

    static constexpr Resistance fromMegaohms(double mohms) {
        return fromOhms(mohms * 1000000.0);
    }
};

using Volts = Voltage<std::ratio<1,1>>;
using Millivolts = Voltage<std::ratio<1,1000>>;
using Kilovolts = Voltage<std::ratio<1000,1>>;

using Amperes = Current<std::ratio<1,1>>;
using Milliamperes = Current<std::ratio<1,1000>>;
using Microamperes = Current<std::ratio<1,1000000>>;

using Ohms = Resistance<std::ratio<1,1>>;
using Kiloohms = Resistance<std::ratio<1000,1>>;
using Megaohms = Resistance<std::ratio<1000000,1>>;

namespace traits {
    template<typename R> struct is_unit<Voltage<R>> : std::true_type {};
    template<typename R> struct is_unit<Current<R>> : std::true_type {};
    template<typename R> struct is_unit<Resistance<R>> : std::true_type {};
}

// ============================================================================
// UNIT OPERATIONS (Physics relationships)
// ============================================================================
// These operator overloads implement fundamental physics laws as C++ operators.
//
// WHY OPERATOR OVERLOADS:
//   1. Natural syntax: velocity = distance / time (looks like math!)
//   2. Compile-time checking: Can't accidentally divide distance by mass
//   3. Automatic unit handling: No manual conversion needed
//   4. Zero runtime overhead: Resolved at compile time
//
// DIMENSIONAL ANALYSIS:
//   Physics equations must be dimensionally consistent:
//   - [Distance] / [Time] = [Velocity] ✓
//   - [Distance] / [Mass] = ??? ✗ (nonsensical!)
//   The compiler enforces these rules automatically.
//
// ============================================================================

// ============================================================================
// DISTANCE / TIME = VELOCITY
// ============================================================================
// PHYSICS LAW: velocity = distance / time
//
// EQUATION: v = d / t
//   Units: [m/s] = [m] / [s]
//
// EXAMPLE:
//   distance = 100 meters
//   time = 10 seconds
//   velocity = 100m / 10s = 10 m/s
//
// IN CODE:
//   Meters d(100);
//   Seconds t(10);
//   auto v = d / t;  // Automatically creates MetersPerSecond(10)
//
// WHY THIS WORKS:
//   Template parameters DR and TR carry the ratio information
//   Result is Velocity<DR, TR> with correct units
//   Division is just d.value / t.value (numbers)
//   Units are tracked at compile time (zero overhead!)
//
template<typename DR, typename TR>
inline constexpr Velocity<DR, TR> operator/(const Distance<DR>& d, const Time<TR>& t) {
    return Velocity<DR, TR>(numerical::safeDivide(d.value, t.value));
}

// ============================================================================
// VELOCITY × TIME = DISTANCE
// ============================================================================
// PHYSICS LAW: distance = velocity × time
//
// EQUATION: d = v × t
//   Units: [m] = [m/s] × [s]
//
// EXAMPLE:
//   velocity = 20 m/s (car speed)
//   time = 5 seconds
//   distance = 20 m/s × 5s = 100 meters
//
// KINEMATIC USE:
//   If robot travels at constant 0.5 m/s for 10 seconds:
//   distance = 0.5 m/s × 10s = 5 meters
//
// UNIT CANCELLATION:
//   [m/s] × [s] = [m × s] / [s] = [m] ✓
//
template<typename DR, typename TR>
inline constexpr Distance<DR> operator*(const Velocity<DR, TR>& v, const Time<TR>& t) {
    return Distance<DR>(v.value * t.value);
}

// ============================================================================
// VELOCITY / TIME = ACCELERATION
// ============================================================================
// PHYSICS LAW: acceleration = change in velocity / time
//
// EQUATION: a = Δv / t
//   Units: [m/s²] = [m/s] / [s]
//
// EXAMPLE:
//   Car accelerates from 0 to 30 m/s in 5 seconds
//   acceleration = 30 m/s / 5s = 6 m/s²
//
// MEANING:
//   Velocity increases by 6 m/s every second
//   After 1s: 6 m/s
//   After 2s: 12 m/s
//   After 3s: 18 m/s
//   etc.
//
// UNIT DERIVATION:
//   [m/s] / [s] = [m] / [s × s] = [m/s²] ✓
//
template<typename DR, typename TR>
inline constexpr Acceleration<DR, TR> operator/(const Velocity<DR, TR>& v, const Time<TR>& t) {
    return Acceleration<DR, TR>(numerical::safeDivide(v.value, t.value));
}

// ============================================================================
// ACCELERATION × TIME = VELOCITY
// ============================================================================
// PHYSICS LAW: velocity = acceleration × time (from rest)
//
// EQUATION: v = a × t (assuming v₀ = 0)
//   Units: [m/s] = [m/s²] × [s]
//
// KINEMATIC EQUATION: v = v₀ + a×t
//   This operator assumes v₀ = 0
//
// EXAMPLE:
//   acceleration = 2 m/s² (constant)
//   time = 5 seconds
//   final velocity = 2 m/s² × 5s = 10 m/s
//
// UNIT CANCELLATION:
//   [m/s²] × [s] = [m × s] / [s² × s] = [m/s] ✓
//
template<typename DR, typename TR>
inline constexpr Velocity<DR, TR> operator*(const Acceleration<DR, TR>& a, const Time<TR>& t) {
    return Velocity<DR, TR>(a.value * t.value);
}

// ============================================================================
// MASS × ACCELERATION = FORCE (Newton's Second Law)
// ============================================================================
// PHYSICS LAW: F = ma (Newton's Second Law of Motion)
//
// EQUATION: F = m × a
//   Units: [N] = [kg] × [m/s²]
//   (1 Newton = 1 kg·m/s²)
//
// MEANING:
//   Force needed to accelerate a mass
//   Heavier objects need more force for same acceleration
//
// EXAMPLES:
//   1. Push a 10 kg box with 2 m/s² acceleration:
//      F = 10 kg × 2 m/s² = 20 N
//
//   2. Robot (5 kg) accelerates at 0.5 m/s²:
//      F = 5 kg × 0.5 m/s² = 2.5 N (motor force needed)
//
//   3. Rocket (1000 kg) at 10 m/s²:
//      F = 1000 kg × 10 m/s² = 10,000 N (thrust required)
//
// INVERSE (a = F/m):
//   Same force on lighter object → higher acceleration
//   Same force on heavier object → lower acceleration
//
template<typename MR, typename DR, typename TR>
inline constexpr Newtons operator*(const Mass<MR>& m, const Acceleration<DR, TR>& a) {
    return Newtons::fromNewtons(m.toKilograms() * a.toMetersPerSecondSquared());
}

// Force / Mass = Acceleration
template<typename FR, typename MR>
inline constexpr MetersPerSecondSquared operator/(const Force<FR>& f, const Mass<MR>& m) {
    return MetersPerSecondSquared::fromMetersPerSecondSquared(
        numerical::safeDivide(f.toNewtons(), m.toKilograms())
    );
}

// Force * Distance = Energy (Work)
template<typename FR, typename DR>
inline constexpr Joules operator*(const Force<FR>& f, const Distance<DR>& d) {
    return Joules::fromJoules(f.toNewtons() * d.toMeters());
}

// Power * Time = Energy
template<typename PR, typename TR>
inline constexpr Joules operator*(const Power<PR>& p, const Time<TR>& t) {
    return Joules::fromJoules(p.toWatts() * t.toSeconds());
}

// Energy / Time = Power
template<typename ER, typename TR>
inline constexpr Watts operator/(const Energy<ER>& e, const Time<TR>& t) {
    return Watts::fromWatts(numerical::safeDivide(e.toJoules(), t.toSeconds()));
}

// Torque * Angular Velocity = Power
template<typename TorqueR, typename AngVelR>
inline constexpr Watts operator*(const Torque<TorqueR>& torque,
                                 const AngularVelocity<AngVelR>& omega) {
    return Watts::fromWatts(torque.toNewtonMeters() * omega.toRadiansPerSecond());
}

// ============================================================================
// VOLTAGE × CURRENT = POWER
// ============================================================================
// ELECTRICAL LAW: P = V × I
//
// EQUATION: P = V × I
//   Units: [W] = [V] × [A]
//   (1 Watt = 1 Volt × 1 Ampere)
//
// MEANING:
//   Electrical power consumed or delivered
//   Power = energy per unit time
//
// EXAMPLES:
//   1. Motor running at 12V drawing 2A:
//      P = 12V × 2A = 24W (motor consumes 24 watts)
//
//   2. LED at 3V drawing 20mA (0.02A):
//      P = 3V × 0.02A = 0.06W (LED consumes 60 milliwatts)
//
//   3. Battery (12V) delivering 10A:
//      P = 12V × 10A = 120W (battery provides 120 watts)
//
// PRACTICAL USE:
//   - Calculate battery life: energy = power × time
//   - Check if motor is overheating: high power = high heat
//   - Size power supply: must provide enough watts
//
template<typename VR, typename CR>
inline constexpr Watts operator*(const Voltage<VR>& v, const Current<CR>& i) {
    return Watts::fromWatts(v.toVolts() * i.toAmperes());
}

// ============================================================================
// VOLTAGE / CURRENT = RESISTANCE (Ohm's Law)
// ============================================================================
// ELECTRICAL LAW: V = I × R → R = V / I
//
// EQUATION: R = V / I
//   Units: [Ω] = [V] / [A]
//   (1 Ohm = 1 Volt / 1 Ampere)
//
// MEANING:
//   Resistance opposes current flow
//   Higher resistance → less current for same voltage
//
// EXAMPLES:
//   1. Circuit at 12V drawing 3A:
//      R = 12V / 3A = 4Ω (total resistance)
//
//   2. LED at 3V with 20mA (0.02A):
//      R = 3V / 0.02A = 150Ω (current-limiting resistor needed)
//
//   3. Motor at 6V drawing 0.5A:
//      R = 6V / 0.5A = 12Ω (motor's resistance)
//
// OHM'S LAW TRIANGLE:
//        V
//       ---
//      | ÷ |
//      |---|
//      I | R
//
//   V = I × R  (cover V, see I×R)
//   I = V / R  (cover I, see V/R)
//   R = V / I  (cover R, see V/I)
//
template<typename VR, typename CR>
inline constexpr Ohms operator/(const Voltage<VR>& v, const Current<CR>& i) {
    return Ohms::fromOhms(numerical::safeDivide(v.toVolts(), i.toAmperes()));
}

// ============================================================================
// CURRENT × RESISTANCE = VOLTAGE (Ohm's Law)
// ============================================================================
// ELECTRICAL LAW: V = I × R
//
// EQUATION: V = I × R
//   Units: [V] = [A] × [Ω]
//
// MEANING:
//   Voltage drop across a resistor
//   More current or higher resistance → bigger voltage drop
//
// EXAMPLES:
//   1. 2A through 10Ω resistor:
//      V = 2A × 10Ω = 20V (voltage drop)
//
//   2. 0.5A through 100Ω resistor:
//      V = 0.5A × 100Ω = 50V
//
//   3. LED circuit: Want 20mA through 150Ω resistor:
//      V = 0.02A × 150Ω = 3V (voltage drop across resistor)
//      If battery is 5V, LED gets 5V - 3V = 2V ✓
//
// USE CASE (Motor voltage drop):
//   Motor draws 5A, wire has 0.1Ω resistance
//   V_drop = 5A × 0.1Ω = 0.5V lost in wire!
//   If battery is 12V, motor only gets 11.5V
//
template<typename CR, typename RR>
inline constexpr Volts operator*(const Current<CR>& i, const Resistance<RR>& r) {
    return Volts::fromVolts(i.toAmperes() * r.toOhms());
}

// ============================================================================
// VOLTAGE / RESISTANCE = CURRENT (Ohm's Law)
// ============================================================================
// ELECTRICAL LAW: V = I × R → I = V / R
//
// EQUATION: I = V / R
//   Units: [A] = [V] / [Ω]
//
// MEANING:
//   Current flow for given voltage and resistance
//   Higher voltage → more current
//   Higher resistance → less current
//
// EXAMPLES:
//   1. 12V battery with 4Ω load:
//      I = 12V / 4Ω = 3A (current draw)
//
//   2. 5V across 1000Ω resistor:
//      I = 5V / 1000Ω = 0.005A = 5mA (small current)
//
//   3. Short circuit! 12V with 0.1Ω wire:
//      I = 12V / 0.1Ω = 120A (DANGER! Wire will melt!)
//
// PRACTICAL USE:
//   - Calculate current draw: Will circuit breaker trip?
//   - Size resistor: R = V / I_desired
//   - Check motor current: Is it within safe limits?
//
// WARNING:
//   Very low resistance → very high current!
//   Always include current limiting (fuses, resistors)
//
template<typename VR, typename RR>
inline constexpr Amperes operator/(const Voltage<VR>& v, const Resistance<RR>& r) {
    return Amperes::fromAmperes(numerical::safeDivide(v.toVolts(), r.toOhms()));
}

// Angle / Time = Angular Velocity
template<typename AR, typename TR>
inline constexpr RadiansPerSecond operator/(const Angle<AR>& angle, const Time<TR>& t) {
    return RadiansPerSecond::fromRadiansPerSecond(
        numerical::safeDivide(angle.toRadians(), t.toSeconds())
    );
}

// Angular Velocity * Time = Angle
template<typename AVR, typename TR>
inline constexpr Radians operator*(const AngularVelocity<AVR>& omega, const Time<TR>& t) {
    return Radians::fromRadians(omega.toRadiansPerSecond() * t.toSeconds());
}

// Angular Velocity / Time = Angular Acceleration
template<typename AVR, typename TR>
inline constexpr RadiansPerSecondSquared operator/(const AngularVelocity<AVR>& omega,
                                                   const Time<TR>& t) {
    return RadiansPerSecondSquared::fromRadiansPerSecondSquared(
        numerical::safeDivide(omega.toRadiansPerSecond(), t.toSeconds())
    );
}

// Helper namespace for physics literals
namespace literals {
    // Velocity
    inline constexpr MetersPerSecond mps(double val) {
        return MetersPerSecond::fromMetersPerSecond(val);
    }
    inline constexpr KilometersPerHour kmh(double val) {
        return KilometersPerHour::fromKilometersPerHour(val);
    }
    inline constexpr MilesPerHour mph(double val) {
        return MilesPerHour::fromMilesPerHour(val);
    }

    // Force
    inline constexpr Newtons N(double val) { return Newtons::fromNewtons(val); }
    inline constexpr PoundsForce lbf(double val) { return PoundsForce::fromPounds(val); }

    // Energy
    inline constexpr Joules J(double val) { return Joules::fromJoules(val); }
    inline constexpr WattHours Wh(double val) { return WattHours::fromWattHours(val); }

    // Power
    inline constexpr Watts W(double val) { return Watts::fromWatts(val); }
    inline constexpr Kilowatts kW(double val) { return Kilowatts::fromKilowatts(val); }
    inline constexpr Horsepower hp(double val) { return Horsepower::fromHorsepower(val); }

    // Torque
    inline constexpr NewtonMeters Nm(double val) { return NewtonMeters::fromNewtonMeters(val); }
    inline constexpr PoundFeet lbft(double val) { return PoundFeet::fromPoundFeet(val); }

    // Electrical
    inline constexpr Volts V(double val) { return Volts::fromVolts(val); }
    inline constexpr Amperes A(double val) { return Amperes::fromAmperes(val); }
    inline constexpr Ohms ohm(double val) { return Ohms::fromOhms(val); }

    // Angular
    inline constexpr RadiansPerSecond radps(double val) {
        return RadiansPerSecond::fromRadiansPerSecond(val);
    }
    inline constexpr RPM rpm(double val) { return RPM::fromRPM(val); }

    // Frequency
    inline constexpr Hertz Hz(double val) { return Hertz::fromHertz(val); }
    inline constexpr Kilohertz kHz(double val) { return Kilohertz::fromKilohertz(val); }
}

// Make physics literals available
using namespace literals;

} // namespace units

#endif // ROBOTICS_UNITS_PHYSICS_H
