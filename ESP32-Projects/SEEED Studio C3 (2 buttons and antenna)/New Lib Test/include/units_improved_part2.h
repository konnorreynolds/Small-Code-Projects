// ============================================================================
// units_improved_part2.h - Derived Units and Robotics Components
// ============================================================================

#ifndef UNITS_PART2_H
#define UNITS_PART2_H

#include "units_improved.h"

namespace units {

// ============================================================================
// DERIVED UNITS - VELOCITY
// ============================================================================
template<typename DistanceRatio, typename TimeRatio>
class Velocity {
public:
    using distance_ratio = DistanceRatio;
    using time_ratio = TimeRatio;
    double value;
    
    constexpr Velocity() : value(0) {}
    constexpr explicit Velocity(double v) : value(v) {}
    
    // Conversion to standard units
    constexpr double toMetersPerSecond() const {
        constexpr double distScale = distance_ratio::num / static_cast<double>(distance_ratio::den);
        constexpr double timeScale = time_ratio::den / static_cast<double>(time_ratio::num);
        return value * distScale * timeScale;
    }
    
    constexpr double toKilometersPerHour() const {
        return toMetersPerSecond() * 3.6;
    }
    
    constexpr double toMilesPerHour() const {
        return toMetersPerSecond() * 2.23693629205;
    }
    
    constexpr double toFeetPerSecond() const {
        return toMetersPerSecond() * constants::METERS_TO_FEET;
    }
    
    constexpr double toKnots() const {
        return toMetersPerSecond() * 1.94384449244;
    }
    
    // Factory methods
    static constexpr Velocity fromMetersPerSecond(double mps) {
        constexpr double distScale = distance_ratio::den / static_cast<double>(distance_ratio::num);
        constexpr double timeScale = time_ratio::num / static_cast<double>(time_ratio::den);
        return Velocity(mps * distScale * timeScale);
    }
    
    static constexpr Velocity fromKilometersPerHour(double kmh) {
        return fromMetersPerSecond(kmh / 3.6);
    }
    
    static constexpr Velocity fromMilesPerHour(double mph) {
        return fromMetersPerSecond(mph / 2.23693629205);
    }
    
    static constexpr Velocity fromFeetPerSecond(double fps) {
        return fromMetersPerSecond(fps / constants::METERS_TO_FEET);
    }
    
    static constexpr Velocity fromKnots(double knots) {
        return fromMetersPerSecond(knots / 1.94384449244);
    }
    
    // Arithmetic operations
    constexpr Velocity operator+(const Velocity& other) const { 
        return Velocity(value + other.value); 
    }
    constexpr Velocity operator-(const Velocity& other) const { 
        return Velocity(value - other.value); 
    }
    constexpr Velocity operator*(double scalar) const { 
        return Velocity(value * scalar); 
    }
    constexpr Velocity operator/(double scalar) const { 
        return Velocity(numerical::safeDivide(value, scalar)); 
    }
    constexpr Velocity operator-() const { 
        return Velocity(-value); 
    }
    
    // Comparison operators
    constexpr bool operator==(const Velocity& other) const { 
        return numerical::approxEqual(value, other.value); 
    }
    constexpr bool operator!=(const Velocity& other) const { 
        return !(*this == other); 
    }
    constexpr bool operator<(const Velocity& other) const { 
        return value < other.value; 
    }
    constexpr bool operator<=(const Velocity& other) const { 
        return value <= other.value; 
    }
    constexpr bool operator>(const Velocity& other) const { 
        return value > other.value; 
    }
    constexpr bool operator>=(const Velocity& other) const { 
        return value >= other.value; 
    }
    
    // Calculate kinetic energy: KE = 0.5 * m * v^2
    template<typename MassRatio>
    double kineticEnergy(const Mass<MassRatio>& mass) const {
        double v = toMetersPerSecond();
        double m = mass.toKilograms();
        return 0.5 * m * v * v;  // Joules
    }
    
    // Calculate momentum: p = m * v
    template<typename MassRatio>
    double momentum(const Mass<MassRatio>& mass) const {
        return mass.toKilograms() * toMetersPerSecond();  // kg·m/s
    }
};

// Common velocity types
using MetersPerSecond = Velocity<std::ratio<1, 1>, std::ratio<1, 1>>;
using KilometersPerHour = Velocity<std::ratio<1000, 1>, std::ratio<3600, 1>>;
using MilesPerHour = Velocity<std::ratio<1609344, 1000>, std::ratio<3600, 1>>;
using FeetPerSecond = Velocity<std::ratio<3048, 10000>, std::ratio<1, 1>>;
using Knots = Velocity<std::ratio<1852, 1>, std::ratio<3600, 1>>;

// ============================================================================
// DERIVED UNITS - ACCELERATION
// ============================================================================
template<typename DistanceRatio, typename TimeRatio>
class Acceleration {
public:
    using distance_ratio = DistanceRatio;
    using time_ratio = TimeRatio;
    double value;
    
    constexpr Acceleration() : value(0) {}
    constexpr explicit Acceleration(double v) : value(v) {}
    
    // Conversion to standard units
    constexpr double toMetersPerSecondSquared() const {
        constexpr double distScale = distance_ratio::num / static_cast<double>(distance_ratio::den);
        constexpr double timeScale = time_ratio::den / static_cast<double>(time_ratio::num);
        return value * distScale * timeScale * timeScale;
    }
    
    constexpr double toGravities() const {
        return toMetersPerSecondSquared() / constants::GRAVITY_EARTH;
    }
    
    constexpr double toFeetPerSecondSquared() const {
        return toMetersPerSecondSquared() * constants::METERS_TO_FEET;
    }
    
    // Factory methods
    static constexpr Acceleration fromMetersPerSecondSquared(double mps2) {
        constexpr double distScale = distance_ratio::den / static_cast<double>(distance_ratio::num);
        constexpr double timeScale = time_ratio::num / static_cast<double>(time_ratio::den);
        return Acceleration(mps2 * distScale * timeScale * timeScale);
    }
    
    static constexpr Acceleration fromGravities(double g) {
        return fromMetersPerSecondSquared(g * constants::GRAVITY_EARTH);
    }
    
    static constexpr Acceleration fromFeetPerSecondSquared(double fps2) {
        return fromMetersPerSecondSquared(fps2 / constants::METERS_TO_FEET);
    }
    
    // Arithmetic operations
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
    constexpr Acceleration operator-() const { 
        return Acceleration(-value); 
    }
    
    // Calculate force: F = m * a
    template<typename MassRatio>
    double force(const Mass<MassRatio>& mass) const {
        return mass.toKilograms() * toMetersPerSecondSquared();  // Newtons
    }
};

// Common acceleration types
using MetersPerSecondSquared = Acceleration<std::ratio<1, 1>, std::ratio<1, 1>>;
using FeetPerSecondSquared = Acceleration<std::ratio<3048, 10000>, std::ratio<1, 1>>;
using StandardGravity = Acceleration<std::ratio<980665, 100000>, std::ratio<1, 1>>;

// ============================================================================
// ANGULAR VELOCITY
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class AngularVelocity : public Unit<AngularVelocity<Ratio>, Ratio> {
    using Base = Unit<AngularVelocity<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toRadiansPerSecond() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toDegreesPerSecond() const {
        return toRadiansPerSecond() * constants::RAD_TO_DEG;
    }
    
    constexpr double toRPM() const {
        return toRadiansPerSecond() * 60.0 / constants::TWO_PI;
    }
    
    constexpr double toHertz() const {
        return toRadiansPerSecond() / constants::TWO_PI;
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
    
    static constexpr AngularVelocity fromHertz(double hz) {
        return fromRadiansPerSecond(hz * constants::TWO_PI);
    }
    
    // Calculate tangential velocity: v = ω × r
    template<typename DistRatio>
    Velocity<DistRatio, std::ratio<1, 1>> tangentialVelocity(const Distance<DistRatio>& radius) const {
        double v = toRadiansPerSecond() * radius.toMeters();
        return Velocity<DistRatio, std::ratio<1, 1>>::fromMetersPerSecond(v);
    }
};

using RadiansPerSecond = AngularVelocity<std::ratio<1, 1>>;
using DegreesPerSecond = AngularVelocity<std::ratio<31415926535897932, 1800000000000000000>>;
using RevolutionsPerMinute = AngularVelocity<std::ratio<31415926535897932, 300000000000000000>>;

// ============================================================================
// ANGULAR ACCELERATION
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class AngularAcceleration : public Unit<AngularAcceleration<Ratio>, Ratio> {
    using Base = Unit<AngularAcceleration<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toRadiansPerSecondSquared() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toDegreesPerSecondSquared() const {
        return toRadiansPerSecondSquared() * constants::RAD_TO_DEG;
    }
    
    // Factory methods
    static constexpr AngularAcceleration fromRadiansPerSecondSquared(double radps2) {
        return AngularAcceleration(radps2 * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr AngularAcceleration fromDegreesPerSecondSquared(double degps2) {
        return fromRadiansPerSecondSquared(degps2 * constants::DEG_TO_RAD);
    }
};

using RadiansPerSecondSquared = AngularAcceleration<std::ratio<1, 1>>;
using DegreesPerSecondSquared = AngularAcceleration<std::ratio<31415926535897932, 1800000000000000000>>;

// ============================================================================
// FORCE
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Force : public Unit<Force<Ratio>, Ratio> {
    using Base = Unit<Force<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toNewtons() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toPounds() const {
        return toNewtons() * 0.224808943;
    }
    
    constexpr double toKilograms() const {
        return toNewtons() / constants::GRAVITY_EARTH;
    }
    
    constexpr double toDynes() const {
        return toNewtons() * 100000.0;
    }
    
    // Factory methods
    static constexpr Force fromNewtons(double n) {
        return Force(n * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Force fromPounds(double lbf) {
        return fromNewtons(lbf / 0.224808943);
    }
    
    static constexpr Force fromKilograms(double kgf) {
        return fromNewtons(kgf * constants::GRAVITY_EARTH);
    }
    
    static constexpr Force fromDynes(double dynes) {
        return fromNewtons(dynes * 0.00001);
    }
    
    // Calculate work/energy: W = F · d
    template<typename DistRatio>
    double work(const Distance<DistRatio>& distance) const {
        return toNewtons() * distance.toMeters();  // Joules
    }
    
    // Calculate torque: τ = r × F
    template<typename DistRatio>
    double torque(const Distance<DistRatio>& leverArm) const {
        return toNewtons() * leverArm.toMeters();  // N·m
    }
};

using Newtons = Force<std::ratio<1, 1>>;
using Pounds = Force<std::ratio<4448222, 1000000>>;
using Kilograms = Force<std::ratio<980665, 100000>>;
using Dynes = Force<std::ratio<1, 100000>>;

// ============================================================================
// TORQUE
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Torque : public Unit<Torque<Ratio>, Ratio> {
    using Base = Unit<Torque<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toNewtonMeters() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toPoundFeet() const {
        return toNewtonMeters() * 0.737562149;
    }
    
    constexpr double toPoundInches() const {
        return toNewtonMeters() * 8.85074579;
    }
    
    constexpr double toKilogramCentimeters() const {
        return toNewtonMeters() * 10.19716213;
    }
    
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
    
    static constexpr Torque fromKilogramCentimeters(double kgcm) {
        return fromNewtonMeters(kgcm / 10.19716213);
    }
    
    // Calculate power: P = τ × ω
    template<typename AngVelRatio>
    double power(const AngularVelocity<AngVelRatio>& angularVelocity) const {
        return toNewtonMeters() * angularVelocity.toRadiansPerSecond();  // Watts
    }
    
    // Calculate angular acceleration: α = τ / I
    double angularAcceleration(double momentOfInertia) const {
        return numerical::safeDivide(toNewtonMeters(), momentOfInertia);  // rad/s²
    }
};

using NewtonMeters = Torque<std::ratio<1, 1>>;
using PoundFeet = Torque<std::ratio<1355818, 1000000>>;
using PoundInches = Torque<std::ratio<112985, 1000000>>;
using KilogramCentimeters = Torque<std::ratio<980665, 100000000>>;

// ============================================================================
// ENERGY
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Energy : public Unit<Energy<Ratio>, Ratio> {
    using Base = Unit<Energy<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toJoules() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toKilojoules() const {
        return toJoules() * 0.001;
    }
    
    constexpr double toCalories() const {
        return toJoules() * 0.239005736;
    }
    
    constexpr double toKilocalories() const {
        return toJoules() * 0.000239005736;
    }
    
    constexpr double toWattHours() const {
        return toJoules() / 3600.0;
    }
    
    constexpr double toKilowattHours() const {
        return toJoules() / 3600000.0;
    }
    
    constexpr double toElectronVolts() const {
        return toJoules() * 6.241509074e18;
    }
    
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
    
    static constexpr Energy fromKilocalories(double kcal) {
        return fromJoules(kcal / 0.000239005736);
    }
    
    static constexpr Energy fromWattHours(double wh) {
        return fromJoules(wh * 3600.0);
    }
    
    static constexpr Energy fromKilowattHours(double kwh) {
        return fromJoules(kwh * 3600000.0);
    }
    
    static constexpr Energy fromElectronVolts(double ev) {
        return fromJoules(ev / 6.241509074e18);
    }
};

using Joules = Energy<std::ratio<1, 1>>;
using Kilojoules = Energy<std::ratio<1000, 1>>;
using Calories = Energy<std::ratio<4184, 1000>>;
using Kilocalories = Energy<std::ratio<4184, 1>>;
using WattHours = Energy<std::ratio<3600, 1>>;
using KilowattHours = Energy<std::ratio<3600000, 1>>;

// ============================================================================
// POWER
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Power : public Unit<Power<Ratio>, Ratio> {
    using Base = Unit<Power<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toWatts() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toKilowatts() const {
        return toWatts() * 0.001;
    }
    
    constexpr double toMegawatts() const {
        return toWatts() * 0.000001;
    }
    
    constexpr double toHorsepower() const {
        return toWatts() * 0.00134102209;
    }
    
    constexpr double toMilliwatts() const {
        return toWatts() * 1000.0;
    }
    
    // Factory methods
    static constexpr Power fromWatts(double w) {
        return Power(w * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Power fromKilowatts(double kw) {
        return fromWatts(kw * 1000.0);
    }
    
    static constexpr Power fromMegawatts(double mw) {
        return fromWatts(mw * 1000000.0);
    }
    
    static constexpr Power fromHorsepower(double hp) {
        return fromWatts(hp / 0.00134102209);
    }
    
    static constexpr Power fromMilliwatts(double mw) {
        return fromWatts(mw * 0.001);
    }
};

using Watts = Power<std::ratio<1, 1>>;
using Kilowatts = Power<std::ratio<1000, 1>>;
using Megawatts = Power<std::ratio<1000000, 1>>;
using Horsepower = Power<std::ratio<7456999, 10000>>;
using Milliwatts = Power<std::ratio<1, 1000>>;

// ============================================================================
// ELECTRICAL UNITS
// ============================================================================
template<typename Ratio = std::ratio<1, 1>>
class Voltage : public Unit<Voltage<Ratio>, Ratio> {
    using Base = Unit<Voltage<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toVolts() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toMillivolts() const {
        return toVolts() * 1000.0;
    }
    
    constexpr double toKilovolts() const {
        return toVolts() * 0.001;
    }
    
    // Factory methods
    static constexpr Voltage fromVolts(double v) {
        return Voltage(v * Ratio::den / static_cast<double>(Ratio::num));
    }
    
    static constexpr Voltage fromMillivolts(double mv) {
        return fromVolts(mv * 0.001);
    }
    
    static constexpr Voltage fromKilovolts(double kv) {
        return fromVolts(kv * 1000.0);
    }
    
    // Calculate power: P = V * I
    template<typename CurrentRatio>
    double power(const Current<CurrentRatio>& current) const {
        return toVolts() * current.toAmperes();  // Watts
    }
};

template<typename Ratio = std::ratio<1, 1>>
class Current : public Unit<Current<Ratio>, Ratio> {
    using Base = Unit<Current<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toAmperes() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toMilliamperes() const {
        return toAmperes() * 1000.0;
    }
    
    constexpr double toMicroamperes() const {
        return toAmperes() * 1000000.0;
    }
    
    // Factory methods
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

template<typename Ratio = std::ratio<1, 1>>
class Resistance : public Unit<Resistance<Ratio>, Ratio> {
    using Base = Unit<Resistance<Ratio>, Ratio>;
    
public:
    using Base::Base;
    using Base::value;
    
    constexpr double toOhms() const {
        return value * Ratio::num / static_cast<double>(Ratio::den);
    }
    
    constexpr double toKiloohms() const {
        return toOhms() * 0.001;
    }
    
    constexpr double toMegaohms() const {
        return toOhms() * 0.000001;
    }
    
    // Factory methods
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

using Volts = Voltage<std::ratio<1, 1>>;
using Millivolts = Voltage<std::ratio<1, 1000>>;
using Kilovolts = Voltage<std::ratio<1000, 1>>;

using Amperes = Current<std::ratio<1, 1>>;
using Milliamperes = Current<std::ratio<1, 1000>>;
using Microamperes = Current<std::ratio<1, 1000000>>;

using Ohms = Resistance<std::ratio<1, 1>>;
using Kiloohms = Resistance<std::ratio<1000, 1>>;
using Megaohms = Resistance<std::ratio<1000000, 1>>;

// ============================================================================
// UNIT OPERATIONS
// ============================================================================

// Distance / Time = Velocity
template<typename DR, typename TR>
constexpr Velocity<DR, TR> operator/(const Distance<DR>& d, const Time<TR>& t) {
    return Velocity<DR, TR>(numerical::safeDivide(d.value, t.value));
}

// Velocity * Time = Distance
template<typename DR, typename TR>
constexpr Distance<DR> operator*(const Velocity<DR, TR>& v, const Time<TR>& t) {
    return Distance<DR>(v.value * t.value);
}

// Velocity / Time = Acceleration
template<typename DR, typename TR>
constexpr Acceleration<DR, TR> operator/(const Velocity<DR, TR>& v, const Time<TR>& t) {
    return Acceleration<DR, TR>(numerical::safeDivide(v.value, t.value));
}

// Acceleration * Time = Velocity
template<typename DR, typename TR>
constexpr Velocity<DR, TR> operator*(const Acceleration<DR, TR>& a, const Time<TR>& t) {
    return Velocity<DR, TR>(a.value * t.value);
}

// Angle / Time = Angular Velocity
template<typename AR, typename TR>
constexpr AngularVelocity<> operator/(const Angle<AR>& angle, const Time<TR>& time) {
    return AngularVelocity<>::fromRadiansPerSecond(
        numerical::safeDivide(angle.toRadians(), time.toSeconds())
    );
}

// Angular Velocity * Time = Angle
template<typename AR, typename TR>
constexpr Angle<> operator*(const AngularVelocity<AR>& omega, const Time<TR>& t) {
    return Angle<>::fromRadians(omega.toRadiansPerSecond() * t.toSeconds());
}

// Force * Distance = Energy
template<typename FR, typename DR>
constexpr Energy<> operator*(const Force<FR>& force, const Distance<DR>& distance) {
    return Energy<>::fromJoules(force.toNewtons() * distance.toMeters());
}

// Torque * Angle = Energy
template<typename TorqueR, typename AngleR>
constexpr Energy<> operator*(const Torque<TorqueR>& torque, const Angle<AngleR>& angle) {
    return Energy<>::fromJoules(torque.toNewtonMeters() * angle.toRadians());
}

// Power * Time = Energy
template<typename PR, typename TR>
constexpr Energy<> operator*(const Power<PR>& power, const Time<TR>& time) {
    return Energy<>::fromJoules(power.toWatts() * time.toSeconds());
}

// Energy / Time = Power
template<typename ER, typename TR>
constexpr Power<> operator/(const Energy<ER>& energy, const Time<TR>& time) {
    return Power<>::fromWatts(numerical::safeDivide(energy.toJoules(), time.toSeconds()));
}

// Voltage * Current = Power
template<typename VR, typename CR>
constexpr Power<> operator*(const Voltage<VR>& voltage, const Current<CR>& current) {
    return Power<>::fromWatts(voltage.toVolts() * current.toAmperes());
}

// Voltage / Current = Resistance
template<typename VR, typename CR>
constexpr Resistance<> operator/(const Voltage<VR>& voltage, const Current<CR>& current) {
    return Resistance<>::fromOhms(numerical::safeDivide(voltage.toVolts(), current.toAmperes()));
}

// Voltage / Resistance = Current
template<typename VR, typename RR>
constexpr Current<> operator/(const Voltage<VR>& voltage, const Resistance<RR>& resistance) {
    return Current<>::fromAmperes(numerical::safeDivide(voltage.toVolts(), resistance.toOhms()));
}

// Current * Resistance = Voltage
template<typename CR, typename RR>
constexpr Voltage<> operator*(const Current<CR>& current, const Resistance<RR>& resistance) {
    return Voltage<>::fromVolts(current.toAmperes() * resistance.toOhms());
}

// Mass * Acceleration = Force
template<typename MR, typename DR, typename TR>
constexpr Force<> operator*(const Mass<MR>& mass, const Acceleration<DR, TR>& accel) {
    return Force<>::fromNewtons(mass.toKilograms() * accel.toMetersPerSecondSquared());
}

// Force / Mass = Acceleration
template<typename FR, typename MR>
constexpr Acceleration<std::ratio<1, 1>, std::ratio<1, 1>> 
operator/(const Force<FR>& force, const Mass<MR>& mass) {
    return Acceleration<std::ratio<1, 1>, std::ratio<1, 1>>::fromMetersPerSecondSquared(
        numerical::safeDivide(force.toNewtons(), mass.toKilograms())
    );
}

} // namespace units

#endif // UNITS_PART2_H