# Professional Robotics Units Library v2.1
## A Type-Safe, Zero-Overhead Units System for Robotics & Engineering

## ⚠️ RECENT UPDATES (Nov 2025)
- ✅ **Fixed C++11 compatibility** (removed C++17 `inline constexpr`)
- ✅ **Fixed constexpr functions** for strict C++11 compliance
- ✅ **Enhanced Pose2D** to accept any Angle type (deg, rad, etc.)
- ✅ **Added utilities header** with streaming operators, differential drive, battery monitoring, and more!

See [CHANGES.md](CHANGES.md) for detailed changelog.

### 📁 File Structure

The library is now organized into four logical components:

1. **[units_core.h](units_core.h)** (28KB)
   - Foundation and base units
   - Mathematical constants and utilities
   - Core units: Distance, Time, Angle, Mass, Temperature
   - Helper functions for unit creation
   - Type traits and metaprogramming utilities

2. **[units_physics.h](units_physics.h)** (29KB)
   - Derived physics units
   - Velocity, Acceleration (linear and angular)
   - Force, Energy, Power, Torque
   - Electrical units (Voltage, Current, Resistance)
   - Frequency and related conversions
   - Automatic unit operations (F=ma, P=VI, etc.)

3. **[units_robotics.h](units_robotics.h)** (27KB)
   - Control systems and filters
   - 2D vectors and poses (SE(2))
   - PID and feedforward controllers
   - Motion profiles (trapezoid)
   - Digital filters (Kalman, complementary, low-pass, median)
   - Path following algorithms (Pure Pursuit)

4. **[units_utilities.h](units_utilities.h)** (13KB) ⭐ NEW!
   - Streaming operators (std::cout support)
   - Additional Vec2D utilities (perpendicular, clamp, etc.)
   - Differential drive kinematics
   - Battery monitoring utilities
   - Motor controller helpers (RPM conversion, deadband)
   - Exponential moving average filter
   - Simple odometry

5. **[units_math.h](units_math.h)** (13KB) ⭐ NEW!
   - Advanced mathematical functions for typed units
   - Square root and power operations
   - Interpolation (linear, cubic Hermite, smoothstep)
   - Ramp/slew rate limiting for smooth motion
   - Statistical functions (mean, variance, std deviation)
   - Angle wrapping and shortest angular distance
   - Hysteresis comparator (Schmitt trigger)
   - Numerical derivatives and first-order lag filter

Total size: ~110KB (header-only library)

## Why This Architecture?

### Naming Improvements
- **Clear purpose**: Each file name describes its contents
- **Logical dependencies**: Core → Physics → Robotics
- **Modular usage**: Include only what you need

### Design Decisions Explained

#### 1. **Why CRTP (Curiously Recurring Template Pattern)?**
```cpp
template<typename Derived, typename Ratio>
class UnitBase {
    // Shared implementation without virtual functions
};
```
- **Zero virtual function overhead** (critical for embedded systems)
- **Compile-time polymorphism** (no runtime dispatch)
- **Code reuse without inheritance penalty**
- **Each unit gets its own type** (prevents mixing incompatible units)

#### 2. **Why Template Ratios?**
```cpp
using Feet = Distance<std::ratio<3048, 10000>>;  // Exact: 0.3048
```
- **Compile-time unit tracking** (zero runtime cost)
- **Exact rational arithmetic** (no floating-point errors in conversions)
- **Type safety** (can't accidentally add feet to meters)

#### 3. **Why Separate Scalar Class?**
```cpp
class Scalar {  // Not derived from UnitBase
    // Different behavior than dimensional units
};
```
- **No unit conversions needed** (it's dimensionless)
- **Can implicitly convert to double** (convenient for math)
- **Represents pure numbers, ratios, percentages**

#### 4. **Why Factory Methods Instead of Constructors?**
```cpp
static constexpr Distance fromMeters(double m);
// Instead of: Distance(double m, Unit unit);
```
- **Type safety** (can't accidentally pass wrong unit)
- **Clear intent** (explicit about input units)
- **Compile-time optimization** (constexpr)
- **No runtime unit checking needed**

#### 5. **Why Safe Division Everywhere?**
```cpp
numerical::safeDivide(numerator, denominator, defaultValue);
```
- **Prevents undefined behavior** (division by zero)
- **Predictable results** (returns default value)
- **No exceptions needed** (important for embedded)

#### 6. **Why Derivative Filtering in PID?**
```cpp
double derivative = alpha * prev_derivative + (1-alpha) * raw_derivative;
```
- **Reduces noise amplification** (derivative is noise-sensitive)
- **Smoother control output** (less mechanical stress)
- **Tunable filtering** (adjust alpha for your system)

#### 7. **Why Anti-Windup in PID?**
```cpp
if (output != saturated_output) {
    integral_ -= excess / gains_.kI * 0.1;
}
```
- **Prevents integral buildup during saturation**
- **Faster recovery when coming out of saturation**
- **Better tracking performance**

## Usage Examples

### Basic Units
```cpp
#include "units_core.h"
using namespace units;

auto distance = m(10) + cm(50);           // 10.5 meters
auto angle = deg(45);                     // 45 degrees
auto temp = degC(25);                     // 25°C

std::cout << distance.toFeet() << " ft\n";
std::cout << angle.toRadians() << " rad\n";
std::cout << temp.toFahrenheit() << " °F\n";
```

### Physics Calculations
```cpp
#include "units_core.h"
#include "units_physics.h"
using namespace units;

auto mass = kg(10);
auto accel = MetersPerSecondSquared::fromMetersPerSecondSquared(9.8);
auto force = mass * accel;                // Automatic: F = ma

auto voltage = V(12);
auto current = A(2);
auto power = voltage * current;           // Automatic: P = VI

std::cout << force.toNewtons() << " N\n";
std::cout << power.toWatts() << " W\n";
```

### Robotics Control
```cpp
#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
using namespace units;

// PID Control with anti-windup
PIDController::Gains gains(1.2, 0.1, 0.05);
gains.outputMin = -1.0;
gains.outputMax = 1.0;
gains.iMax = 10.0;
PIDController pid(gains);

double error = setpoint - measurement;
double output = pid.calculate(error, dt);

// Motion profiling
TrapezoidProfile::Constraints constraints(3.0, 1.5);  // 3 m/s, 1.5 m/s²
TrapezoidProfile::State goal(10.0, 0, 0);  // Go to 10m
TrapezoidProfile profile(constraints, goal);

auto state = profile.calculate(t);
std::cout << "Position: " << state.position << " m\n";
std::cout << "Velocity: " << state.velocity << " m/s\n";

// Sensor fusion
ComplementaryFilter imu(0.98);
auto fusedAngle = imu.updateAngle(gyroRate, accelAngle, dt);
```

## Platform Support

### Embedded Platforms
- ✅ Arduino (AVR, ARM)
- ✅ ESP32/ESP8266
- ✅ STM32
- ✅ Raspberry Pi Pico
- ✅ Teensy

### Desktop/Server
- ✅ Linux/Unix
- ✅ Windows
- ✅ macOS
- ✅ Raspberry Pi

### Compiler Requirements
- C++11 minimum (full support)
- C++14/17/20 (enhanced constexpr)
- No RTTI required
- No exceptions required
- No dynamic allocation

## Performance Characteristics

| Operation | Runtime Cost | Memory Cost |
|-----------|-------------|-------------|
| Unit conversion | 0 (compile-time) | 0 |
| Type checking | 0 (compile-time) | 0 |
| Basic arithmetic | Same as double | 8 bytes per unit |
| Trig functions | Same as std::sin | 0 extra |
| PID calculation | ~20 FLOPs | 48 bytes state |
| Kalman filter | ~10 FLOPs | 32 bytes state |
| Trapezoid profile | ~15 FLOPs | 64 bytes state |

## Best Practices

### 1. Use Helper Functions
```cpp
// Good
auto dist = m(5.0);

// Verbose
auto dist = Meters::fromMeters(5.0);
```

### 2. Let Auto-Conversion Handle Units
```cpp
// Good
auto force = kg(10) * MetersPerSecondSquared::fromGravities(1);

// The library handles the conversion automatically
```

### 3. Use Appropriate Filters
- **Low-pass**: General smoothing, sensor noise
- **Median**: Spike/outlier rejection
- **Kalman**: Optimal for Gaussian noise
- **Complementary**: IMU sensor fusion
- **Moving Average**: Simple averaging

### 4. Configure PID Properly
```cpp
PIDController::Gains gains(kP, kI, kD);
gains.outputMin = -1.0;    // Always set output limits
gains.outputMax = 1.0;
gains.iMax = 10.0;         // Prevent windup
gains.kF = 0.5;            // Add feedforward if model known
```

## Migration from Original Library

### Changed Files
- `units_improved.h` → `units_core.h`
- `units_improved_part2.h` → `units_physics.h`
- `units_improved_part3.h` → `units_robotics.h`

### API Changes
- All core functionality remains the same
- Helper functions now in `units::literals` namespace (auto-imported)
- Robotics components in `units::robotics` namespace (auto-imported)

### Include Order
```cpp
#include "units_core.h"      // Always first
#include "units_physics.h"    // If needed
#include "units_robotics.h"   // If needed
```

## Testing Recommendations

1. **Unit Tests**
   - Test all conversions against known values
   - Verify compile-time type checking
   - Check edge cases (zero, negative, overflow)

2. **Integration Tests**
   - Test with real sensor data
   - Verify filter performance
   - Check control loop stability

3. **Performance Tests**
   - Benchmark against raw double arithmetic
   - Verify no runtime overhead
   - Check memory usage

## Future Enhancements

- [ ] 3D transformations (SE(3), quaternions)
- [ ] Extended Kalman Filter (EKF)
- [ ] Model Predictive Control (MPC)
- [ ] ROS2 message conversions
- [ ] MATLAB/Simulink code generation
- [ ] More path planning algorithms

## License

MIT License - Free for commercial and personal use

## Contributing

Contributions welcome! Please ensure:
- No dynamic allocation
- Maintain constexpr where possible
- Include unit tests
- Document mathematical derivations
- Follow existing naming conventions

## Credits

Developed for professional robotics applications with contributions from the open-source community.

---

*"Type safety with zero overhead - the C++ promise delivered"*
