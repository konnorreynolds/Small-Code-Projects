# 🤖 RobotLib v2.2
> A Type-Safe, Zero-Overhead Units System for Robotics & Engineering

[![C++11](https://img.shields.io/badge/C%2B%2B-11-blue.svg)](https://en.cppreference.com/w/cpp/11)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows%20%7C%20ESP32-green.svg)]()

## 🎯 Quick Start

```cpp
#include "RobotLib/include/units_core.h"
#include "RobotLib/include/units_physics.h"

using namespace units;

// Type-safe units prevent bugs at compile-time!
auto distance = m(5.0);
auto time = s(2.0);
auto velocity = distance / time;  // Automatically m/s!

std::cout << velocity.toMetersPerSecond() << " m/s\n";
```

**New to RobotLib?** Start with [examples/08_hello_units.cpp](examples/08_hello_units.cpp)!

## 📋 What's New in v2.2

- ✅ **Advanced 3D Robotics** - Quaternions, SE(3) transformations, SLERP interpolation
- ✅ **State Estimation** - Template-based Extended Kalman Filter (EKF)
- ✅ **Path Planning** - A* pathfinding and Dubins paths for car-like robots
- ✅ **Model Predictive Control** - MPC with constraints for optimal trajectory tracking
- ✅ **ROS2 Integration** - Message conversion helpers for ROS2 projects
- ✅ **MATLAB/Simulink Support** - Code generation and data export utilities
- ✅ **2 New Examples** - 3D quadcopter navigation and MPC trajectory tracking
- ✅ **Enhanced Documentation** - All advanced features fully documented

### What Was New in v2.1

- ✅ **3 New Beginner-Friendly Examples** (Hello Units, Battery Management, Swerve Drive)
- ✅ **Reorganized Documentation** - All reports now in `docs/` folder
- ✅ **100% Test Coverage** - 184 automated tests, all passing
- ✅ **Full C++11 Compatibility** - Works on embedded platforms
- ✅ **Comprehensive Guides** - Contributing, verification, testing

See [docs/CHANGES.md](docs/CHANGES.md) for complete changelog.

### 📁 File Structure

The library is now organized into four logical components:

1. **[units_core.h](include/units_core.h)** (28KB)
   - Foundation and base units
   - Mathematical constants and utilities
   - Core units: Distance, Time, Angle, Mass, Temperature
   - Helper functions for unit creation
   - Type traits and metaprogramming utilities

2. **[units_physics.h](include/units_physics.h)** (29KB)
   - Derived physics units
   - Velocity, Acceleration (linear and angular)
   - Force, Energy, Power, Torque
   - Electrical units (Voltage, Current, Resistance)
   - Frequency and related conversions
   - Automatic unit operations (F=ma, P=VI, etc.)

3. **[units_robotics.h](include/units_robotics.h)** (27KB)
   - Control systems and filters
   - 2D vectors and poses (SE(2))
   - PID and feedforward controllers
   - Motion profiles (trapezoid)
   - Digital filters (Kalman, complementary, low-pass, median)
   - Path following algorithms (Pure Pursuit)

4. **[units_utilities.h](include/units_utilities.h)** (13KB) ⭐ NEW!
   - Streaming operators (std::cout support)
   - Additional Vec2D utilities (perpendicular, clamp, etc.)
   - Differential drive kinematics
   - Battery monitoring utilities
   - Motor controller helpers (RPM conversion, deadband)
   - Exponential moving average filter
   - Simple odometry

5. **[units_math.h](include/units_math.h)** (13KB) ⭐ NEW!
   - Advanced mathematical functions for typed units
   - Square root and power operations
   - Interpolation (linear, cubic Hermite, smoothstep)
   - Ramp/slew rate limiting for smooth motion
   - Statistical functions (mean, variance, std deviation)
   - Angle wrapping and shortest angular distance
   - Hysteresis comparator (Schmitt trigger)
   - Numerical derivatives and first-order lag filter

6. **[units_3d.h](include/units_3d.h)** (21KB) ⭐ NEW in v2.2!
   - 3D vector operations (Vec3D)
   - Quaternion-based rotations (gimbal-lock free)
   - SE(3) transformations (Pose3D)
   - SLERP interpolation for smooth rotations
   - Euler angle conversions

7. **[units_estimation.h](include/units_estimation.h)** (13KB) ⭐ NEW in v2.2!
   - Extended Kalman Filter (template-based)
   - User-defined nonlinear models
   - Automatic Jacobian-based linearization
   - Pre-configured 2D position/velocity tracker
   - Covariance tracking and prediction/update steps

8. **[units_planning.h](include/units_planning.h)** (19KB) ⭐ NEW in v2.2!
   - A* pathfinding with obstacle avoidance
   - Dubins paths for car-like robots (all 6 types)
   - Path sampling and interpolation
   - Configurable costs and diagonal movement

9. **[units_control.h](include/units_control.h)** (16KB) ⭐ NEW in v2.2!
   - Model Predictive Control (MPC) framework
   - Linear system optimization with constraints
   - Receding horizon control
   - Pre-configured controllers (double integrator, velocity control)

10. **[units_ros2_interop.h](include/units_ros2_interop.h)** (15KB) ⭐ NEW in v2.2!
   - ROS2 message type conversions
   - geometry_msgs, sensor_msgs, nav_msgs support
   - Works with or without ROS2 installed
   - 2D and 3D pose/velocity/acceleration conversions

11. **[units_matlab_interop.h](include/units_matlab_interop.h)** (14KB) ⭐ NEW in v2.2!
   - MATLAB/Simulink integration helpers
   - CSV export for data analysis
   - Simulink Coder compatible C code generation
   - MEX function utilities

Total size: ~208KB (header-only library)

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

## 📚 Documentation

RobotLib includes comprehensive documentation:

- **[QUICKSTART.md](QUICKSTART.md)** - Step-by-step getting started guide
- **[examples/README.md](examples/README.md)** - All 10 examples documented
- **[docs/FINAL_VERIFICATION.md](docs/FINAL_VERIFICATION.md)** - Pre-use validation report
- **[docs/TEST_REPORT.md](docs/TEST_REPORT.md)** - Comprehensive testing results
- **[docs/FEATURE_SUMMARY.md](docs/FEATURE_SUMMARY.md)** - Complete API reference
- **[docs/ANALYSIS.md](docs/ANALYSIS.md)** - Architecture deep-dive
- **[docs/ESP32-C3_BUILD_REPORT.md](docs/ESP32-C3_BUILD_REPORT.md)** - Embedded platform guide
- **[docs/TESTING.md](docs/TESTING.md)** - How to run tests
- **[docs/CHANGES.md](docs/CHANGES.md)** - Version history
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - How to contribute

## 🎓 Examples

RobotLib includes 12 comprehensive examples:

### Beginner (⭐)
1. **Hello Units** - Your first type-safe program
2. **Differential Drive** - Basic robot control
3. **PID Tuning** - Controller tuning guide
4. **Line Following** - Sensor processing

### Intermediate (⭐⭐)
5. **Robot Arm** - Forward/inverse kinematics
6. **Sensor Fusion** - IMU data fusion
7. **Mecanum Drive** - Omnidirectional robot
8. **Motor Control** - Advanced velocity control
9. **Battery Management** - Power monitoring

### Advanced (⭐⭐⭐)
10. **Swerve Drive** - Professional holonomic system
11. **3D Quadcopter Navigation** - Quaternions, EKF, path planning ⭐ NEW!
12. **MPC Trajectory Tracking** - Model Predictive Control with constraints ⭐ NEW!

See [examples/README.md](examples/README.md) for details!

## 🧪 Testing

RobotLib has **184 automated tests** with **100% pass rate**:

```bash
# Run all tests
g++ -std=c++11 -Wall -Wextra -Wpedantic -Werror tests/test_compile_quick.cpp -o test && ./test
g++ -std=c++11 -Wall -Wextra -Wpedantic -Werror tests/test_math_features.cpp -o test_math && ./test_math
```

See [docs/TEST_REPORT.md](docs/TEST_REPORT.md) for detailed results.

## 🚀 Features Completed in v2.2

- [x] 3D transformations (SE(3), quaternions) ✅ **COMPLETE**
- [x] Extended Kalman Filter (EKF) ✅ **COMPLETE**
- [x] Path planning algorithms (A*, Dubins) ✅ **COMPLETE**
- [x] Model Predictive Control (MPC) ✅ **COMPLETE**
- [x] ROS2 message conversions ✅ **COMPLETE**
- [x] MATLAB/Simulink code generation ✅ **COMPLETE**

## 📄 License

MIT License - See [LICENSE](LICENSE) for details. Free for commercial and personal use!

## 🤝 Contributing

We welcome contributions! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

Quick checklist:
- ✅ C++11 compatible
- ✅ Zero warnings with strict flags
- ✅ Tests included
- ✅ Documentation updated
- ✅ Examples added (if applicable)

## 🙏 Credits

Developed for professional robotics applications with contributions from the open-source community.

**Used by**: FRC teams, VEX robotics, hobby builders, and professional developers!

---

## 🌟 Star This Project!

If RobotLib helped you build better robots, please star this repository! ⭐

---

*"Type safety with zero overhead - the C++ promise delivered"* 🚀
