# 🤖 RobotLib v2.2
> A Type-Safe, Zero-Overhead Units System for Robotics & Engineering

[![C++11](https://img.shields.io/badge/C%2B%2B-11-blue.svg)](https://en.cppreference.com/w/cpp/11)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20%7C%20ESP32%20%7C%20STM32%20%7C%20Teensy-green.svg)]()
[![AI-Assisted](https://img.shields.io/badge/AI-Assisted%20Development-purple.svg)](DISCLAIMER.md)

## ⚠️ Important: AI-Generated Library - Use at Your Own Discretion

**This library was developed with significant AI assistance (Claude by Anthropic).**

- ✅ **Extensively tested** with unit tests and examples
- ✅ **Simulation environment** for safe testing
- ⚠️ **Use at your own risk** - Review code before production use
- ⚠️ **No liability** - See [DISCLAIMER.md](DISCLAIMER.md) for full details

**Recommended for**: Education, hobby projects, prototyping, non-critical applications
**Requires validation for**: Commercial products, safety-critical systems

---

## 🎯 Quick Start

```cpp
#include <RobotLib.h>

using namespace units;
using namespace robotlib::output;

int main() {
    // Type-safe units prevent bugs at compile-time!
    auto distance = m(5.0);
    auto time = s(2.0);
    auto velocity = distance / time;  // Automatically m/s!

    println("Velocity: ", velocity.toMetersPerSecond(), " m/s");
    return 0;
}
```

**New to RobotLib?** Start with [examples/06_fluent_api/08_hello_units.cpp](examples/06_fluent_api/08_hello_units.cpp)!

## 📋 What's Inside

- ✅ **Type-Safe Units** - Compile-time dimensional analysis prevents unit errors
- ✅ **Robot Simulation** - Test algorithms before hardware deployment (SDL2-based)
- ✅ **Real-Time Visualization** - See your robot move in 2D simulation
- ✅ **Advanced Algorithms** - EKF, A*, MPC, path planning, kinematics
- ✅ **Platform Support** - Arduino, ESP32, STM32, Teensy, and desktop
- ✅ **Clean Output API** - Platform-agnostic logging (Arduino + PC)
- ✅ **26 Examples** - From basic motor control to advanced navigation
- ✅ **Zero Overhead** - Header-only, compile-time optimizations
- ✅ **C++11 Compatible** - Works on embedded platforms

## 🚀 Features

### Core Units System
- **Physical quantities**: Distance, Time, Angle, Mass, Temperature
- **Derived units**: Velocity, Acceleration, Force, Energy, Power, Torque
- **Electrical units**: Voltage, Current, Resistance
- **Automatic conversions**: m/s, ft/s, km/h, mph, etc.
- **Compile-time safety**: Can't mix incompatible units!

### Robotics Algorithms
- **Control**: PID, feedforward, motion profiles
- **Estimation**: Kalman filters, complementary filters, EKF
- **Planning**: A* pathfinding, Dubins paths, trajectory generation
- **Kinematics**: Differential drive, swerve drive, robot arm FK/IK
- **3D Math**: Quaternions, SE(3) transforms, SLERP interpolation

### Robot Simulation (Optional)
- **Physics simulation**: Realistic differential drive model
- **Collision detection**: Obstacles and world boundaries
- **Sensor simulation**: Ultrasonic, IR line sensors, encoders
- **Real-time visualization**: SDL2-based 2D rendering
- **Zero overhead**: Completely separate from main library

### Platform Examples
- **Arduino IDE**: Line follower, obstacle avoidance, odometry
- **PlatformIO**: ESP32 WiFi robot, STM32 RTOS, Teensy high-speed
- **Simulation**: Test algorithms before deploying to hardware

## 📦 Installation

### Arduino IDE

1. Download this repository
2. Copy `RobotLib` folder to your Arduino libraries directory:
   - **Windows**: `Documents\Arduino\libraries\`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`
3. Restart Arduino IDE
4. Include: `#include <RobotLib.h>`

### PlatformIO

**Option 1: library.json**
```ini
[env:myproject]
lib_deps =
    https://github.com/konnorreynolds/RobotLib.git#v2.2.0
```

**Option 2: Git submodule**
```bash
cd your-project
git submodule add https://github.com/konnorreynolds/RobotLib.git lib/RobotLib
```

### CMake

```cmake
add_subdirectory(lib/RobotLib)
target_link_libraries(my_robot RobotLib)
```

### Manual

Just copy `include/` folder to your project and add to include path.

## 📚 Documentation

- **[examples/README.md](examples/README.md)** - 20 core examples + 6 platform examples
- **[simulation/README.md](simulation/README.md)** - Robot simulation guide
- **[QUICKSTART.md](QUICKSTART.md)** - Step-by-step tutorial
- **[DISCLAIMER.md](DISCLAIMER.md)** - ⚠️ Important: AI generation notice and use guidelines
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - How to contribute
- **[CHANGELOG.md](CHANGELOG.md)** - Version history

## 🎓 Examples

### Basic Motor Control
```cpp
#include <RobotLib.h>

Arm motor = Arm();
motor.setDutyCycle(0.5);  // 50% power
motor.stop();
```

### PID Line Following
```cpp
PIDController pid(1.5, 0.3, 0.1);
double linePosition = readLineSensors();
double correction = pid.calculate(0.0, linePosition, dt);
robot.setMotorPWM(baseSpeed - correction, baseSpeed + correction);
```

### Robot Simulation
```cpp
#include <RobotLib.h>
#include "units_simulation.h"
#include "units_visualization.h"

DifferentialDriveSimulator robot(0.15, 0.10, 1000, 4.0, 3.0);
RobotVisualizer viz(800, 600, 150, "My Robot");

while (viz.isRunning()) {
    double distance = robot.getUltrasonicDistance(0);
    robot.setMotorPWM(distance < 0.3 ? -0.3 : 0.5,
                      distance < 0.3 ?  0.3 : 0.5);
    robot.update(0.02);
    viz.render(robot);
}
```

See [examples/](examples/) for 26 complete examples!

## 🛡️ Safety & Validation

**Before using in any robot:**

1. ✅ **Review the code** - Understand what it does
2. ✅ **Test in simulation** - Use the simulation system first
3. ✅ **Start simple** - Begin with basic examples
4. ✅ **Add safety measures** - Emergency stops, timeouts, limits
5. ✅ **Test incrementally** - One feature at a time
6. ✅ **Monitor closely** - Never leave robot unattended during testing

**Read [DISCLAIMER.md](DISCLAIMER.md) before use!**

## 🤝 Contributing

Contributions welcome! Please:

1. Read [CONTRIBUTING.md](CONTRIBUTING.md)
2. Fork the repository
3. Create a feature branch
4. Add tests for new features
5. Submit a pull request

Found a bug? Please [open an issue](https://github.com/konnorreynolds/RobotLib/issues)!

## 📄 License

MIT License - see [LICENSE](LICENSE) for details

**Copyright (c) 2025 Konnor Reynolds**

## ⚠️ Disclaimer

**This library is provided "as-is" with no warranty or liability.**

- Developed with AI assistance
- Use at your own risk
- Not suitable for safety-critical applications without professional review
- Author not liable for any damages

See [DISCLAIMER.md](DISCLAIMER.md) for complete details.

---

**Built with ❤️ and 🤖 AI assistance**

**Always prioritize safety when working with robots!** 🤖⚠️
