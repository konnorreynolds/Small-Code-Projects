# RobotLib Quick Usage Guide

## How to Include RobotLib in Your C++ Project

### Option 1: Single Header Include (Recommended)

Just include the main header and you're ready to go!

```cpp
#include <RobotLib.h>

int main() {
    using namespace units;

    auto distance = m(10.0);
    auto velocity = mps(2.5);

    std::cout << "Distance: " << distance.toMeters() << " m" << std::endl;
    return 0;
}
```

**Compile:**
```bash
g++ -std=c++11 -I/path/to/RobotLib/include main.cpp -o program
```

---

### Option 2: Individual Module Includes

For faster compile times, include only what you need:

```cpp
// Core units only
#include <units_core.h>
#include <units_physics.h>

// Or robotics features
#include <units_core.h>
#include <units_robotics.h>

// Or everything
#include <RobotLib.h>
```

---

## Project Structure Examples

### Structure 1: Copying to System Includes

```bash
sudo cp -r RobotLib/include/* /usr/local/include/
```

**Your project:**
```
MyRobot/
├── src/
│   └── main.cpp
└── CMakeLists.txt
```

**main.cpp:**
```cpp
#include <RobotLib.h>  // Works because it's in system includes!

int main() {
    auto dist = units::m(5.0);
    return 0;
}
```

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.10)
project(MyRobot)
set(CMAKE_CXX_STANDARD 11)

add_executable(myrobot src/main.cpp)
# No special include needed - it's in system path!
```

---

### Structure 2: Local Library Folder (Portable)

**Project structure:**
```
MyRobot/
├── src/
│   └── main.cpp
├── lib/
│   └── RobotLib/          # Copy entire RobotLib folder here
│       ├── include/
│       │   ├── RobotLib.h
│       │   ├── units_core.h
│       │   └── ...
│       └── examples/
└── CMakeLists.txt
```

**main.cpp:**
```cpp
#include <RobotLib.h>

int main() {
    using namespace units;
    auto dist = m(5.0);
    return 0;
}
```

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.10)
project(MyRobot)
set(CMAKE_CXX_STANDARD 11)

# Add RobotLib include directory
include_directories(${CMAKE_SOURCE_DIR}/lib/RobotLib/include)

add_executable(myrobot src/main.cpp)
```

**Build:**
```bash
mkdir build
cd build
cmake ..
make
```

---

### Structure 3: In-Tree with Submodules (Git Projects)

If your project is in Git:

```bash
cd MyRobot
git submodule add https://github.com/user/RobotLib.git lib/RobotLib
```

**Project structure:**
```
MyRobot/
├── src/
│   └── main.cpp
├── lib/
│   └── RobotLib/          # Git submodule
└── CMakeLists.txt
```

Same CMake setup as Structure 2.

---

## Common Usage Patterns

### Basic Units
```cpp
#include <RobotLib.h>
using namespace units;

// Create units
auto distance = m(10.0);        // 10 meters
auto velocity = mps(2.5);       // 2.5 m/s
auto angle = deg(90);           // 90 degrees
auto force = N(150);            // 150 Newtons

// Convert units
double meters = distance.toMeters();
double radians = angle.toRadians();
double degrees = angle.toDegrees();

// Type-safe math
auto area = m(5.0) * m(3.0);    // Compiles: Distance * Distance
// auto wrong = m(5.0) + s(3.0); // Error: Can't add Distance + Time!
```

### PID Control
```cpp
#include <RobotLib.h>
using namespace units::robotics;

PIDController pid(1.0, 0.1, 0.05);  // kP, kI, kD

// In your control loop:
double setpoint = 90.0;   // Target position
double current = 45.0;    // Current position
double dt = 0.02;         // 20ms control loop

double output = pid.calculate(setpoint, current, dt);
// Apply output to your motor
```

### Differential Drive Robot
```cpp
#include <RobotLib.h>
using namespace units;
using namespace units::utilities;

// Configure robot
auto wheelbase = m(0.5);  // 50cm between wheels

// Command robot to drive forward and turn
auto linear = mps(1.0);   // 1 m/s forward
auto angular = rad(0.5);  // 0.5 rad/s turning

// Calculate individual wheel speeds
DifferentialDrive drive = DifferentialDrive::fromTwist(
    linear, angular / s(1.0), wheelbase
);

std::cout << "Left wheel: " << drive.leftVelocity.toMetersPerSecond() << " m/s" << std::endl;
std::cout << "Right wheel: " << drive.rightVelocity.toMetersPerSecond() << " m/s" << std::endl;
```

### Kalman Filtering
```cpp
#include <RobotLib.h>
using namespace units::estimation;

// Create a 2D position+velocity tracker
EKF2DPositionVelocity ekf(0.01, 0.1);  // process noise, measurement noise

// Prediction step (called every dt)
ekf.predict(0.02);  // 20ms timestep

// Update step (when measurement arrives)
std::array<double, 2> gps_measurement = {x_measured, y_measured};
ekf.update(gps_measurement);

// Get estimated state [x, y, vx, vy]
auto state = ekf.getState();
std::cout << "Position: (" << state[0] << ", " << state[1] << ")" << std::endl;
```

### Path Planning
```cpp
#include <RobotLib.h>
using namespace units::planning;

// Create a 100x100 grid
AStarPlanner planner(100, 100);

// Mark obstacles
planner.setObstacle(50, 50);
planner.setObstacle(51, 50);
planner.setObstacle(52, 50);

// Plan path
GridCell start(10, 10);
GridCell goal(90, 90);
auto path = planner.plan(start, goal);

// Use the path
for (const auto& cell : path) {
    std::cout << "(" << cell.x << ", " << cell.y << ")" << std::endl;
}
```

### Fluent API (High-Level)
```cpp
#include <RobotLib.h>
using namespace robotlib;
using namespace units;

// Create and configure an arm with chaining
Arm leftArm = Arm()
    .withPID(1.5, 0.1, 0.05)
    .withLimits(deg(-90), deg(90))
    .withFeedforward(0.3, 0.002)
    .withSpeed(0.8);

// Control the arm
leftArm.moveTo(deg(45));

// In your control loop:
leftArm.update(0.02);  // 20ms update
double power = leftArm.getPower();
// Send power to motor...
```

---

## Troubleshooting Include Issues

### Issue: "RobotLib.h: No such file or directory"

**Solution 1: Check include path**
```bash
# Verify the file exists
ls /path/to/RobotLib/include/RobotLib.h

# Compile with explicit path
g++ -std=c++11 -I/path/to/RobotLib/include main.cpp
```

**Solution 2: CMake**
```cmake
# In CMakeLists.txt
include_directories(/path/to/RobotLib/include)
```

**Solution 3: Environment variable**
```bash
export CPLUS_INCLUDE_PATH=/path/to/RobotLib/include:$CPLUS_INCLUDE_PATH
```

### Issue: Compile errors about 'constexpr' or templates

**Solution:** Enable C++11 or newer
```bash
g++ -std=c++11 main.cpp        # C++11
g++ -std=c++14 main.cpp        # C++14 (better)
g++ -std=c++17 main.cpp        # C++17 (best)
```

### Issue: Linker errors

**Solution:** RobotLib is header-only, so no linking needed!
- Don't try to link against any RobotLib `.a` or `.so` files
- Only compile your own `.cpp` files

---

## Complete Example

**my_robot.cpp:**
```cpp
#include <RobotLib.h>
#include <iostream>

int main() {
    using namespace units;
    using namespace units::robotics;
    using namespace units::utilities;

    std::cout << "RobotLib v" << robotlib::VERSION_STRING << std::endl;

    // 1. Define robot parameters
    auto wheelbase = m(0.508);         // 20 inches
    auto wheelDiameter = m(0.1016);    // 4 inches
    auto maxRPM = rpm(200);

    // 2. Setup PID controller for driving straight
    PIDController drivePID(1.0, 0.1, 0.05);

    // 3. Command robot
    auto targetSpeed = mps(1.0);       // 1 m/s forward
    auto turnRate = rad(0.2) / s(1.0); // 0.2 rad/s turn

    // 4. Convert to wheel velocities
    auto drive = DifferentialDrive::fromTwist(targetSpeed, turnRate, wheelbase);

    // 5. Convert to motor RPM
    auto leftRPM = MotorController::velocityToRPM(drive.leftVelocity, wheelDiameter);
    auto rightRPM = MotorController::velocityToRPM(drive.rightVelocity, wheelDiameter);

    std::cout << "Left motor: " << leftRPM.toRPM() << " RPM" << std::endl;
    std::cout << "Right motor: " << rightRPM.toRPM() << " RPM" << std::endl;

    // 6. Convert to PWM duty cycle (0-1)
    double leftDuty = MotorController::rpmToPWM(leftRPM, maxRPM);
    double rightDuty = MotorController::rpmToPWM(rightRPM, maxRPM);

    std::cout << "Left PWM: " << leftDuty * 100 << "%" << std::endl;
    std::cout << "Right PWM: " << rightDuty * 100 << "%" << std::endl;

    return 0;
}
```

**Compile and run:**
```bash
g++ -std=c++11 -I./lib/RobotLib/include my_robot.cpp -o my_robot
./my_robot
```

**Output:**
```
RobotLib v2.2.0
Left motor: 182.96 RPM
Right motor: 217.04 RPM
Left PWM: 91.48%
Right PWM: 108.52%
```

---

## Next Steps

1. ✅ Include RobotLib in your project
2. 📖 Read module headers for detailed documentation
3. 💻 Check out [examples/](examples/) for more use cases
4. 🧪 Run the test suite in [tests/](tests/)
5. 🔬 Experiment with different modules!

---

## Module Reference

| Module | Include | Purpose |
|--------|---------|---------|
| Core | `units_core.h` | Basic units (distance, time, angle) |
| Physics | `units_physics.h` | Force, energy, voltage, etc. |
| Robotics | `units_robotics.h` | PID, filters, vectors |
| 3D Math | `units_3d.h` | Quaternions, 3D transforms |
| Utilities | `units_utilities.h` | Motor control, odometry |
| Estimation | `units_estimation.h` | Kalman filters |
| Planning | `units_planning.h` | A*, RRT, Dubins paths |
| Control | `units_control.h` | MPC, LQR |
| Fluent API | `robotlib_api.h` | High-level chainable interface |
| **All modules** | `RobotLib.h` | Everything! |

Enjoy using RobotLib! 🤖
