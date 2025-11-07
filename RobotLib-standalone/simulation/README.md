# Robot Simulation & Visualization

Test and visualize your robot code before deploying to hardware!

## Overview

The RobotLib simulation system allows you to:
- ✅ **Test robot algorithms** in a safe virtual environment
- ✅ **Visualize** robot behavior in real-time with SDL2
- ✅ **Debug** control parameters before hardware deployment
- ✅ **Develop offline** without needing physical robot
- ✅ **Use same code** for simulation and real hardware

## Features

### Physics Simulation (`units_simulation.h`)
- **Differential drive kinematics** - Accurate robot movement model
- **Collision detection** - Obstacles and world boundaries
- **Sensor simulation** - Ultrasonic, IR line sensors, encoders
- **Noise modeling** - Realistic sensor and motor noise
- **2D world** - Obstacles, boundaries, configurable size

### Visualization (`units_visualization.h`)
- **Real-time rendering** - SDL2-based graphics
- **Path tracing** - See where robot has been
- **Sensor rays** - Visualize ultrasonic sensors
- **Grid overlay** - Easy distance estimation
- **Smooth animation** - 50+ FPS typical

## Requirements

### SDL2 Library

**Ubuntu/Debian:**
```bash
sudo apt-get install libsdl2-dev
```

**macOS:**
```bash
brew install sdl2
```

**Windows:**
Download from [libsdl.org](https://www.libsdl.org/download-2.0.php)

## Building Examples

### Option 1: Make (Simplest)

```bash
cd simulation/
make              # Build all examples
make run-basic    # Build and run basic simulation
```

### Option 2: CMake (Recommended for large projects)

```bash
cd simulation/
mkdir build && cd build
cmake ..
make
./basic_simulation
```

### Option 3: Manual Compilation

```bash
g++ -std=c++11 -I../include 01_basic_simulation.cpp -lSDL2 -o basic_sim
./basic_sim
```

## Examples

### 01_basic_simulation.cpp
**Difficulty**: Beginner
**Description**: Simple obstacle avoidance robot

**Features**:
- Basic motor control
- Ultrasonic sensor reading
- Reactive obstacle avoidance
- Real-time visualization

**What you'll learn**:
- How to create a simulated robot
- How to read simulated sensors
- How to control motors
- How visualization works

**Run**:
```bash
make run-basic
```

---

### 02_pid_navigation.cpp
**Difficulty**: Intermediate
**Description**: Waypoint navigation using PID control

**Features**:
- PID controllers for heading and speed
- Multi-waypoint navigation
- Obstacle detection
- Path visualization

**What you'll learn**:
- PID controller tuning
- Waypoint following algorithms
- Combining multiple controllers
- Navigation strategies

**Run**:
```bash
make run-pid
```

---

### 03_line_follower.cpp
**Difficulty**: Intermediate
**Description**: Line following robot with 5 IR sensors

**Features**:
- 5-sensor array simulation
- Weighted line position algorithm
- PID line following
- Performance metrics

**What you'll learn**:
- Line sensor algorithms
- PID tuning for line following
- Sensor fusion
- Real-time control

**Run**:
```bash
make run-line
```

## Controls

All simulation windows support:
- **ESC** - Exit simulation
- **T** - Toggle path tracing
- **C** - Clear path

## Using Simulation in Your Code

### Basic Pattern

```cpp
#include <RobotLib.h>
#include "../include/units_simulation.h"
#include "../include/units_visualization.h"

using namespace robotlib::simulation;
using namespace robotlib::visualization;

int main() {
    // Create robot
    DifferentialDriveSimulator robot(
        0.15,  // wheelbase (m)
        0.10,  // radius (m)
        1000,  // encoder counts/m
        4.0,   // world width (m)
        3.0    // world height (m)
    );

    // Create visualizer
    RobotVisualizer viz(800, 600, 150, "My Robot");

    // Simulation loop
    double dt = 0.02;  // 20ms timestep
    while (viz.isRunning()) {
        viz.handleEvents();

        // Your robot code here!
        robot.setMotorPWM(0.5, 0.5);

        // Update physics
        robot.update(dt);

        // Render
        viz.render(robot);

        SDL_Delay((uint32_t)(dt * 1000));
    }

    return 0;
}
```

### Adding Obstacles

```cpp
robot.addObstacle(Rectangle(2.0, 1.5, 0.5, 0.5));  // x, y, width, height
robot.addObstacle(Rectangle(3.0, 2.0, 0.3, 0.8));
```

### Reading Sensors

```cpp
// Ultrasonic (returns distance in meters)
double distance = robot.getUltrasonicDistance(0);  // Forward sensor
double leftDist = robot.getUltrasonicDistance(M_PI/4);  // 45° left

// Line sensors (returns true if line detected)
bool leftSensor = robot.getLineSensor(0.05, -0.025);  // 5cm forward, 2.5cm left
bool centerSensor = robot.getLineSensor(0.05, 0);
bool rightSensor = robot.getLineSensor(0.05, 0.025);

// Encoders
long leftCount = robot.getLeftEncoder();
long rightCount = robot.getRightEncoder();
robot.resetEncoders();
```

### Motor Control

```cpp
// PWM control (-1.0 to 1.0)
robot.setMotorPWM(0.5, 0.5);    // Forward
robot.setMotorPWM(0.5, -0.5);   // Turn right
robot.setMotorPWM(-0.5, -0.5);  // Backward
robot.stop();

// Direct velocity (m/s)
robot.setWheelVelocities(0.3, 0.3);
```

### Getting Robot State

```cpp
double x = robot.getX();          // meters
double y = robot.getY();          // meters
double theta = robot.getTheta();  // radians

double leftVel = robot.getLeftVelocity();
double rightVel = robot.getRightVelocity();
```

## Simulation vs Real Hardware

### Code Portability

The simulation uses the same RobotLib API as real hardware. Here's how to structure code for both:

```cpp
// Configuration
#define USE_SIMULATION  // Comment out for real hardware

#ifdef USE_SIMULATION
    #include "../include/units_simulation.h"
    #include "../include/units_visualization.h"
    using Robot = robotlib::simulation::DifferentialDriveSimulator;
#else
    // Your real robot hardware interface
    #include "my_robot_hardware.h"
    using Robot = MyRobotHardware;
#endif

int main() {
    Robot robot(/* params */);

    // Rest of code is identical!
    while (running) {
        double distance = robot.getUltrasonicDistance(0);

        if (distance < 0.3) {
            robot.setMotorPWM(-0.3, 0.3);
        } else {
            robot.setMotorPWM(0.5, 0.5);
        }

        robot.update(dt);
    }
}
```

### Key Differences

| Feature | Simulation | Real Hardware |
|---------|------------|---------------|
| **Motor Control** | `setMotorPWM()` | digitalWrite/analogWrite |
| **Sensors** | `getUltrasonicDistance()` | pulseIn/digitalRead |
| **Update** | Call `update(dt)` | Automatic (interrupts) |
| **Time** | SDL_GetTicks() | millis() |
| **Visualization** | Built-in | External (optional) |

## Advanced Features

### Custom World Setup

```cpp
DifferentialDriveSimulator robot(
    0.15,   // wheelbase
    0.10,   // robot radius
    1000,   // encoder resolution
    10.0,   // larger world (10m x 8m)
    8.0
);

// Create maze
robot.addObstacle(Rectangle(5.0, 2.0, 0.1, 4.0));  // Vertical wall
robot.addObstacle(Rectangle(2.5, 4.0, 5.0, 0.1));  // Horizontal wall
// ... more obstacles
```

### Path Visualization

```cpp
viz.enableTrace(true);   // Enable path tracing
viz.clearPath();         // Clear traced path
```

### Performance Tuning

```cpp
// Higher resolution (more pixels per meter)
RobotVisualizer viz(1024, 768, 200, "High Res");

// Lower resolution (faster, less detail)
RobotVisualizer viz(640, 480, 100, "Fast");

// Adjust simulation timestep
double dt = 0.01;  // 10ms = 100 Hz (more accurate, slower)
double dt = 0.05;  // 50ms = 20 Hz (faster, less accurate)
```

## Tips for Success

✅ **Start simple** - Begin with basic_simulation.cpp and modify it
✅ **Tune in simulation** - Perfect PID parameters before deploying
✅ **Add obstacles** - Test edge cases and failure modes
✅ **Visualize sensors** - Use sensor rays to debug perception
✅ **Match hardware** - Use realistic wheelbase, speed, sensor range
✅ **Test algorithms** - Verify path planning, localization, etc.

## Troubleshooting

### "SDL2 not found"

**Solution**: Install SDL2 development libraries:
```bash
# Ubuntu/Debian
sudo apt-get install libsdl2-dev

# macOS
brew install sdl2

# Check installation
sdl2-config --version
```

### Window doesn't appear

**Solution**: Check if SDL2 is properly linked:
```bash
ldd basic_simulation | grep SDL2
```

### Compilation errors

**Solution**: Verify include path:
```bash
g++ -std=c++11 -I../include 01_basic_simulation.cpp -lSDL2 -o basic_sim
```

### Robot behaves strangely

**Solution**: Check simulation parameters:
- Wheelbase too small/large?
- Robot radius realistic?
- Timestep too large?
- Motor speeds reasonable?

## Performance

Typical performance on modern hardware:
- **60+ FPS** visualization
- **Sub-millisecond** physics update
- **Real-time** control (20-50 Hz loop)

## Zero Overhead for Production

**Important**: The simulation system is completely optional and adds **zero overhead** to embedded/production code.

### How?

1. **Separate headers** - `units_simulation.h` and `units_visualization.h` are separate from main library
2. **No dependencies** - Main RobotLib has no SDL2 or simulation dependencies
3. **Conditional compilation** - Use `#ifdef USE_SIMULATION` to exclude simulation code
4. **Clean separation** - Simulation code is in `simulation/` folder, not in main library

### Example Project Structure

```
my_robot_project/
├── src/
│   └── main.cpp          # Your robot code
├── simulation/
│   └── test.cpp          # Simulation test
├── include/
│   └── RobotLib/         # Main library (no sim)
└── CMakeLists.txt
```

## Next Steps

1. Build and run basic_simulation
2. Modify parameters (speeds, PID gains)
3. Add more obstacles
4. Create your own simulation
5. Port working code to real hardware!

## Resources

- [SDL2 Documentation](https://wiki.libsdl.org/)
- [RobotLib Examples](../examples/)
- [Main README](../README.md)

## Support

- Issues: GitHub Issues
- Questions: See main README

Happy simulating! 🤖✨
