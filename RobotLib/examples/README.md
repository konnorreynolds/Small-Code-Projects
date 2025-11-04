# RobotLib Examples

This directory contains comprehensive, working examples that demonstrate how to use RobotLib in real-world robotics applications.

## 📚 Example Index

### 01. Differential Drive Robot (`01_differential_drive_robot.cpp`)
**Difficulty**: ⭐ Beginner
**Topics**: Basic robot control, odometry, battery monitoring

A complete implementation of a differential drive robot (tank-style) controller. Learn how to:
- Convert linear and angular velocities to wheel speeds
- Calculate motor RPM from desired velocities
- Track robot position using odometry
- Monitor battery state of charge
- Control motors with type-safe units

**Compile**: `g++ -std=c++11 -I.. 01_differential_drive_robot.cpp -o diff_drive`

**What you'll learn**:
```cpp
// Convert velocity commands to motor RPM
auto drive = DifferentialDrive::fromTwist(
    mps(0.5),      // 0.5 m/s forward
    radps(0.3),    // 0.3 rad/s turning
    m(0.3)         // 30 cm wheelbase
);

auto leftRPM = MotorController::velocityToRPM(drive.leftVelocity, wheelDiameter);
auto rightRPM = MotorController::velocityToRPM(drive.rightVelocity, wheelDiameter);
```

---

### 02. PID Controller Tuning Guide (`02_pid_tuning_guide.cpp`)
**Difficulty**: ⭐⭐ Intermediate
**Topics**: PID control, tuning methodology, performance analysis

Interactive demonstration of PID controller tuning. Compares:
- P-only control (fast but steady-state error)
- PI control (eliminates error but can overshoot)
- Full PID control (optimal performance)
- Aggressive vs conservative tuning
- Anti-windup protection

**Compile**: `g++ -std=c++11 -I.. 02_pid_tuning_guide.cpp -o pid_tune`

**What you'll learn**:
- Ziegler-Nichols tuning method
- Manual tuning step-by-step process
- Performance metrics (overshoot, settling time, rise time)
- When to use P, PI, or PID
- Anti-windup techniques

**Output includes**:
- Step response plots
- Performance metric calculations
- Tuning guidelines and troubleshooting

---

### 03. Line Following Robot (`03_line_following_robot.cpp`)
**Difficulty**: ⭐⭐ Intermediate
**Topics**: Sensor arrays, filtering, path tracking

Complete line-following robot with sensor fusion and PID control. Features:
- 5-sensor IR array processing
- Moving average filtering for noise reduction
- Weighted line position calculation
- PID-based steering control
- Speed adaptation for curves
- Line lost detection

**Compile**: `g++ -std=c++11 -I.. 03_line_following_robot.cpp -o line_follow`

**What you'll learn**:
```cpp
// Sensor array with filtering
class SensorArray {
    std::array<MovingAverageFilter<3>, 5> filters_;

    double getLinePosition() const {
        // Weighted average: -1.0 (left) to +1.0 (right)
        ...
    }
};

// PID steering control
double steering = pid_.calculate(linePosition, dt);
```

**Real-world use cases**:
- Warehouse line-following robots
- Automated guided vehicles (AGVs)
- Competition robots (RoboCup, etc.)

---

### 04. Robot Arm Kinematics (`04_robot_arm_kinematics.cpp`)
**Difficulty**: ⭐⭐⭐ Advanced
**Topics**: Forward/inverse kinematics, Jacobian, trajectory planning

2-DOF planar robot arm with complete kinematic analysis. Demonstrates:
- **Forward kinematics**: Joint angles → end-effector position
- **Inverse kinematics**: Target position → joint angles
- Multiple IK solutions (elbow-up vs elbow-down)
- Workspace analysis and reachability
- Singularity detection
- Jacobian matrix calculation
- Trajectory planning (joint space and Cartesian space)

**Compile**: `g++ -std=c++11 -I.. 04_robot_arm_kinematics.cpp -o arm_kin`

**What you'll learn**:
```cpp
// Forward kinematics
auto pos = arm.forwardKinematics(JointState(deg(45), deg(60)));

// Inverse kinematics with multiple solutions
auto ikElbowUp = arm.inverseKinematics(target, RobotArm::IKSolution::ELBOW_UP);
auto ikElbowDown = arm.inverseKinematics(target, RobotArm::IKSolution::ELBOW_DOWN);

// Singularity check
if (arm.isNearSingularity(joints)) {
    // Handle reduced dexterity
}

// Jacobian for velocity analysis
auto J = arm.calculateJacobian(joints);
```

**Real-world applications**:
- Pick-and-place robots
- SCARA assembly arms
- Surgical robots
- 3D printer articulated heads

---

### 05. IMU Sensor Fusion (`05_sensor_fusion_imu.cpp`)
**Difficulty**: ⭐⭐⭐ Advanced
**Topics**: Sensor fusion, complementary filter, Kalman filter

Comprehensive comparison of sensor fusion techniques for IMU data. Compares:
1. **Accelerometer-only**: No drift but very noisy
2. **Gyroscope integration**: Smooth but drifts over time
3. **Complementary filter**: Simple fusion, good for most applications
4. **Kalman filter**: Optimal fusion with bias estimation

**Compile**: `g++ -std=c++11 -I.. 05_sensor_fusion_imu.cpp -o sensor_fusion`

**What you'll learn**:
```cpp
// Complementary filter (98% gyro, 2% accel)
ComplementaryFilterOrientation filter(0.98);
auto angle = filter.update(imuReading, dt);

// Kalman filter with bias estimation
KalmanFilterOrientation kalman;
auto angle = kalman.update(imuReading, dt);
double estimatedBias = kalman.getEstimatedBias();
```

**Performance comparison**:
- Mean absolute error for each method
- Drift analysis over time
- Computational cost comparison
- When to use each approach

**Extensions**:
- Add magnetometer for heading
- Implement full 3D orientation (quaternions)
- Madgwick/Mahony filters
- GPS fusion for position

---

## 🚀 Quick Start

### Running All Examples

```bash
cd RobotLib/examples

# Compile all examples
g++ -std=c++11 -I.. 01_differential_drive_robot.cpp -o diff_drive
g++ -std=c++11 -I.. 02_pid_tuning_guide.cpp -o pid_tune
g++ -std=c++11 -I.. 03_line_following_robot.cpp -o line_follow
g++ -std=c++11 -I.. 04_robot_arm_kinematics.cpp -o arm_kin
g++ -std=c++11 -I.. 05_sensor_fusion_imu.cpp -o sensor_fusion

# Run them
./diff_drive
./pid_tune
./line_follow
./arm_kin
./sensor_fusion
```

### On ESP32 with PlatformIO

Each example can be adapted for ESP32 by:
1. Create new PlatformIO project
2. Copy example to `src/main.cpp`
3. Copy library headers to `lib/RobotLib/`
4. Build and upload

Example `platformio.ini`:
```ini
[env:esp32-c3]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
build_flags = -std=c++11
```

---

## 📖 Learning Path

### Path 1: Beginner (Mobile Robots)
1. Start with **01_differential_drive_robot.cpp**
2. Learn PID control with **02_pid_tuning_guide.cpp**
3. Apply to line following with **03_line_following_robot.cpp**

### Path 2: Intermediate (Manipulators)
1. Review basics in **01_differential_drive_robot.cpp**
2. Master PID with **02_pid_tuning_guide.cpp**
3. Dive into kinematics with **04_robot_arm_kinematics.cpp**

### Path 3: Advanced (Sensor Fusion)
1. Understand filtering in **03_line_following_robot.cpp**
2. Study advanced fusion in **05_sensor_fusion_imu.cpp**
3. Combine with control from **02_pid_tuning_guide.cpp**

---

## 🎯 Common Patterns

### Pattern 1: Type-Safe Configuration
```cpp
struct RobotConfig {
    Meters wheelbase = m(0.3);
    Meters wheelDiameter = cm(6.5);
    MetersPerSecond maxSpeed = mps(0.5);
};
```

### Pattern 2: Sensor Filtering
```cpp
MovingAverageFilter<5> filter;
double filtered = filter.update(rawSensor);
```

### Pattern 3: PID Control
```cpp
PIDController pid(kP, kI, kD);
double output = pid.calculate(error, dt);
```

### Pattern 4: Differential Drive
```cpp
auto drive = DifferentialDrive::fromTwist(linear, angular, wheelbase);
auto leftRPM = MotorController::velocityToRPM(drive.leftVelocity, wheelDiameter);
```

---

## 🔧 Adapting for Your Robot

Each example is designed to be easily adapted:

1. **Change physical parameters**:
```cpp
// Adjust to match your robot
Meters wheelbase = m(0.25);        // Your wheelbase
Meters wheelDiameter = cm(8);      // Your wheels
```

2. **Tune control gains**:
```cpp
// Adjust PID gains for your system
PIDController pid(1.5, 0.2, 0.1);  // Tune these
```

3. **Modify sensor configuration**:
```cpp
// Change sensor array size
static constexpr size_t NUM_SENSORS = 7;  // Use 7 sensors instead of 5
```

4. **Add your hardware interface**:
```cpp
// Replace simulation with real hardware
void updateMotors(RPM left, RPM right) {
    motor_left.setPWM(rpmToPWM(left));
    motor_right.setPWM(rpmToPWM(right));
}
```

---

## 💡 Tips for Learning

1. **Start Simple**: Run examples as-is before modifying
2. **Read Comments**: Each example has extensive inline documentation
3. **Experiment**: Change parameters and observe effects
4. **Measure**: Use the performance metrics to validate improvements
5. **Combine**: Mix concepts from different examples
6. **Hardware**: Test on actual hardware after simulation

---

## 🐛 Troubleshooting

**Example won't compile**:
- Check you're using C++11 or newer: `g++ -std=c++11`
- Verify include path: `-I..` (examples are in subdirectory)
- Make sure all library headers are present

**Results look wrong**:
- Check unit conversions (meters vs centimeters, etc.)
- Verify your robot's physical parameters
- Tune PID gains for your specific system

**Performance issues**:
- Reduce filter size for faster computation
- Decrease sample rate if needed
- Check for numerical instabilities

---

## 📚 Further Reading

After completing these examples, explore:

- **RobotLib/QUICKSTART.md** - 5-minute tutorial
- **RobotLib/README.md** - Complete library documentation
- **RobotLib/ANALYSIS.md** - Deep technical insights
- **RobotLib/ESP32-C3_BUILD_REPORT.md** - ESP32 build guide

---

## 🤝 Contributing

Have an example you'd like to add? We welcome:
- New robot types (quadruped, omnidirectional, etc.)
- Advanced algorithms (MPC, adaptive control, etc.)
- Real hardware integrations (specific motor drivers, etc.)
- Competition robot examples (FIRST, VEX, etc.)

---

**Happy Robot Building! 🤖**
