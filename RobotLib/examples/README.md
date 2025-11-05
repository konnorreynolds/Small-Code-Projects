# 📚 RobotLib Examples

Comprehensive, working examples demonstrating RobotLib in real-world robotics applications.

## 🎯 Quick Start

**New to RobotLib?** Start with **Example 08 (Hello Units)** for a gentle introduction!

## 📑 Example Index

### ⭐ Beginner Examples

### 08. Hello Units - Your First Program (`08_hello_units.cpp`) 🆕
**Difficulty**: ⭐ Beginner
**Topics**: Basic units, conversions, type safety

The perfect first program! Friendly introduction to type-safe units.

**Compile**: `g++ -std=c++11 -I.. 08_hello_units.cpp -o hello`

**Features**: Creating units, arithmetic, conversions, angles, temperature

---

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

### 06. Omnidirectional Robot (`06_omnidirectional_robot.cpp`)
**Difficulty**: ⭐⭐ Intermediate
**Topics**: Mecanum wheels, holonomic drive, field-centric control

Complete mecanum wheel robot with omnidirectional kinematics. Features:
- **Inverse kinematics**: Desired motion → wheel speeds
- **Forward kinematics**: Measured wheels → actual robot motion
- Holonomic motion (3 degrees of freedom)
- Field-centric control (drive relative to field, not robot)
- Velocity normalization for wheel saturation
- Omnidirectional odometry

**Compile**: `g++ -std=c++11 -I.. 06_omnidirectional_robot.cpp -o omni_robot`

**What you'll learn**:
```cpp
// Calculate wheel velocities for desired motion
auto wheels = drive.inverseKinematics(
    mps(0.5),    // Forward velocity
    mps(0.3),    // Strafe velocity (sideways!)
    radps(0.2)   // Rotation
);

// Field-centric control (drive relative to field)
auto robotVel = MecanumDrive::fieldToRobot(
    vx_field, vy_field, omega, robotHeading
);

// Normalize to prevent wheel saturation
wheels.normalize(maxSpeed);
```

**Real-world use cases**:
- Warehouse robots (AGVs)
- Competition robots (FRC, VEX)
- Mobile manipulation platforms
- Precision maneuvering in tight spaces

**Advantages over differential drive**:
- Can strafe sideways without turning
- Maintain heading while translating
- Superior maneuverability

---

### 07. Advanced Motor Control (`07_motor_control_encoders.cpp`)
**Difficulty**: ⭐⭐⭐ Advanced
**Topics**: Velocity control, encoder feedback, feedforward+feedback control

Professional-grade motor control with encoder feedback. Demonstrates:
- **Velocity control**: Precise RPM control with PID
- **Feedforward control**: Model-based control (kS, kV, kA)
- **Feedback control**: PID for error correction
- Encoder filtering (moving average + low-pass)
- Acceleration limiting for smooth motion
- Load disturbance rejection

**Compile**: `g++ -std=c++11 -I.. 07_motor_control_encoders.cpp -o motor_control`

**What you'll learn**:
```cpp
// Configure feedforward gains
controller.setFeedforward(
    0.5,   // kS: Static friction (volts)
    0.002, // kV: Velocity feedforward (V per RPM)
    0.01   // kA: Acceleration feedforward
);

// Configure feedback (PID) gains
controller.setVelocityPID(0.05, 0.5, 0.002);

// Control loop
auto output = controller.update(targetRPM, currentTime, dt);
motor.setVoltage(output.voltage);
```

**Key concepts**:
- Feedforward provides ~90% of control effort
- Feedback (PID) corrects for model errors and disturbances
- Acceleration limiting prevents wheel slip
- Encoder filtering reduces measurement noise

**Applications**:
- FRC/VEX competition robots
- Precision CNC machines
- Electric vehicle traction control
- Drone motor control
- Industrial automation

**Tuning procedure included**:
1. Measure kV (velocity feedforward)
2. Measure kS (static friction)
3. Tune feedback PID
4. Add kA for acceleration

---

### 09. Battery Management System (`09_battery_management.cpp`) 🆕
**Difficulty**: ⭐⭐ Intermediate
**Topics**: Power monitoring, battery health, electrical units

Practical battery management system with monitoring and safety features.

**Compile**: `g++ -std=c++11 -I.. 09_battery_management.cpp -o battery`

**Features**: Voltage/current monitoring, state of charge, power budgets, warnings

---

### 10. Swerve Drive Kinematics (`10_swerve_drive.cpp`) 🆕
**Difficulty**: ⭐⭐⭐ Advanced
**Topics**: Swerve drive, holonomic motion, advanced kinematics

Complete swerve drive implementation - the most advanced drive system!

**Compile**: `g++ -std=c++11 -I.. 10_swerve_drive.cpp -o swerve`

**Features**: Inverse/forward kinematics, module optimization, speed normalization, field-centric control

---

### 11. 3D Quadcopter Navigation (`11_3d_quadcopter_navigation.cpp`) 🆕
**Difficulty**: ⭐⭐⭐ Advanced
**Topics**: 3D transformations, quaternions, EKF, path planning

Complete demonstration of advanced robotics features for 3D navigation.

**Compile**: `g++ -std=c++11 -I.. 11_3d_quadcopter_navigation.cpp -o quadcopter`

**What you'll learn**:
```cpp
// Quaternion rotations (gimbal-lock free)
auto quat = Quaternion::fromAxisAngle(Vec3D(0, 0, 1), deg(90).toRadians());
auto rotated = quat.rotate(point);

// SLERP interpolation for smooth rotations
auto interpolated = q_start.slerp(q_end, t);

// Extended Kalman Filter for state estimation
EKF2DPositionVelocity ekf;
ekf.predict(dt);
ekf.update({gps_x, gps_y});  // Noisy measurements
auto state = ekf.getState();  // Filtered position/velocity

// A* pathfinding with obstacles
AStarPlanner planner(width, height);
planner.setObstacle(5, 5);
auto path = planner.plan(start, goal);

// Dubins paths for car-like robots
auto dubins = DubinsPath::shortestPath(start, goal, turning_radius);
auto samples = DubinsPath::sample(start, dubins, turning_radius, 0.1);
```

**Features**:
- 3D vector operations and transformations
- Quaternion-based rotations (no gimbal lock)
- SLERP interpolation for smooth motion
- Extended Kalman Filter with noisy GPS
- A* pathfinding with obstacle avoidance
- Dubins paths for smooth trajectories
- SE(3) transformations and pose composition

**Real-world applications**:
- Drone navigation and control
- 3D SLAM systems
- Robotic manipulators
- Autonomous vehicles
- Motion planning systems

---

### 12. MPC Trajectory Tracking (`12_mpc_trajectory_tracking.cpp`) 🆕
**Difficulty**: ⭐⭐⭐ Advanced
**Topics**: Model Predictive Control, optimal control, constraints

Complete demonstration of MPC for trajectory tracking with hard constraints.

**Compile**: `g++ -std=c++11 -I.. 12_mpc_trajectory_tracking.cpp -o mpc`

**What you'll learn**:
```cpp
// MPC for double integrator (position + velocity)
MPCDoubleIntegrator mpc(0.1);  // 0.1s time step
mpc.setLimits(2.0, 1.0);  // max velocity, max acceleration

// Solve optimization problem
auto control = mpc.solve(current_state, target_state);
double acceleration = control[0];

// Get predicted trajectory
auto trajectory = mpc.getPredictedTrajectory(current_state);
```

**Features**:
- Point-to-point motion with constraints
- Sinusoidal trajectory tracking
- MPC vs PID comparison
- Constraint satisfaction (velocity, acceleration limits)
- Receding horizon optimization
- Gradient descent solver (embedded-friendly)

**Real-world applications**:
- Autonomous vehicle path following
- Robot arm motion planning
- Drone trajectory optimization
- Constrained motion control
- Optimal trajectory generation

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
g++ -std=c++11 -I.. 06_omnidirectional_robot.cpp -o omni_robot
g++ -std=c++11 -I.. 07_motor_control_encoders.cpp -o motor_control
g++ -std=c++11 -I.. 08_hello_units.cpp -o hello
g++ -std=c++11 -I.. 09_battery_management.cpp -o battery
g++ -std=c++11 -I.. 10_swerve_drive.cpp -o swerve
g++ -std=c++11 -I.. 11_3d_quadcopter_navigation.cpp -o quadcopter
g++ -std=c++11 -I.. 12_mpc_trajectory_tracking.cpp -o mpc

# Run them (start with hello for beginners!)
./hello
./diff_drive
./pid_tune
./line_follow
./arm_kin
./sensor_fusion
./omni_robot
./motor_control
./battery
./swerve
./quadcopter
./mpc
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
