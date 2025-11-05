# 📚 RobotLib Examples

Comprehensive, working examples demonstrating RobotLib in real-world robotics applications.

## 🗂️ Organization

Examples are organized by complexity and feedback requirements:

### 📁 01_basics/ - Simple Control (No Feedback)
**Perfect for beginners** - Direct motor/servo control without sensors

- **simple_motor.cpp** - Basic motor control (open-loop)
- **simple_servo.cpp** - Hobby servo control (gripper, pan/tilt, arms)
- **simple_differential_drive.cpp** - Basic robot driving (no odometry)

**Use when**: Testing hardware, simple timed movements, no encoders available

**Compile**: `g++ -std=c++11 -I.. 01_basics/simple_motor.cpp -o simple_motor`

---

### 📁 02_with_feedback/ - Control with Sensors
**Intermediate** - Closed-loop control using sensor feedback

- **servo_with_position_feedback.cpp** - Servo with PID and position sensing
- **drive_with_odometry.cpp** - Robot navigation with wheel encoders

**Use when**: You have encoders, need accurate positioning, autonomous navigation

**Compile**: `g++ -std=c++11 -I.. 02_with_feedback/servo_with_position_feedback.cpp -o servo_feedback`

---

### 📁 03_full_systems/ - Complete Robot Systems
**Intermediate** - Real robot implementations

- **01_differential_drive_robot.cpp** - Complete tank-style robot
- **03_line_following_robot.cpp** - Line-following with sensor array
- **06_omnidirectional_robot.cpp** - Mecanum wheel robot
- **10_swerve_drive.cpp** - Advanced swerve drive system

**Use when**: Building a complete robot, need full system integration

**Compile**: `g++ -std=c++11 -I.. 03_full_systems/01_differential_drive_robot.cpp -o diff_drive`

---

### 📁 04_algorithms/ - Control Algorithms
**Intermediate to Advanced** - Specific control algorithms

- **02_pid_tuning_guide.cpp** - PID controller tuning
- **05_sensor_fusion_imu.cpp** - IMU sensor fusion (Kalman, complementary)
- **07_motor_control_encoders.cpp** - Feedforward + feedback motor control
- **12_mpc_trajectory_tracking.cpp** - Model predictive control

**Use when**: Learning control theory, optimizing performance

**Compile**: `g++ -std=c++11 -I.. 04_algorithms/02_pid_tuning_guide.cpp -o pid_tune`

---

### 📁 05_advanced/ - Advanced Topics
**Advanced** - Complex robotics concepts

- **04_robot_arm_kinematics.cpp** - Forward/inverse kinematics, Jacobian
- **11_3d_quadcopter_navigation.cpp** - 3D navigation, quaternions, EKF, pathfinding

**Use when**: Building manipulators, drones, complex systems

**Compile**: `g++ -std=c++11 -I.. 05_advanced/04_robot_arm_kinematics.cpp -o arm_kin`

---

### 📁 06_fluent_api/ - Fluent API Demos
**Beginner to Intermediate** - Beautiful chainable API

- **08_hello_units.cpp** - Gentle introduction to type-safe units
- **09_battery_management.cpp** - Battery monitoring and power management
- **13_fluent_api_demo.cpp** - Complete fluent API demonstration

**Use when**: Want easy-to-read code, teaching, rapid prototyping

**Compile**: `g++ -std=c++11 -I.. 06_fluent_api/08_hello_units.cpp -o hello`

---

## 🎯 Quick Start Paths

### Path 1: Complete Beginner
1. Start with `06_fluent_api/08_hello_units.cpp` - Learn the basics
2. Try `01_basics/simple_servo.cpp` - Control a servo
3. Move to `01_basics/simple_differential_drive.cpp` - Drive a robot

### Path 2: I Have Encoders
1. `02_with_feedback/servo_with_position_feedback.cpp` - Closed-loop control
2. `02_with_feedback/drive_with_odometry.cpp` - Track position
3. `03_full_systems/01_differential_drive_robot.cpp` - Complete robot

### Path 3: Learning Control Theory
1. `04_algorithms/02_pid_tuning_guide.cpp` - Master PID
2. `04_algorithms/07_motor_control_encoders.cpp` - Feedforward control
3. `04_algorithms/05_sensor_fusion_imu.cpp` - Sensor fusion
4. `04_algorithms/12_mpc_trajectory_tracking.cpp` - Optimal control

### Path 4: Building an Arm
1. `01_basics/simple_servo.cpp` - Basic servo control
2. `02_with_feedback/servo_with_position_feedback.cpp` - Add feedback
3. `05_advanced/04_robot_arm_kinematics.cpp` - Full kinematics

---

## 💡 Choosing the Right Example

### "I just want to make a motor spin"
→ `01_basics/simple_motor.cpp`

### "I have a servo-based gripper or arm"
→ `01_basics/simple_servo.cpp`

### "I want to drive a robot around (no sensors)"
→ `01_basics/simple_differential_drive.cpp`

### "I have wheel encoders and want accurate positioning"
→ `02_with_feedback/drive_with_odometry.cpp`

### "I'm building a competition robot"
→ Start with `03_full_systems/01_differential_drive_robot.cpp`

### "My control is wobbly, need to tune PID"
→ `04_algorithms/02_pid_tuning_guide.cpp`

### "Building a robot arm with inverse kinematics"
→ `05_advanced/04_robot_arm_kinematics.cpp`

### "I want the easiest, most readable API"
→ `06_fluent_api/13_fluent_api_demo.cpp`

---

## 📊 Comparison: Basic vs Feedback

| Feature | Basic | With Feedback |
|---------|-------|---------------|
| **Encoders needed** | ❌ No | ✅ Yes |
| **Position accuracy** | ⚠️ Low | ✅ High |
| **Code complexity** | ✅ Simple | ⚠️ Moderate |
| **Cost** | ✅ Lower | 💰 Higher |
| **Use case** | Testing, simple robots | Precision, autonomous |
| **Control loop** | Open-loop | Closed-loop |

---

**Happy Robot Building! 🤖**
