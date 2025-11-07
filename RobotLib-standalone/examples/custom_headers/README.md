# Custom Headers - Extending RobotLib

This folder contains **example headers** showing how to extend RobotLib for your specific robot hardware and configuration.

## 📁 Files Overview

| File | Purpose | Key Concepts |
|------|---------|--------------|
| `custom_motor_driver.h` | Motor controller wrappers | Hardware abstraction, type-safe units |
| `custom_subsystems.h` | Subsystem organization | Update loop pattern, clean separation |
| `custom_robot_config.h` | Complete robot setup | Configuration management, autonomous |

## 🎯 Why These Examples?

**Problem**: Every robot is different!
- Different motor controllers (PWM, CAN, I2C)
- Different physical dimensions
- Different sensor configurations
- Different control requirements

**Solution**: These examples show you how to:
1. Wrap YOUR specific hardware
2. Use RobotLib's type-safe units everywhere
3. Organize multiple subsystems cleanly
4. Update everything in a single loop
5. Separate configuration from logic

---

## 📖 Detailed Guide

### 1. Custom Motor Drivers (`custom_motor_driver.h`)

Shows **three different motor driver patterns**:

#### Example 1: SimplePWMMotor
For basic PWM motor controllers (L298N, TB6612, motor shields, etc.)

```cpp
SimplePWMMotor motor(9, 8,           // PWM pin, DIR pin
                     rpm(200),        // Max speed
                     20.0);           // Gear ratio

motor.setDutyCycle(0.5);              // 50% power
motor.setVelocity(rpm(100));          // Type-safe velocity!
```

**Key Features**:
- Wraps Arduino `analogWrite()` and `digitalWrite()`
- Converts duty cycle (-1 to 1) to PWM (0-255)
- Accepts type-safe velocity units
- Handles direction and inversion

#### Example 2: CANMotorController
For CAN bus controllers (SparkMax, TalonSRX, CTRE, etc.)

```cpp
CANMotorController motor(5);          // CAN ID

motor.configurePID(1.0, 0.1, 0.05);   // PID on the controller
motor.setPosition(rad(1.57));         // Move to position
motor.setCurrentLimit(amp(20));       // Safety limit

auto pos = motor.getPosition();       // Read encoder
```

**Key Features**:
- Shows how to wrap vendor libraries
- Multiple control modes (duty, velocity, position)
- Built-in encoder feedback
- PID configuration

#### Example 3: ServoMotor
For hobby servos (SG90, MG996R, etc.)

```cpp
ServoMotor gripper(10,                // Servo pin
                   deg(0),             // Min angle
                   deg(180));          // Max angle

gripper.setAngle(deg(90));            // Using degrees
gripper.setAngle(rad(1.57));          // OR radians!
gripper.setNormalizedPosition(0.5);   // OR 0.0-1.0
```

**Key Features**:
- Template methods accept Degrees OR Radians
- Normalized position (0.0 to 1.0) option
- Configurable angle limits

---

### 2. Custom Subsystems (`custom_subsystems.h`)

Shows the **subsystem pattern** for organizing robot code.

#### The Pattern

Every subsystem follows this structure:

```cpp
class MySubsystem {
private:
    // Hardware (motors, sensors, etc.)
    // Configuration (physical constants)
    // State (current values)

public:
    // Constructor: initialize hardware
    MySubsystem() { }

    // UPDATE: called every loop (THIS IS THE KEY!)
    void update(double dt) {
        // 1. Read sensors
        // 2. Run controllers
        // 3. Update motors
    }

    // Commands: high-level control
    void doSomething() { }

    // Getters: query state
    double getValue() const { }
};
```

#### Four Example Subsystems

**1. DriveSubsystem**
- Manages drivetrain (motors + odometry)
- Uses `DifferentialDrive` from RobotLib
- Provides `arcade()` and `drive()` commands

**2. ArmSubsystem**
- Manages robotic arm with PID control
- Uses `Arm` controller from RobotLib
- Tracks target position and current state

**3. IntakeSubsystem**
- Simple motor for game piece manipulation
- Shows minimal subsystem pattern
- intake(), outtake(), stop() commands

**4. SensorSubsystem**
- Manages all sensors and filtering
- Shows filter integration
- Provides cleaned sensor data to other subsystems

#### The Magic: Complete Robot Class

```cpp
class MyRobot {
private:
    DriveSubsystem drive_;
    ArmSubsystem arm_;
    IntakeSubsystem intake_;
    SensorSubsystem sensors_;

public:
    // ONE METHOD UPDATES EVERYTHING!
    void update(double current_time) {
        double dt = current_time - last_update_time_;

        // Update all subsystems
        sensors_.update(dt);
        drive_.update(dt, encoder_left, encoder_right);
        arm_.update(dt);
        intake_.update(dt);
    }

    // Access subsystems
    DriveSubsystem& getDrive() { return drive_; }
    ArmSubsystem& getArm() { return arm_; }
    // ...
};
```

#### Your Main Loop Becomes SIMPLE:

```cpp
MyRobot robot;

void loop() {
    double current_time = millis() / 1000.0;

    // Update everything in one call!
    robot.update(current_time);

    // Control logic
    robot.getDrive().arcade(forward, turn);
    robot.getArm().moveTo(deg(90));
}
```

**Benefits**:
- ✅ All subsystems update with consistent timing
- ✅ Easy to add new subsystems
- ✅ Clean separation of concerns
- ✅ Simple main loop
- ✅ Easy to test individual subsystems

---

### 3. Robot Configuration (`custom_robot_config.h`)

Shows **best practices for robot configuration**.

#### Three Configuration Namespaces

**1. Hardware Configuration**
```cpp
namespace hardware {
    constexpr int DRIVE_LEFT_PWM = 9;
    constexpr int DRIVE_RIGHT_PWM = 11;
    constexpr int ARM_CAN_ID = 5;
    // ... all pin numbers
}
```
**Why**: Change pin numbers in ONE place when you rewire!

**2. Physical Constants**
```cpp
namespace physical {
    constexpr auto WHEELBASE = m(0.508);       // 20 inches
    constexpr auto WHEEL_DIAMETER = m(0.1016); // 4 inches
    constexpr auto ARM_LENGTH = m(0.6);        // 60cm
    // ... all physical measurements
}
```
**Why**: Accurate odometry requires accurate measurements!

**3. Control Parameters**
```cpp
namespace control {
    constexpr double ARM_KP = 1.5;
    constexpr double ARM_KI = 0.1;
    constexpr double ARM_KD = 0.08;
    // ... all tuned constants
}
```
**Why**: Tune PID values in ONE place!

#### Complete Robot Implementation

```cpp
class ConfiguredRobot {
    // Subsystems (using configuration)
    DriveSubsystem drive_;
    ArmSubsystem arm_;

public:
    ConfiguredRobot() {
        // Apply configuration
        drive_.withWheelbase(physical::WHEELBASE)
              .withWheelDiameter(physical::WHEEL_DIAMETER);

        arm_.withPID(control::ARM_KP, control::ARM_KI, control::ARM_KD)
            .withLimits(physical::ARM_MIN_ANGLE, physical::ARM_MAX_ANGLE);
    }

    void update(double time) { /* ... */ }

    // Autonomous routines built-in!
    void autonomousRoutine1() { /* ... */ }
};
```

---

## 🚀 Quick Start Guide

### Step 1: Copy the Examples

```bash
cp custom_motor_driver.h my_motor_driver.h
cp custom_subsystems.h my_subsystems.h
cp custom_robot_config.h my_robot.h
```

### Step 2: Customize for YOUR Robot

**In `my_robot.h`**:
1. Edit `hardware` namespace with YOUR pin numbers
2. Edit `physical` namespace with YOUR robot dimensions
3. Edit `control` namespace with YOUR tuned PID values

### Step 3: Use in Your Main File

```cpp
#include "my_robot.h"

ConfiguredRobot robot;

void setup() {
    Serial.begin(115200);
}

void loop() {
    double t = millis() / 1000.0;
    robot.update(t);

    // Your control logic here
    robot.getDrive().arcade(joystick_y, joystick_x);
}
```

**That's it!** The examples handle all the complexity.

---

## 💡 Design Patterns Used

### 1. Hardware Abstraction Layer (HAL)
- Motor drivers abstract hardware details
- Subsystems don't care about pin numbers
- Easy to swap hardware

### 2. Subsystem Pattern
- Each subsystem manages its own resources
- `update()` method called every loop
- Clean interfaces between subsystems

### 3. Configuration Separation
- Hardware config separate from logic
- Physical constants in one place
- Easy to support multiple robots

### 4. Type Safety
- RobotLib units prevent unit errors
- Compile-time checking
- Self-documenting code

---

## 📚 Advanced Topics

### Different Robot Configurations

**Support multiple robots** in same codebase:

```cpp
// robot_a_config.h
namespace hardware {
    constexpr int DRIVE_LEFT_PWM = 9;  // Robot A pins
}

// robot_b_config.h
namespace hardware {
    constexpr int DRIVE_LEFT_PWM = 3;  // Robot B pins
}

// main.cpp
#ifdef ROBOT_A
    #include "robot_a_config.h"
#else
    #include "robot_b_config.h"
#endif
```

### State Machines

Add **state machine** to subsystem:

```cpp
class ArmSubsystem {
    enum class State { STOWED, MOVING, HOLDING };
    State state_;

    void update(double dt) {
        switch(state_) {
            case State::STOWED:
                // Keep arm stowed
                break;
            case State::MOVING:
                // Move to target
                if (atTarget()) state_ = State::HOLDING;
                break;
            case State::HOLDING:
                // Hold position
                break;
        }
    }
};
```

### Logging and Debugging

Add **telemetry** to your robot:

```cpp
void MyRobot::update(double t) {
    // ... update subsystems ...

    // Log every 100ms
    if (t - last_log_time_ > 0.1) {
        Serial.print("Drive X: "); Serial.println(drive_.getX());
        Serial.print("Arm Pos: "); Serial.println(arm_.getPosition().toDegrees());
        last_log_time_ = t;
    }
}
```

---

## ✅ Checklist: Creating Custom Subsystem

- [ ] Define hardware (motors, sensors, etc.)
- [ ] Store configuration (physical constants)
- [ ] Track state (current position, velocity, etc.)
- [ ] Implement `update(double dt)` method
- [ ] Provide high-level command methods
- [ ] Provide getters for querying state
- [ ] Test subsystem standalone before integrating
- [ ] Document units and coordinate systems
- [ ] Add safety checks (limit switches, current limits)

---

## 🎓 Learning Path

1. **Start with**: `custom_motor_driver.h`
   - Learn hardware abstraction
   - See type-safe units in action

2. **Then read**: `custom_subsystems.h`
   - Understand subsystem pattern
   - See update loop in action

3. **Finally**: `custom_robot_config.h`
   - See complete robot organization
   - Learn configuration management

4. **Practice**: Create your own!
   - Start with one subsystem
   - Add more subsystems
   - Build complete robot

---

## 🤝 Contributing

If you create a cool custom subsystem or robot configuration:
1. Document it well
2. Add usage examples
3. Share with the community!

---

## 📞 Need Help?

- These are **example headers** (not actual hardware implementations)
- They show **patterns and best practices**
- Adapt them for **your specific hardware**
- Check main RobotLib documentation for core features

---

## 🎯 Key Takeaways

1. **One `update()` method** updates all subsystems
2. **Type-safe units** prevent errors
3. **Configuration separation** makes code reusable
4. **Subsystem pattern** keeps code organized
5. **Your main loop stays simple** and readable

**Now go build amazing robots!** 🤖
