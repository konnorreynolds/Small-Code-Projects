# 🚀 RobotLib Quickstart Guide

## Welcome to RobotLib!
In just **5 minutes**, you'll be writing type-safe robotics code with zero overhead.

---

## 📦 What You'll Need

- **C++11 compiler** (or newer)
- **5 minutes** of your time
- **An embedded project** (ESP32, Arduino, STM32, or desktop)

---

## 🎯 60-Second Example

Let's build a simple robot controller:

```cpp
#include "units_core.h"
#include "units_physics.h"
#include "units_utilities.h"

using namespace units;

int main() {
    // Type-safe units - no more unit bugs!
    auto wheelDiameter = cm(10);           // 10 centimeter wheels
    auto wheelbase = m(0.3);               // 30 cm between wheels
    auto targetSpeed = mps(0.5);           // 0.5 meters per second

    // Calculate wheel RPM needed
    auto wheelRPM = MotorController::velocityToRPM(targetSpeed, wheelDiameter);

    std::cout << "Set motor to: " << wheelRPM.toRPM() << " RPM\n";
    // Output: "Set motor to: 95.49 RPM"

    return 0;
}
```

**That's it!** Compile with: `g++ -std=c++11 -I. robot.cpp`

---

## 📚 Core Concepts in 3 Minutes

### 1. Creating Units (30 seconds)

```cpp
// Distance
auto d1 = m(10);          // 10 meters
auto d2 = cm(50);         // 50 centimeters
auto d3 = ft(5);          // 5 feet

// Time
auto t1 = s(5);           // 5 seconds
auto t2 = ms(1000);       // 1000 milliseconds

// Angle
auto a1 = deg(90);        // 90 degrees
auto a2 = rad(1.57);      // 1.57 radians

// Mass
auto m1 = kg(10);         // 10 kilograms
auto m2 = lb(22);         // 22 pounds
```

### 2. Converting Units (30 seconds)

```cpp
auto distance = m(10);

// Convert to any unit
double meters = distance.toMeters();      // 10.0
double feet = distance.toFeet();          // 32.808
double inches = distance.toInches();      // 393.701
```

### 3. Type-Safe Operations (30 seconds)

```cpp
// This works - same unit type
auto total = m(10) + cm(50);    // ✅ = 10.5 meters

// This fails at compile time - prevents bugs!
// auto bad = m(10) + s(5);     // ❌ ERROR: can't add distance and time
// auto bad = m(10) + kg(5);    // ❌ ERROR: can't add distance and mass
```

### 4. Physics Calculations (30 seconds)

```cpp
// Velocity = Distance / Time (automatic!)
auto velocity = m(100) / s(10);
std::cout << velocity.toMetersPerSecond();  // 10.0 m/s

// Force = Mass × Acceleration (automatic!)
auto force = kg(10) * MetersPerSecondSquared::fromGravities(1.0);
std::cout << force.toNewtons();  // 98.07 N

// Power = Voltage × Current (automatic!)
auto power = V(12) * A(5);
std::cout << power.toWatts();  // 60 W
```

### 5. Robotics Features (30 seconds)

```cpp
// 2D vectors
Vec2D position(10, 20);
Vec2D target(30, 40);

double distance = position.distanceTo(target);  // 28.28

// Robot poses (position + angle)
Pose2D robotPose(0, 0, deg(0));      // At origin, facing east
Vec2D localPoint(1, 0);               // 1 meter in front

Vec2D globalPoint = robotPose.toGlobal(localPoint);

// PID control
PIDController pid(1.0, 0.1, 0.05);    // kP, kI, kD
double error = target_position - current_position;
double output = pid.calculate(error, dt);
```

---

## 🎮 5 Common Use Cases

### Use Case 1: Differential Drive Robot

```cpp
#include "units_utilities.h"

// Robot specs
Meters wheelbase = m(0.3);
Meters wheelDiameter = cm(10);

// Command: go forward 1 m/s, turn 0.5 rad/s
auto drive = DifferentialDrive::fromTwist(
    mps(1.0),      // forward velocity
    radps(0.5),    // turning rate
    wheelbase      // wheelbase width
);

// Get individual wheel speeds
std::cout << "Left:  " << drive.leftVelocity.toMetersPerSecond() << " m/s\n";
std::cout << "Right: " << drive.rightVelocity.toMetersPerSecond() << " m/s\n";

// Convert to motor RPM
auto leftRPM = MotorController::velocityToRPM(drive.leftVelocity, wheelDiameter);
auto rightRPM = MotorController::velocityToRPM(drive.rightVelocity, wheelDiameter);
```

### Use Case 2: Battery Monitoring

```cpp
// Define battery specs
BatteryMonitor battery(
    V(12.0),   // nominal voltage
    V(10.0),   // minimum voltage
    V(12.6)    // maximum voltage
);

// In your loop:
void loop() {
    float voltage = analogRead(BATTERY_PIN) * (12.6 / 1023.0);
    battery.update(V(voltage));

    if (battery.isLow()) {
        Serial.println("Battery low! Return to base!");
    }

    Serial.print("Battery: ");
    Serial.print(battery.getPercentage());
    Serial.println("%");
}
```

### Use Case 3: Motion Profiling

```cpp
// Define motion limits
TrapezoidProfile::Constraints constraints(
    2.0,  // max velocity (m/s)
    1.0   // max acceleration (m/s²)
);

// Define goal
TrapezoidProfile::State goal(
    10.0,  // target position (m)
    0.0,   // target velocity (m/s)
    0.0    // target acceleration (m/s²)
);

TrapezoidProfile profile(constraints, goal);

// In your control loop:
void loop() {
    double t = millis() / 1000.0;  // time in seconds
    auto state = profile.calculate(t);

    // Use state.position, state.velocity, state.acceleration
    moveToPosition(state.position);
    setVelocity(state.velocity);
}
```

### Use Case 4: Sensor Filtering

```cpp
// Low-pass filter for noisy sensor
LowPassFilter distanceFilter(0.9);  // 90% smoothing

void loop() {
    float rawDistance = readUltrasonic();
    double filtered = distanceFilter.update(rawDistance);

    // Use filtered value
    if (filtered < 0.3) {
        stopRobot();
    }
}

// Moving average for IMU
MovingAverageFilter<10> gyroFilter;

void readIMU() {
    float rawGyro = readGyroscope();
    double smoothedGyro = gyroFilter.update(rawGyro);
    // Use smoothedGyro...
}
```

### Use Case 5: Odometry Tracking

```cpp
SimpleOdometry odom;

void loop() {
    // Read wheel encoders
    auto leftVel = mps(getLeftWheelVelocity());
    auto rightVel = mps(getRightWheelVelocity());
    auto now = s(millis() / 1000.0);

    // Update odometry
    odom.updateDifferential(leftVel, rightVel, m(0.3), now);

    // Get current position
    Pose2D currentPose = odom.getPose();
    Serial.print("X: "); Serial.println(currentPose.position.x);
    Serial.print("Y: "); Serial.println(currentPose.position.y);
    Serial.print("Heading: "); Serial.println(currentPose.theta.toDegrees());
}
```

---

## 🎓 Next Steps

### Level 1: Beginner (You are here!)
- ✅ Read this quickstart
- 📖 Review the [examples](examples/) directory
- 🧪 Run the [quick test](test_compile_quick.cpp)

### Level 2: Intermediate
- 📖 Read the full [README.md](README.md)
- 🔍 Study [CHANGES.md](CHANGES.md) to understand design decisions
- 💻 Build your first robot with RobotLib

### Level 3: Advanced
- 📖 Read [ESP32-C3_BUILD_REPORT.md](ESP32-C3_BUILD_REPORT.md)
- 🧠 Study [ANALYSIS.md](ANALYSIS.md) for deep insights
- 🤝 Contribute improvements!

---

## ⚠️ Common Pitfalls (Save Yourself Hours!)

### Pitfall 1: Forgetting to Convert
```cpp
// ❌ WRONG - No implicit conversion
double meters = m(10);  // ERROR!

// ✅ RIGHT - Explicit conversion
double meters = m(10).toMeters();  // OK!
```

### Pitfall 2: Mixing Unit Types
```cpp
// ❌ WRONG - Can't add different types
auto bad = m(10) + s(5);  // ERROR: Distance + Time

// ✅ RIGHT - Convert to same type first
auto d1 = m(10);
auto d2 = cm(50);
auto sum = d1 + d2;  // OK: Both are distances
```

### Pitfall 3: Using Raw Angles
```cpp
// ❌ LESS SAFE - Easy to mix radians and degrees
double angle = 1.57;
double result = sin(angle);  // Is this radians or degrees? 🤔

// ✅ BETTER - Type-safe angles
auto angle = rad(1.57);
double result = angle.sin();  // Clearly radians!

auto angle2 = deg(90);
double result2 = angle2.sin();  // Auto-converts to radians!
```

### Pitfall 4: Temperature Addition
```cpp
// ⚠️ CAREFUL - Physical meaning unclear
auto temp1 = degC(20);
auto temp2 = degC(30);
auto sum = temp1 + temp2;  // = 50°C... but what does this mean?

// ✅ BETTER - Use for temperature differences
double tempDiff = degC(30).toCelsius() - degC(20).toCelsius();  // 10°C difference
```

---

## 🔧 Platform-Specific Tips

### Arduino / ESP32
```cpp
void setup() {
    Serial.begin(115200);
}

void loop() {
    auto speed = mps(0.5);

    // Convert for display
    Serial.print("Speed: ");
    Serial.print(speed.toMetersPerSecond());
    Serial.println(" m/s");

    delay(100);
}
```

### STM32 (HAL)
```cpp
#include "main.h"
#include "units_core.h"

extern TIM_HandleTypeDef htim1;

void updateMotor(RPM targetRPM) {
    double rpm = targetRPM.toRPM();
    uint32_t pwm = (uint32_t)(rpm / MAX_RPM * 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}
```

### Raspberry Pi / Linux
```cpp
#include "units_utilities.h"
#include <iostream>

int main() {
    auto distance = m(10);

    // Use streaming operators
    std::cout << "Distance: " << distance << "\n";
    // Output: "Distance: 10.000 m"

    return 0;
}
```

---

## 🎯 Cheat Sheet

| Task | Code | Result |
|------|------|--------|
| Create distance | `m(10)` | 10 meters |
| Create velocity | `mps(5)` or `m(100)/s(20)` | 5 m/s |
| Create angle | `deg(90)` or `rad(1.57)` | 90° or 1.57 rad |
| Convert units | `m(10).toFeet()` | 32.808 |
| Calculate velocity | `m(100) / s(10)` | 10 m/s |
| Calculate force | `kg(10) * accel` | Force in Newtons |
| Vector distance | `v1.distanceTo(v2)` | Double |
| Normalize angle | `deg(450).normalizePositive()` | 90° |
| Filter sensor | `filter.update(reading)` | Filtered value |
| Run PID | `pid.calculate(error, dt)` | Control output |

---

## 🎊 Congratulations!

You've completed the quickstart guide! You now know enough to:
- ✅ Use type-safe units in your robot code
- ✅ Avoid common unit conversion bugs
- ✅ Perform physics calculations automatically
- ✅ Use robotics algorithms (PID, odometry, filters)

**Next**: Check out the [examples/](examples/) directory for complete projects!

---

## 💬 Need Help?

- 📖 Read the [full documentation](README.md)
- 🐛 Found a bug? Check [CHANGES.md](CHANGES.md)
- ❓ Question? Review [TESTING.md](TESTING.md)
- 💡 Want to contribute? See [ANALYSIS.md](ANALYSIS.md)

---

*Happy Robot Building! 🤖*
