# RobotLib with PlatformIO

## Quick Setup

Add this to your `platformio.ini`:

```ini
[env:your_board]
platform = ...
board = ...
framework = arduino

# Add RobotLib include path
build_flags =
    -std=c++11
    -I lib/RobotLib/include

# Enable C++11
build_unflags = -std=gnu++11
```

---

## Complete PlatformIO Setup Guide

### Step 1: Project Structure

Your PlatformIO project should look like this:

```
MyRobotProject/
├── platformio.ini          ← Configuration file
├── src/
│   └── main.cpp           ← Your code
├── lib/
│   └── RobotLib/          ← Copy RobotLib here!
│       └── include/
│           ├── RobotLib.h
│           ├── units_core.h
│           └── ...
└── .pio/
```

### Step 2: Copy RobotLib to lib folder

```bash
# From your PlatformIO project root:
cp -r /path/to/RobotLib lib/

# Verify:
ls lib/RobotLib/include/RobotLib.h
# Should show the file
```

### Step 3: Configure platformio.ini

**For Arduino (Uno, Mega, Nano):**
```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino

build_flags =
    -std=c++11
    -I lib/RobotLib/include
    -D UNITS_EMBEDDED=1        ; Important for Arduino!

build_unflags = -std=gnu++11

lib_deps =
    # Any other libraries you need
```

**For ESP32:**
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

build_flags =
    -std=c++11
    -I lib/RobotLib/include
    ; ESP32 has plenty of RAM, no need for UNITS_EMBEDDED

build_unflags = -std=gnu++11
```

**For STM32:**
```ini
[env:bluepill]
platform = ststm32
board = bluepill_f103c8
framework = arduino

build_flags =
    -std=c++11
    -I lib/RobotLib/include
    -D UNITS_EMBEDDED=1

build_unflags = -std=gnu++11
```

**For Teensy:**
```ini
[env:teensy40]
platform = teensy
board = teensy40
framework = arduino

build_flags =
    -std=c++11
    -I lib/RobotLib/include
    ; Teensy 4.0 has lots of RAM, UNITS_EMBEDDED optional
```

### Step 4: Your Code (src/main.cpp)

```cpp
#include <Arduino.h>
#include <RobotLib.h>  // Include RobotLib!

using namespace units;
using namespace robotics;

// Robot configuration
const auto WHEELBASE = m(0.5);
const auto WHEEL_DIAMETER = m(0.1);

// PID controller
PIDController drivePID(1.0, 0.1, 0.05);

void setup() {
    Serial.begin(115200);

    // Wait for serial
    while (!Serial) { delay(10); }

    Serial.println("RobotLib on PlatformIO!");
    Serial.print("Version: ");
    Serial.println(robotlib::VERSION_STRING);

    // Test basic units
    auto distance = m(10.0);
    auto velocity = mps(2.5);

    Serial.print("Distance: ");
    Serial.print(distance.toMeters());
    Serial.println(" m");

    Serial.print("Velocity: ");
    Serial.print(velocity.toMetersPerSecond());
    Serial.println(" m/s");
}

void loop() {
    // Use PID controller
    double setpoint = 90.0;
    double current = analogRead(A0) * 180.0 / 1023.0;

    double output = drivePID.calculate(setpoint, current, 0.02);

    // Output to motor
    analogWrite(9, abs(output) * 255);

    delay(20);  // 50Hz control loop
}
```

### Step 5: Build and Upload

```bash
# Build
pio run

# Upload to board
pio run --target upload

# Monitor serial output
pio device monitor
```

---

## Common PlatformIO Issues

### ❌ Error: "RobotLib.h: No such file or directory"

**Problem:** Include path not set correctly.

**Solution:** Check your `platformio.ini`:
```ini
build_flags =
    -I lib/RobotLib/include    # Must be relative to project root!
```

**Verify structure:**
```bash
ls lib/RobotLib/include/RobotLib.h
# Should exist!
```

### ❌ Error: "'constexpr' does not name a type"

**Problem:** C++11 not enabled.

**Solution:** Add to `platformio.ini`:
```ini
build_flags = -std=c++11
build_unflags = -std=gnu++11
```

### ❌ Compilation is slow or fails on Arduino Uno

**Problem:** Not enough RAM/Flash for full RobotLib.

**Solution 1:** Enable embedded mode:
```ini
build_flags =
    -D UNITS_EMBEDDED=1
    -I lib/RobotLib/include
```

**Solution 2:** Include only what you need:
```cpp
// Instead of:
#include <RobotLib.h>

// Use specific modules:
#include <units_core.h>
#include <units_robotics.h>
```

### ❌ Error: "multiple definition of..."

**Problem:** RobotLib included in multiple .cpp files.

**Solution:** RobotLib is header-only, this should work. But if you get this error:

**platformio.ini:**
```ini
build_flags =
    -I lib/RobotLib/include
    -D UNITS_HEADER_ONLY=1     # Force header-only mode
```

---

## Complete Example Project

### platformio.ini
```ini
; PlatformIO Project Configuration for RobotLib

[platformio]
default_envs = uno

[env]
; Common settings for all environments
build_flags =
    -std=c++11
    -I lib/RobotLib/include
    -D UNITS_EMBEDDED=1

build_unflags = -std=gnu++11

[env:uno]
; Arduino Uno (limited RAM)
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 115200

[env:mega]
; Arduino Mega (more RAM, can disable UNITS_EMBEDDED)
platform = atmelavr
board = megaatmega2560
framework = arduino
monitor_speed = 115200
build_flags =
    ${env.build_flags}
    -U UNITS_EMBEDDED      ; Mega has enough RAM

[env:esp32]
; ESP32 (lots of RAM and Flash)
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags =
    -std=c++11
    -I lib/RobotLib/include
    ; No UNITS_EMBEDDED needed

[env:teensy40]
; Teensy 4.0 (very powerful)
platform = teensy
board = teensy40
framework = arduino
monitor_speed = 115200
build_flags =
    -std=c++11
    -I lib/RobotLib/include
    -O2                        ; Enable optimization
```

### src/main.cpp (Complete Robot Example)
```cpp
#include <Arduino.h>
#include <RobotLib.h>

using namespace units;
using namespace utilities;

// Hardware pins
const int MOTOR_LEFT_PWM = 5;
const int MOTOR_LEFT_DIR = 4;
const int MOTOR_RIGHT_PWM = 6;
const int MOTOR_RIGHT_DIR = 7;

const int ENCODER_LEFT = 2;
const int ENCODER_RIGHT = 3;

// Robot parameters
const auto WHEELBASE = m(0.508);         // 20 inches
const auto WHEEL_DIAMETER = m(0.1016);   // 4 inches
const auto MAX_SPEED = mps(1.0);         // 1 m/s max

// Controllers
robotics::PIDController leftPID(1.0, 0.0, 0.05);
robotics::PIDController rightPID(1.0, 0.0, 0.05);

// Encoder counts
volatile long leftCount = 0;
volatile long rightCount = 0;

// Interrupt handlers
void leftEncoderISR() {
    leftCount++;
}

void rightEncoderISR() {
    rightCount++;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  RobotLib on PlatformIO Demo        ║");
    Serial.print("║  Version: ");
    Serial.print(robotlib::VERSION_STRING);
    Serial.println("                      ║");
    Serial.println("╚══════════════════════════════════════╝\n");

    // Setup motor pins
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);

    // Setup encoders
    pinMode(ENCODER_LEFT, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rightEncoderISR, RISING);

    Serial.println("🤖 Robot initialized!");
    Serial.print("📏 Wheelbase: ");
    Serial.print(WHEELBASE.toMeters() * 100);
    Serial.println(" cm");
    Serial.print("⚙️  Max speed: ");
    Serial.print(MAX_SPEED.toMetersPerSecond());
    Serial.println(" m/s\n");
}

void loop() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Target velocities (could come from joystick/remote)
    auto targetLinear = mps(0.5);    // 0.5 m/s forward
    auto targetAngular = rad(0.2) / s(1.0);  // 0.2 rad/s turn

    // Calculate individual wheel speeds using differential drive
    DifferentialDrive drive = DifferentialDrive::fromTwist(
        targetLinear,
        targetAngular,
        WHEELBASE
    );

    // Get target RPM
    auto leftRPM = MotorController::velocityToRPM(drive.leftVelocity, WHEEL_DIAMETER);
    auto rightRPM = MotorController::velocityToRPM(drive.rightVelocity, WHEEL_DIAMETER);

    // Apply PID control
    // (In real robot: Read actual RPM from encoders)
    double leftPower = leftPID.calculate(leftRPM.toRPM(), 0, dt);
    double rightPower = rightPID.calculate(rightRPM.toRPM(), 0, dt);

    // Clamp to [-1, 1]
    leftPower = constrain(leftPower, -1.0, 1.0);
    rightPower = constrain(rightPower, -1.0, 1.0);

    // Set motor directions
    digitalWrite(MOTOR_LEFT_DIR, leftPower >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_DIR, rightPower >= 0 ? HIGH : LOW);

    // Set motor speeds
    analogWrite(MOTOR_LEFT_PWM, abs(leftPower) * 255);
    analogWrite(MOTOR_RIGHT_PWM, abs(rightPower) * 255);

    // Print status every second
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint > 1000) {
        lastPrint = currentTime;

        Serial.print("Left: ");
        Serial.print(leftRPM.toRPM());
        Serial.print(" RPM | Right: ");
        Serial.print(rightRPM.toRPM());
        Serial.print(" RPM | Encoders: L=");
        Serial.print(leftCount);
        Serial.print(" R=");
        Serial.println(rightCount);
    }

    delay(20);  // 50Hz control loop
}
```

---

## Testing Your Setup

**1. Quick compile test:**
```bash
cd YourPlatformIOProject
pio run
```

**2. If successful, you should see:**
```
Building .pio/build/uno/src/main.cpp.o
...
Linking .pio/build/uno/firmware.elf
Building .pio/build/uno/firmware.hex
...
SUCCESS
```

**3. Upload and monitor:**
```bash
pio run --target upload && pio device monitor
```

**Expected serial output:**
```
╔══════════════════════════════════════╗
║  RobotLib on PlatformIO Demo        ║
║  Version: 2.2.0                      ║
╚══════════════════════════════════════╝

🤖 Robot initialized!
📏 Wheelbase: 50.8 cm
⚙️  Max speed: 1 m/s

Left: 123.4 RPM | Right: 156.7 RPM | Encoders: L=450 R=523
...
```

---

## Memory Usage

RobotLib is designed for embedded systems!

**Arduino Uno (2KB RAM):**
- With `UNITS_EMBEDDED=1`: ~800 bytes
- Core modules only: ~600 bytes
- ✅ Fits comfortably!

**Arduino Mega (8KB RAM):**
- Full RobotLib: ~2KB
- ✅ Plenty of room

**ESP32 (320KB RAM):**
- Use full library: ~5KB
- ✅ No worries!

**Teensy 4.0 (1MB RAM):**
- Everything works: ~10KB
- ✅ Use all features!

---

## Troubleshooting PlatformIO

### Check include paths are working:
```bash
pio run -v | grep "RobotLib"
# Should show: -I lib/RobotLib/include
```

### Clean and rebuild:
```bash
pio run --target clean
pio run
```

### Check directory structure:
```bash
tree -L 4 lib/
# Should show RobotLib/include/*.h
```

---

## VSCode Integration

If using VSCode with PlatformIO extension:

**1. IntelliSense should work automatically** (reads platformio.ini)

**2. If red squiggles appear:**
   - Press `Ctrl+Shift+P`
   - Type "PlatformIO: Rebuild IntelliSense Index"
   - Select your environment

**3. Manual fix in c_cpp_properties.json:**
```json
{
    "configurations": [
        {
            "name": "PlatformIO",
            "includePath": [
                "${workspaceFolder}/lib/RobotLib/include",
                "${workspaceFolder}/include",
                "${workspaceFolder}/src",
                "${workspaceFolder}/.pio/libdeps/**"
            ]
        }
    ]
}
```

---

## Summary

**✅ To use RobotLib in PlatformIO:**

1. **Copy RobotLib to `lib/` folder**
2. **Add to `platformio.ini`:**
   ```ini
   build_flags =
       -std=c++11
       -I lib/RobotLib/include
       -D UNITS_EMBEDDED=1    ; For Arduino Uno/Nano
   ```
3. **In your code:**
   ```cpp
   #include <RobotLib.h>
   ```
4. **Build:** `pio run`

That's it! 🚀
