# PlatformIO Examples

Complete robot projects for PlatformIO with advanced features and multiple platform support.

## Why PlatformIO?

- **Better dependency management** - Automatic library installation
- **Multiple platforms** - ESP32, STM32, Teensy, Arduino, and more
- **Built-in debugging** - Professional development tools
- **Faster builds** - Incremental compilation
- **Unit testing** - Built-in test framework
- **CI/CD ready** - Easy integration with GitHub Actions

## Installation

### Step 1: Install PlatformIO

**VSCode (Recommended)**:
1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Open Extensions (Ctrl+Shift+X)
3. Search for "PlatformIO IDE"
4. Click Install

**Command Line**:
```bash
pip install platformio
```

### Step 2: Install RobotLib

Each project expects RobotLib to be in `lib/RobotLib/`. You have two options:

**Option A: Symlink** (Recommended for development)
```bash
cd <project-folder>
mkdir -p lib
ln -s /path/to/RobotLib lib/RobotLib
```

**Option B: Copy**
```bash
cd <project-folder>
mkdir -p lib
cp -r /path/to/RobotLib lib/
```

### Step 3: Build and Upload

**VSCode**:
1. Open project folder
2. Click PlatformIO icon (alien head)
3. Click "Build" then "Upload"

**Command Line**:
```bash
cd <project-folder>
pio run -t upload
```

## Examples

### esp32_wifi_robot/
**Platform**: ESP32
**Difficulty**: Intermediate
**Features**:
- WiFi web server control
- Mobile-responsive UI
- Real-time telemetry
- Remote robot control from phone/browser

**Hardware**:
- ESP32 DevKit
- L298N Motor Driver
- 2x DC Motors
- Battery (7.4V)

**Usage**:
1. Edit `src/main.cpp` and set your WiFi SSID/password
2. Upload to ESP32
3. Open Serial Monitor to see IP address
4. Open browser and navigate to the IP address
5. Control robot from web interface!

---

### stm32_robot_rtos/
**Platform**: STM32F103 (Blue Pill)
**Difficulty**: Advanced
**Features**:
- FreeRTOS multitasking
- Concurrent tasks (motor, sensor, telemetry)
- Mutex-protected shared resources
- Professional architecture

**Hardware**:
- STM32F103C8 Blue Pill
- ST-Link V2 programmer
- L298N Motor Driver
- 2x DC Motors with encoders
- HC-SR04 Ultrasonic

**Requires**:
- ST-Link driver installed
- `upload_protocol = stlink` in platformio.ini

---

### teensy_fast_robot/
**Platform**: Teensy 4.0
**Difficulty**: Advanced
**Features**:
- 1kHz control loop (very fast!)
- High-resolution encoder tracking
- Advanced PID control
- Real-time velocity estimation
- Serial command interface

**Hardware**:
- Teensy 4.0 (600 MHz!)
- Motor driver
- 2x DC Motors with high-res encoders
- Optional: MPU6050 IMU

**Why Teensy?**:
- Extremely fast (600 MHz ARM Cortex-M7)
- High-speed PWM (up to 150 kHz)
- Many hardware timers
- Perfect for high-performance robotics

---

## Project Structure

Each PlatformIO project follows this structure:

```
project_name/
├── platformio.ini       # Project configuration
├── src/
│   └── main.cpp        # Main source code
├── lib/
│   └── RobotLib/       # RobotLib (symlink or copy)
├── include/            # Additional headers (optional)
└── test/              # Unit tests (optional)
```

## platformio.ini Explained

```ini
[env:esp32dev]
platform = espressif32        # Platform (esp32, ststm32, teensy, etc.)
board = esp32dev             # Specific board
framework = arduino          # Framework (arduino, mbed, espidf, etc.)

build_flags =
    -std=c++11               # C++11 standard
    -I lib/RobotLib/include  # Include RobotLib headers
    -D UNITS_EMBEDDED=1      # Enable embedded mode
    -D ROBOTLIB_DEBUG=1      # Enable debug output

build_unflags = -std=gnu++11  # Remove GNU extensions

monitor_speed = 115200        # Serial monitor baud rate

lib_deps =                    # External libraries
    # Add libraries here
```

## Common Tasks

### Build Project
```bash
pio run
```

### Upload to Board
```bash
pio run -t upload
```

### Open Serial Monitor
```bash
pio device monitor
```

### Clean Build
```bash
pio run -t clean
```

### Run Tests
```bash
pio test
```

### Update Libraries
```bash
pio lib update
```

## Troubleshooting

### "lib/RobotLib/include: No such file or directory"

**Solution**: RobotLib is not installed in the lib/ folder. Use symlink or copy as shown above.

### Upload fails: "No device found"

**ESP32**: Install CP2102 or CH340 USB driver
**STM32**: Install ST-Link driver and connect ST-Link programmer
**Teensy**: Install Teensy Loader

### Wrong board selected

Check `board =` in `platformio.ini`. List available boards:
```bash
pio boards
```

### Serial port permission denied (Linux)

Add your user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and back in.

## Adding Custom Code

### Modify main.cpp
Edit `src/main.cpp` to customize robot behavior.

### Add New Files
```cpp
// src/motors.cpp
#include "motors.h"

void setupMotors() {
    // Your code
}
```

```cpp
// include/motors.h
#ifndef MOTORS_H
#define MOTORS_H

void setupMotors();

#endif
```

### Add External Libraries
Edit `platformio.ini`:
```ini
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.4
    Wire
```

## Best Practices

✅ **Use version control** - Git is your friend
✅ **Test incrementally** - Build often, test often
✅ **Use Serial output** - Debug with print statements
✅ **Separate concerns** - Split code into functions/files
✅ **Handle errors** - Check return values, add timeouts
✅ **Comment your code** - Future you will thank you
✅ **Use units** - RobotLib units prevent errors!

## Advanced Topics

### Custom Environments

Add multiple build configurations:

```ini
[env:debug]
build_flags = ${env.build_flags} -DDEBUG_MODE

[env:release]
build_flags = ${env.build_flags} -O3
```

Build specific environment:
```bash
pio run -e debug
```

### OTA Updates (ESP32)

Add to `platformio.ini`:
```ini
upload_protocol = espota
upload_port = 192.168.1.100
```

### Continuous Integration

Create `.github/workflows/build.yml`:
```yaml
name: PlatformIO CI
on: [push]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    - uses: actions/setup-python@v2
    - run: pip install platformio
    - run: pio run
```

## Next Steps

1. Try the examples as-is to verify your setup
2. Modify parameters (speeds, pins, etc.)
3. Add new sensors or actuators
4. Combine features from multiple examples
5. Build your own robot project!

## Resources

- [PlatformIO Docs](https://docs.platformio.org/)
- [RobotLib Documentation](../../docs/)
- [PlatformIO Community](https://community.platformio.org/)

## Support

- GitHub Issues: https://github.com/yourusername/RobotLib/issues
- PlatformIO Forums: https://community.platformio.org/
