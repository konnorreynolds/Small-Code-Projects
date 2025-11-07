# Platform-Specific Examples

Complete, ready-to-run robot examples organized by platform.

## Philosophy

These examples use **clean, simple code** with RobotLib's output utilities instead of cluttered `std::cout` calls. This makes the code:
- ✅ Easier to read and understand
- ✅ Works on both Arduino and desktop C++
- ✅ Simple to modify for your robot
- ✅ Platform-agnostic where possible

## Quick Start

Choose your platform:

### 🔵 Arduino IDE → [arduino_ide/](arduino_ide/)
Perfect for beginners! Open `.ino` files directly in Arduino IDE.

**Examples**:
- Line follower robot
- Differential drive with odometry
- Obstacle avoidance

### ⚡ PlatformIO → [platformio/](platformio/)
Professional development with better tools and multi-platform support.

**Examples**:
- ESP32 WiFi robot
- STM32 with FreeRTOS
- Teensy high-speed robot

### Platform-Specific:
- 📶 **ESP32** → [esp32/](esp32/) - WiFi, BLE, dual-core
- 🔧 **STM32** → [stm32/](stm32/) - Professional, real-time
- 🚀 **Teensy** → [teensy/](teensy/) - Ultimate performance

## Platform Comparison

| Platform | Speed | Price | Best For | Difficulty |
|----------|-------|-------|----------|------------|
| **Arduino Uno** | 16 MHz | $5 | Learning, simple robots | ⭐ Easy |
| **Arduino Mega** | 16 MHz | $10 | Many I/O pins | ⭐ Easy |
| **ESP32** | 240 MHz | $5 | WiFi/BLE, IoT robots | ⭐⭐ Medium |
| **STM32** | 72-168 MHz | $2-20 | Real-time, professional | ⭐⭐⭐ Advanced |
| **Teensy 4.0** | 600 MHz | $20 | High performance | ⭐⭐ Medium |

## Features by Platform

### Arduino (Uno, Nano, Mega)
✅ Easiest to learn
✅ Most tutorials available
✅ Works with Arduino IDE
✅ Large community
❌ Limited speed
❌ Limited memory

**Use when**: Learning, simple robots, lots of support needed

---

### ESP32
✅ Built-in WiFi & Bluetooth
✅ Fast (240 MHz dual-core)
✅ Cheap ($5)
✅ Many GPIO pins
✅ Deep sleep for battery
❌ 3.3V logic only
❌ Some pin restrictions

**Use when**: WiFi/BLE control, IoT robots, remote monitoring

---

### STM32
✅ Very cheap ($2+)
✅ Professional ARM architecture
✅ Fast (72-168 MHz)
✅ All pins interrupt-capable
✅ FreeRTOS support
❌ Needs programmer (ST-Link)
❌ Setup more complex

**Use when**: Budget robots, professional development, real-time control

---

### Teensy
✅ **Extremely fast** (600 MHz!)
✅ High-speed PWM (150 kHz)
✅ Many hardware timers
✅ Excellent documentation
✅ Native USB
❌ More expensive ($20)
❌ 3.3V logic only

**Use when**: Competition robots, fast control loops, best performance

## Code Style: Old vs New

### ❌ Old Style (cluttered)

```cpp
std::cout << "Distance: " << distance.toMeters() << " m" << std::endl;
std::cout << "Velocity: " << velocity.toMetersPerSecond() << " m/s" << std::endl;
std::cout << "Position: (" << x << ", " << y << ")" << std::endl;
```

### ✅ New Style (clean!)

```cpp
using namespace robotlib::output;

logUnit("Distance", distance);
logUnit("Velocity", velocity);
println("Position: (", x, ", ", y, ")");
```

Much cleaner and easier to read!

## Getting Started

### For Beginners
1. Start with **Arduino IDE examples** → [arduino_ide/](arduino_ide/)
2. Try the line follower or obstacle avoider
3. Learn the basics of sensors and motors
4. Move to more complex examples

### For Intermediate
1. Try **PlatformIO** → [platformio/](platformio/)
2. Experiment with ESP32 WiFi robot
3. Learn about RTOS with STM32 example
4. Add more sensors and features

### For Advanced
1. Use **Teensy** for high-performance → [teensy/](teensy/)
2. Implement fast control loops (1kHz+)
3. Add advanced algorithms (MPC, SLAM)
4. Build competition robots

## Common Hardware

Most examples use similar hardware:

**Motors**:
- 2x DC motors (6-12V)
- L298N or TB6612 motor driver
- Battery (7.4V LiPo recommended)

**Sensors**:
- HC-SR04 ultrasonic (distance)
- Encoders (position/speed)
- IR sensors (line following)
- MPU6050 (IMU, optional)

**Microcontroller**:
- Choose based on your needs (see comparison above)

## Tips for All Platforms

✅ **Start simple** - Get one thing working, then add more
✅ **Use output utilities** - `robotlib::output::println()` is cleaner
✅ **Test incrementally** - Test each part before integration
✅ **Read datasheets** - Know your hardware specs
✅ **Power motors separately** - Motors need their own battery
✅ **Add safety** - Emergency stop, timeout protection
✅ **Comment your code** - Future you will thank you

## Next Steps

1. **Choose your platform** based on the comparison table
2. **Get the hardware** listed in the example README
3. **Follow the setup guide** for your chosen platform
4. **Upload an example** and verify it works
5. **Modify and experiment** to learn
6. **Build your own robot** combining what you learned!

## Support

- **Issues**: https://github.com/yourusername/RobotLib/issues
- **Docs**: [../../docs/](../../docs/)
- **Examples**: Browse folders above

Good luck building your robot! 🤖
