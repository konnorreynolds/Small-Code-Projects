# Teensy Platform-Specific Examples

Examples for Teensy microcontrollers with RobotLib - the ultimate performance platform!

## Teensy Features

- **Extremely fast** - Up to 600 MHz!
- **Floating point hardware** - Fast math
- **High-speed PWM** - Up to 150 kHz
- **Many timers** - Professional control loops
- **USB native** - Fast, reliable uploads
- **Real-time** - Best-in-class control performance
- **DMA** - Efficient data transfers

## Teensy Boards

### Teensy 4.0
- **CPU**: 600 MHz ARM Cortex-M7
- **Flash**: 2MB
- **RAM**: 1MB
- **Price**: $20
- **Best for**: High-performance robots, fast control loops

### Teensy 4.1
- **CPU**: 600 MHz ARM Cortex-M7
- **Flash**: 8MB
- **RAM**: 1MB + SD card slot
- **Ethernet**: Built-in
- **Price**: $27
- **Best for**: Advanced robotics, data logging

### Teensy 3.6
- **CPU**: 180 MHz ARM Cortex-M4F
- **Flash**: 1MB
- **RAM**: 256KB
- **Price**: $30
- **Best for**: 5V tolerant I/O needed

## Why Teensy for Robotics?

✅ **Speed** - 600 MHz = lightning fast control loops
✅ **Reliability** - Professional quality, well-tested
✅ **PWM** - Ultra-high frequency for smooth motor control
✅ **Timers** - IntervalTimer library for precise timing
✅ **Float math** - Hardware FPU, no penalty for floating point
✅ **USB** - Native USB, very fast serial communication
✅ **Documentation** - Excellent docs and community

## Setup

### Arduino IDE with Teensyduino

1. Download [Arduino IDE](https://www.arduino.cc/en/software)
2. Download [Teensyduino](https://www.pjrc.com/teensy/td_download.html)
3. Install Teensyduino (adds Teensy support to Arduino IDE)
4. Select `Tools → Board → Teensy 4.0`

### PlatformIO

See `examples/platforms/platformio/teensy_fast_robot/` for complete example.

platformio.ini:
```ini
[env:teensy40]
platform = teensy
board = teensy40
framework = arduino
```

## Performance Comparison

| Board | CPU Speed | Control Loop |
|-------|-----------|--------------|
| Arduino Uno | 16 MHz | ~100 Hz |
| ESP32 | 240 MHz | ~500 Hz |
| STM32F103 | 72 MHz | ~500 Hz |
| **Teensy 4.0** | **600 MHz** | **>1000 Hz** ⚡ |

Teensy can run a full PID control loop at **1 kHz** or faster!

## Advanced Features

### IntervalTimer - Precise Control Loops

```cpp
IntervalTimer controlTimer;

void controlLoop() {
    // Your fast control code
    // Runs exactly every 1ms
}

void setup() {
    controlTimer.begin(controlLoop, 1000);  // 1000μs = 1ms
}
```

### High-Speed PWM

```cpp
void setup() {
    analogWriteFrequency(pin, 20000);  // 20 kHz PWM!
    analogWriteResolution(12);         // 0-4095 range
}
```

### Digital I/O Speed

```cpp
// Fast digital I/O
digitalWriteFast(LED, HIGH);  // Single instruction!
digitalReadFast(SENSOR);
```

### DMA for Efficiency

```cpp
// DMA-based ADC for high-speed sensor reading
// See Teensy documentation for details
```

## Pin Capabilities

### Teensy 4.0

**All digital pins**:
- Interrupt-capable ✅
- 3.3V logic levels

**PWM pins**:
- 0-15, 18-19, 22-25, 28-29, 33, 36-37
- Configurable frequency

**Analog input**:
- A0-A9 (pins 14-23)
- 10-bit or 12-bit resolution

**I2C**:
- Pins 18 (SDA), 19 (SCL)
- Multiple I2C buses available

**SPI**:
- Pins 11 (MOSI), 12 (MISO), 13 (SCK)

## Example: 1kHz Control Loop

```cpp
#include <RobotLib.h>
using namespace units;
using namespace robotics;

PIDController pid(2.0, 0.5, 0.1);
IntervalTimer controlTimer;

void controlLoop() {
    // This runs every 1ms (1000 Hz!)
    static double lastError = 0;

    double target = 1.0;
    double current = readSensor();
    double output = pid.calculate(target, current, 0.001);

    setMotorPWM(output);
}

void setup() {
    setupMotors();
    controlTimer.begin(controlLoop, 1000);  // 1000μs period
}

void loop() {
    // Telemetry, user interface, etc.
}
```

## Performance Tips

✅ **Use IntervalTimer** - Don't use delay() for control loops
✅ **Use digitalWriteFast** - Faster than digitalWrite
✅ **Optimize compiler** - `-O2` or `-O3` optimization
✅ **Use FPU** - Floating point is fast on Teensy 4.0
✅ **Minimize Serial** - Serial.print can slow things down
✅ **DMA when possible** - For ADC, SPI, serial

## Common Use Cases

### Competition Robots
- Fast response time
- Precise control
- Reliable performance

### Balancing Robots
- Need very fast control loop
- Teensy is perfect!

### High-Speed Data Logging
- Fast ADC sampling
- SD card on Teensy 4.1

### Advanced Control
- MPC, LQR algorithms
- Matrix math with FPU

## Gotchas

❌ **3.3V only** - Not 5V tolerant (except Teensy 3.6)
❌ **Current limits** - 100mA per pin, use driver for motors
❌ **Price** - More expensive than Arduino/ESP32
❌ **Availability** - Sometimes out of stock

But the performance is worth it! 🚀

## Example Projects

See `examples/platforms/platformio/teensy_fast_robot/` for:
- 1kHz control loop
- High-resolution encoders
- Advanced PID
- Real-time velocity estimation
- Serial command interface

## Next Steps

1. Get a Teensy 4.0 ($20)
2. Install Teensyduino
3. Try the fast robot example
4. Build the fastest robot in your club! 🏎️

## Resources

- [PJRC Teensy](https://www.pjrc.com/teensy/)
- [Teensy Forum](https://forum.pjrc.com/)
- [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)
- [RobotLib Examples](../../)
