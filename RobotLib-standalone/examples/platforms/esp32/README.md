# ESP32 Platform-Specific Examples

Examples showcasing ESP32 unique features with RobotLib.

## ESP32 Features

- **Dual-core** - Run tasks on separate cores
- **WiFi** - Built-in wireless connectivity
- **Bluetooth/BLE** - Wireless control
- **Touch sensors** - Capacitive touch pins
- **DAC** - True analog output
- **Multiple UARTs** - Up to 3 serial ports
- **Fast ADC** - Better analog readings
- **RTC** - Real-time clock with deep sleep

## Setup

All examples are for **Arduino framework on ESP32**. Use either:
- Arduino IDE with ESP32 board support
- PlatformIO (recommended)

### Arduino IDE Setup

1. Add ESP32 board manager URL:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. Install "ESP32 by Espressif Systems"
3. Select board: "ESP32 Dev Module"

### PlatformIO Setup

See `examples/platforms/platformio/` for complete PlatformIO examples.

## Examples Coming Soon

- **ESP32 BLE Robot** - Control robot via Bluetooth
- **ESP32 Dual-Core Robot** - Motor control on Core 0, sensing on Core 1
- **ESP32 Web Dashboard** - Advanced web interface with charts
- **ESP32 Deep Sleep** - Battery-efficient autonomous robot
- **ESP32 Camera Robot** - With vision processing

## Tips for ESP32

✅ **Use both cores** - `xTaskCreatePinnedToCore()` for dual-core
✅ **WiFi power management** - Disable WiFi when not needed
✅ **LEDC for PWM** - Use `ledcSetup()` and `ledcWrite()`
✅ **Deep sleep** - `esp_deep_sleep_start()` for battery saving
✅ **Brown-out detector** - May need to disable for motor loads
✅ **Flash size** - 4MB is standard, check your board

## Common Pin Configuration

```cpp
// Motors (any GPIO except input-only)
const int MOTOR_LEFT_PWM = 32;   // Channel 0
const int MOTOR_RIGHT_PWM = 33;  // Channel 1

// Encoders (interrupt-capable pins)
const int ENCODER_LEFT = 34;     // Input only, no pullup
const int ENCODER_RIGHT = 35;

// I2C (standard pins)
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Touch sensors
const int TOUCH_START = T0;  // GPIO4
const int TOUCH_STOP = T3;   // GPIO15
```

## Next Steps

Check `examples/platforms/platformio/esp32_wifi_robot/` for a complete working example.
