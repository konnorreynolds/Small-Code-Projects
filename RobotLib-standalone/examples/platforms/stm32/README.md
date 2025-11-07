# STM32 Platform-Specific Examples

Examples for STM32 microcontrollers with RobotLib.

## STM32 Features

- **Low cost** - Blue Pill (~$2), Black Pill (~$5)
- **ARM Cortex-M** - Professional architecture
- **Fast** - 72-168 MHz depending on model
- **Many peripherals** - Timers, ADCs, UARTs, I2C, SPI
- **Real-time** - Excellent for control systems
- **FreeRTOS support** - Professional multitasking

## Common Boards

### STM32F103C8 (Blue Pill)
- **CPU**: 72 MHz ARM Cortex-M3
- **Flash**: 64KB (sometimes 128KB)
- **RAM**: 20KB
- **Price**: ~$2
- **Best for**: Budget robots, learning

### STM32F411CE (Black Pill)
- **CPU**: 100 MHz ARM Cortex-M4F
- **Flash**: 512KB
- **RAM**: 128KB
- **Price**: ~$5
- **Best for**: Advanced robots, FPU math

### STM32F407VG (Discovery)
- **CPU**: 168 MHz ARM Cortex-M4F
- **Flash**: 1MB
- **RAM**: 192KB
- **Price**: ~$20
- **Best for**: Professional development

## Programming Methods

### ST-Link V2 (Recommended)
```ini
upload_protocol = stlink
```
- Cheap (~$3)
- Fast uploads
- Debugging support

### USB DFU (Blue Pill)
```ini
upload_protocol = dfu
```
- No programmer needed
- Requires boot0 jumper

### Serial (FTDI)
```ini
upload_protocol = serial
```
- Requires USB-Serial adapter
- Slower than ST-Link

## Setup

### PlatformIO (Recommended)

See `examples/platforms/platformio/stm32_robot_rtos/` for complete example.

platformio.ini:
```ini
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = stlink
```

### Arduino IDE

1. Add STM32 board manager URL:
   ```
   https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
   ```
2. Install "STM32 MCU based boards"
3. Select your board

## Pin Mapping

### Blue Pill (STM32F103C8)

```cpp
// PWM-capable pins
PA0, PA1, PA2, PA3, PA6, PA7
PB0, PB1, PB6, PB7, PB8, PB9

// Interrupt-capable (all pins!)
// Any pin can be used for encoders

// I2C
PB6 (SCL), PB7 (SDA)  // I2C1
PB10 (SCL), PB11 (SDA) // I2C2

// SPI
PA5 (SCK), PA6 (MISO), PA7 (MOSI)

// UART
PA9 (TX), PA10 (RX)  // USART1
PA2 (TX), PA3 (RX)   // USART2
```

## Tips for STM32

✅ **Use ST-Link** - Much better than serial upload
✅ **Check voltage** - Most STM32 are 3.3V, not 5V tolerant!
✅ **Use FreeRTOS** - Professional task management
✅ **Hardware timers** - Better than software PWM
✅ **DMA** - Use DMA for ADC, SPI, UART for performance
✅ **Flash size** - Blue Pill often has 128KB, not 64KB

## Common Gotchas

❌ **5V inputs** - STM32 pins are 3.3V max! Use level shifters
❌ **Boot0 jumper** - May need to change for uploading
❌ **Fake chips** - Many clones, check flash size
❌ **USB issues** - Built-in USB can be unreliable on Blue Pill

## Examples

See `examples/platforms/platformio/stm32_robot_rtos/` for:
- FreeRTOS multitasking
- Professional architecture
- Real-time control

## Next Steps

1. Get ST-Link V2 programmer (~$3)
2. Try the RTOS example
3. Learn FreeRTOS basics
4. Build advanced control systems!

## Resources

- [STM32duino](https://github.com/stm32duino)
- [Blue Pill Guide](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html)
- [FreeRTOS](https://www.freertos.org/)
