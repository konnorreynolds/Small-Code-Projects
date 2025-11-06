# Arduino IDE Examples

Complete, ready-to-run robot projects for Arduino IDE.

## Installation

### Step 1: Install RobotLib

1. Download or clone the RobotLib folder
2. Copy `RobotLib` folder to your Arduino libraries directory:
   - **Windows**: `C:\Users\<username>\Documents\Arduino\libraries\`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`

### Step 2: Open Arduino IDE

1. Restart Arduino IDE (if it was open)
2. Go to `File → Examples → RobotLib → platforms → arduino_ide`
3. Select an example to open

### Step 3: Upload

1. Select your board: `Tools → Board`
2. Select your port: `Tools → Port`
3. Click Upload ⬆️

## Examples

### 01_line_follower_robot.ino
**Difficulty**: Beginner
**Description**: PID-based line following robot using IR sensors

**Hardware**:
- Arduino Uno/Nano/Mega
- L298N Motor Driver
- 2x DC Motors
- 5x IR Line Sensors
- Battery (7.4V)

**Features**:
- PID control for smooth line tracking
- Clean, readable code using RobotLib output utilities
- Weighted sensor algorithm for precise positioning

---

### 02_differential_drive_robot.ino
**Difficulty**: Intermediate
**Description**: Differential drive robot with wheel encoders and odometry

**Hardware**:
- Arduino Uno/Nano/Mega
- L298N Motor Driver
- 2x DC Motors with encoders
- Battery (7.4V)

**Features**:
- Real-time position tracking (X, Y, θ)
- Encoder-based odometry
- Demo movement patterns (square, circle)
- Type-safe units from RobotLib

---

### 03_obstacle_avoidance_robot.ino
**Difficulty**: Beginner
**Description**: Autonomous navigation using ultrasonic sensor

**Hardware**:
- Arduino Uno/Nano
- L298N Motor Driver
- 2x DC Motors
- HC-SR04 Ultrasonic Sensor
- Battery (7.4V)

**Features**:
- State machine behavior
- Dynamic speed control
- Safe distance monitoring
- Random turn direction for variety

---

## Wiring Diagrams

### Standard Motor Driver (L298N) Pinout

```
Arduino    L298N
-------    -----
Pin 3   →  ENA (Left motor speed)
Pin 5   →  IN1 (Left motor forward)
Pin 6   →  IN2 (Left motor reverse)
Pin 9   →  IN3 (Right motor forward)
Pin 10  →  IN4 (Right motor reverse)
Pin 11  →  ENB (Right motor speed)
GND     →  GND
```

### Ultrasonic Sensor (HC-SR04)

```
Arduino    HC-SR04
-------    -------
Pin 12  →  TRIG
Pin 13  →  ECHO
5V      →  VCC
GND     →  GND
```

### IR Line Sensors

```
Arduino    Sensor Position
-------    ---------------
Pin 2   →  Far Left
Pin 4   →  Left
Pin 7   →  Center
Pin 8   →  Right
Pin 12  →  Far Right
```

## Troubleshooting

### "RobotLib.h: No such file or directory"

**Solution**: Make sure RobotLib is installed in the correct libraries folder. Restart Arduino IDE.

### Motors not moving

1. Check battery voltage (should be 7-12V)
2. Verify motor driver connections
3. Check motor driver enable jumpers (ENA/ENB)
4. Test with simple digitalWrite/analogWrite first

### Serial output not showing

1. Open Serial Monitor: `Tools → Serial Monitor`
2. Set baud rate to **115200**
3. Make sure `begin(115200)` is called in setup()

### Encoders not counting

1. Use interrupt-capable pins (2, 3 on Uno)
2. Check encoder connections (A and B channels)
3. Test with simple interrupt example first

### Robot drifts to one side

1. Motors may have different speeds - adjust in code
2. Check wheel diameters (should be equal)
3. Battery voltage may be low
4. One motor may be weaker - add calibration offset

## Tips for Success

✅ **Start simple**: Begin with basic motor control before adding sensors
✅ **Use Serial Monitor**: Print debug info to understand what's happening
✅ **Power separately**: Use separate battery for motors (not USB power)
✅ **Add delays**: Small delays (10-50ms) help with sensor stability
✅ **Calibrate sensors**: Test sensors individually before integration
✅ **Check voltages**: Motors need 6-12V, Arduino needs 5V

## Next Steps

After mastering these examples:

1. Combine features (line following + obstacle avoidance)
2. Add more sensors (IMU, compass, GPS)
3. Implement advanced algorithms (SLAM, path planning)
4. Build a competition robot (sumo, maze solver)
5. Check out PlatformIO examples for more advanced features

## Support

- Issues: https://github.com/yourusername/RobotLib/issues
- Documentation: See `RobotLib/docs/`
- More examples: `RobotLib/examples/`
