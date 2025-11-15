# YAMS Swerve Drive with Obstacle Avoidance

This project demonstrates integrating obstacle avoidance with YAMS (Yet Another Mechanism System) swerve drive for FRC robots.

## Features

- Real YAMS SwerveDrive integration with 4 modules
- Artificial Potential Field (APF) obstacle avoidance
- Opponent robot avoidance with configurable behavior
- Field boundary obstacles (2025 Reefscape dimensions)
- Simulation support with WPILib simulation GUI

## Hardware Configuration

### Motors & Encoders
- **Front Left**: Drive Motor CAN 1, Azimuth Motor CAN 2, CANcoder CAN 3
- **Front Right**: Drive Motor CAN 4, Azimuth Motor CAN 5, CANcoder CAN 6
- **Back Left**: Drive Motor CAN 7, Azimuth Motor CAN 8, CANcoder CAN 9
- **Back Right**: Drive Motor CAN 10, Azimuth Motor CAN 11, CANcoder CAN 12
- **Gyro**: Pigeon 2.0 on CAN 14

### Gearing
- **Drive**: 24:1 reduction (12:1 × 2:1)
- **Azimuth**: 21:1 reduction
- **Wheel Diameter**: 4 inches

## Build Requirements

1. **WPILib 2025.3.2** installed
2. **Java 17** (included with WPILib)
3. **GradleRIO 2025.3.2**

### Vendordeps
- Phoenix6 (CTRE CANcoder and Pigeon 2.0)
- REVLib (REV Spark Max motor controllers)
- WPILibNewCommands
- YAMS (Yet Another Mechanism System) version 2025.10.29

## Building the Project

In a real WPILib environment with internet access:

```bash
./gradlew build
```

## Running in Simulation

1. Open WPILib VSCode
2. Press `Ctrl+Shift+P` and select "WPILib: Simulate Robot Code"
3. In the simulation GUI, enable the Field2d widget to visualize robot position
4. Connect a virtual Xbox controller or use keyboard controls
5. Press the **A button** to trigger obstacle avoidance navigation

## Obstacle Avoidance System

### Classes

- **`SwerveDriveSubsystem`**: Main swerve subsystem with YAMS integration
- **`ObstacleNavigator`**: APF algorithm for path planning
- **`Obstacle`**: Represents static/dynamic obstacles
- **`Config`**: Navigation parameters (velocity, avoidance radius, etc.)

### Testing the A Button Command

When A button is pressed:
1. Robot navigates from current position to target `(8.0, 4.0)`
2. Avoids opponent robot at `(5.0, 3.0)` with aggressive behavior
3. Respects field boundaries at y=0 and y=8.23 meters
4. Uses APF with 98.2% decisiveness for smooth, committed paths

### Obstacle Configuration

**Opponent Behavior:**
- Position: `(5.0, 3.0)` meters
- Velocity: `(0.2, 0.1)` m/s
- Aggressive avoidance weight: 2.5
- Difficulty level: 1.0 (maximum)

**Navigation Config:**
- Max velocity: 4.5 m/s
- Avoidance radius: 2.5 meters
- Avoidance strength: 2.0
- Goal attraction: 12.0

## Simulation Expected Behavior

1. Robot starts at `(2.0, 2.0)` facing 0°
2. Presses A button triggers navigation to `(8.0, 4.0)`
3. Robot should smoothly avoid opponent robot at `(5.0, 3.0)`
4. Path should be decisive with minimal wavering (<10° total)
5. `System.out.println` messages show APF calculations
6. Field2d shows real-time robot pose and trajectory

## Customization

### Modify Obstacle Avoidance Behavior

Edit `SwerveDriveSubsystem.driveToWithOpponentAvoidance()`:

```java
// Change target position
Pose2d target = new Pose2d(12.0, 6.0, new Rotation2d());

// Modify opponent behavior
Obstacle opponent = Obstacle.robot(
    new Pose2d(6.0, 4.0, new Rotation2d()),  // Position
    new Translation2d(0.5, 0.2),             // Velocity
    true                                      // Dynamic
).aggressive().difficulty(0.8);              // 80% difficulty

// Adjust navigation parameters
Config config = Config.forOpponent();
config.maxVelocity = 3.0;
config.avoidanceRadius = 3.0;
```

### Add More Obstacles

```java
private List<Obstacle> createObstacles(Obstacle opponent) {
    List<Obstacle> obstacleList = new ArrayList<>();
    obstacleList.add(opponent);

    // Add circular obstacle
    obstacleList.add(Obstacle.circle(
        new Translation2d(6.0, 5.0),  // Center
        1.5                            // Radius
    ));

    // Add wall obstacles
    obstacleList.add(Obstacle.wall(
        new Translation2d(0, 0),
        new Translation2d(16.54, 0)
    ));

    return obstacleList;
}
```

## File Structure

```
test-project/
├── build.gradle              # GradleRIO configuration with YAMS source
├── settings.gradle           # Project settings
├── vendordeps/               # Third-party dependencies
│   ├── Phoenix6.json
│   ├── REVLib.json
│   └── WPILibNewCommands.json
└── src/main/java/frc/robot/
    ├── Main.java            # Entry point
    ├── Robot.java           # TimedRobot
    ├── RobotContainer.java  # Command bindings (A button)
    └── subsystems/
        └── SwerveDriveSubsystem.java  # YAMS + Obstacle avoidance
```

## Troubleshooting

### Build fails with "GradleRIO not found"
Ensure WPILib 2025.3.2 is installed. The offline build environment doesn't have access to Gradle distribution servers.

### Simulation doesn't start
1. Check that all vendordeps are correctly installed
2. Verify YAMS source path in `build.gradle` points to `../yams-repo/yams/java`
3. Ensure Java 17 is configured

### Robot doesn't move when A is pressed
1. Check `RobotContainer.java` line 18-20 for A button binding
2. Verify `SwerveDriveSubsystem.driveToWithOpponentAvoidance()` is called
3. Check console output for obstacle avoidance debug messages

## Next Steps

- Add more complex obstacle scenarios
- Implement real-time obstacle detection with vision
- Tune APF parameters for specific game strategies
- Add autonomous routines with obstacle avoidance
- Test on real hardware with actual swerve modules

## References

- [YAMS Repository](https://github.com/Yet-Another-Software-Suite/YAMS)
- [WPILib Documentation](https://docs.wpilib.org/)
- [Obstacle Avoidance Implementation](../ObstacleAvoidance.java)
