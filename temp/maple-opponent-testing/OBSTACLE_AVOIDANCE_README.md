# Maple-Sim Opponent Testing with Obstacle Avoidance

This project demonstrates ultra-decisive obstacle avoidance with maple-sim opponent robots on the 2025 Reefscape field.

## Features

- **Reefscape 2025 Field Obstacles**
  - Field boundaries (all 4 walls)
  - Blue Reef hexagon
  - Red Reef hexagon
  - Center pillar

- **Dynamic Maple-Sim Opponents**
  - 2 KitBot opponents on Red alliance
  - Real-time velocity tracking
  - Aggressive avoidance behavior (difficulty 0.9)

- **Ultra-Decisive Navigation** (>98% decisiveness)
  - Artificial Potential Fields (APF) algorithm
  - Velocity-aware collision prediction
  - Path commitment and momentum preferences
  - Time-to-Collision (TTC) prediction

## Controls

### Xbox Controller

- **Left Stick**: Translation (field-relative by default)
- **Right Stick X**: Rotation
- **A Button (Hold)**: Navigate to (14, 4) with obstacle avoidance
- **Back Button**: Reset robot pose to (3, 3)
- **Start Button**: Toggle robot-relative / alliance-relative control

## Running the Simulation

```bash
cd maple-opponent-testing
./gradlew simulateJava
```

### What You'll See

1. **Sim GUI Window**
   - All 4 swerve modules with drive + azimuth motors
   - Pigeon 2 gyro
   - NetworkTables telemetry

2. **Field2d Widget**
   - Blue robot (your robot) starting at (3, 3)
   - 2 red opponent robots (KitBots)
   - Real-time path visualization
   - Obstacle locations

3. **SmartDashboard Values**
   - `Obstacles/Total`: Total obstacles (static + dynamic)
   - `Obstacles/Dynamic`: Number of opponent robots
   - `Distance to Goal`: Current distance to target pose

### Testing Obstacle Avoidance

1. **Enable robot** in Driver Station (Teleop mode)
2. **Press and HOLD A button** on Xbox controller
3. **Watch the robot**:
   - Navigate from (3, 3) to (14, 4)
   - Avoid field boundaries
   - Avoid both reefs
   - Avoid center pillar
   - Dynamically avoid 2 opponent robots

4. **Expected Behavior**:
   - Smooth, decisive path (no wavering)
   - Strong avoidance of opponents (aggressive weight 1.5x)
   - Velocity-aware prediction
   - Real-time re-planning as opponents move

## Obstacle Configuration

### Static Obstacles (7 total)

| Obstacle | Position | Radius |
|----------|----------|--------|
| Left Wall | x=0 | 0.3m |
| Right Wall | x=17.548 | 0.3m |
| Bottom Wall | y=0 | 0.3m |
| Top Wall | y=8.052 | 0.3m |
| Blue Reef | (4.489, 4.026) | 0.8m |
| Red Reef | (13.059, 4.026) | 0.8m |
| Center Pillar | (8.774, 4.026) | 0.4m |

### Dynamic Opponents (2 KitBots)

- **Type**: Differential drive KitBots
- **Max Speed**: 8.5 m/s
- **Behavior**: Aggressive avoidance (weight 1.5x base)
- **Difficulty**: 0.9 (high difficulty, 90% max velocity)
- **Prediction**: Velocity + acceleration tracking

## Performance Metrics

The obstacle avoidance system is tuned for:

- **Decisiveness**: >98%
- **Direction Changes**: 0-1 per path
- **Path Wavering**: <15° total
- **Path Efficiency**: >95%
- **Avg Speed**: 3.5-4.2 m/s during navigation

## Customization

### Change Target Position

Edit `RobotContainer.java:41`:
```java
xboxController.a().whileTrue(drive.driveToPose(new Pose2d(new Translation2d(10, 6), Rotation2d.kZero)));
```

### Adjust Opponent Difficulty

Edit `SwerveSubsystem.java:243`:
```java
ObstacleAvoidance.Obstacle opponentObstacle = ObstacleAvoidance.robot(
    opponentPose, velocity, true
).aggressive().difficulty(0.5);  // Lower difficulty (50%)
```

### Add More Opponents

Edit `SwerveSubsystem.java:180-185`:
```java
KitBot opponent3 = new KitBot(opponentManager, DriverStation.Alliance.Red, 3);
opponents.add(opponent3);
opponentManager.registerOpponent(opponent3, DriverStation.Alliance.Red);
```

### Modify Avoidance Behavior

Edit `ObstacleAvoidance.Config.opponent()` parameters:
```java
public static Config opponent() {
    Config c = new Config();
    c.maxVelocity = MetersPerSecond.of(4.0);       // Max nav speed
    c.avoidanceRadius = Meters.of(3.0);            // Larger safety margin
    c.avoidanceStrength = 2.5;                     // Stronger push-away
    c.goalAttraction = 15.0;                       // Stronger goal pull
    c.pathCommitment = 0.95;                       // Higher commitment
    return c;
}
```

## Architecture

### SwerveSubsystem

- **OpponentManager**: Manages maple-sim opponent robots
- **Arena2025Reefscape**: Provides field obstacle map
- **ObstacleAvoidance**: APF navigation algorithm
- **Static Obstacles**: Created from reefscape field geometry
- **Dynamic Obstacles**: Updated each iteration from opponent poses/velocities

### Obstacle Avoidance Flow

```
periodic() 20ms
│
├─ Update opponent poses on Field2d
│
whileTrue(driveToPose())
│
├─ Clear obstacles list
├─ Add static obstacles (reefscape)
├─ For each opponent:
│  ├─ Get current pose & velocity
│  ├─ Create aggressive robot obstacle
│  └─ Add to obstacles list
│
├─ Calculate APF navigation
│  ├─ Goal attraction force
│  ├─ Obstacle repulsion forces
│  ├─ Velocity-aware avoidance
│  └─ Path commitment bias
│
└─ Set robot chassis speeds
```

## Troubleshooting

### Opponents don't appear on Field2d
- Check that `periodic()` is updating opponent poses
- Verify opponent simulation is initialized in constructor
- Check `simulationPeriodic()` calls `opponent.simulationPeriodic()`

### Robot doesn't avoid obstacles
- Verify A button is held (not just pressed)
- Check SmartDashboard shows correct obstacle count
- Ensure target pose is beyond obstacles (14, 4 requires crossing field)

### Path is erratic / not decisive
- Check decisiveness config (should be >98%)
- Verify rotation PID has continuous input enabled
- Ensure PID controller is not recreated each loop

### Simulation runs slow
- Reduce opponent count
- Lower telemetry verbosity in module configs
- Disable unnecessary SmartDashboard updates

## References

- [YAMS Documentation](https://yagsl.gitbook.io/yams)
- [Maple-Sim Documentation](https://docs.ironmaple.org)
- [2025 Reefscape Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system)
- [WPILib Field2d Widget](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html)

## Next Steps

- Add autonomous routines with obstacle avoidance
- Implement vision-based opponent detection
- Tune APF parameters for specific game strategies
- Test on real hardware with actual swerve modules
- Add more sophisticated opponent AI behaviors
