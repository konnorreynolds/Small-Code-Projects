# Advanced Obstacle Avoidance System

Tournament-grade obstacle avoidance with collision prediction, smart path planning, and shaped obstacles.

## Quick Start

```java
import obstacles.*;

// Create controller
AdvancedDriveController controller = new AdvancedDriveController();

// Build obstacles using factory methods
List<Obstacle> obstacles = new ArrayList<>();
obstacles.add(ObstacleFactory.robot(opponentPose, opponentVel, true));
obstacles.add(ObstacleFactory.boundary(new Translation2d(0, 0), Meters.of(0.2)));
obstacles.add(ObstacleFactory.soft(gamePiecePos, Meters.of(0.25)));

// Create config
DriveConfig config = DriveConfig.forOpponent();

// Drive with avoidance
ChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
    currentPose, targetPose, obstacles, rotationPID, config
);

swerveDrive.drive(speeds);
```

## ObstacleFactory - Single Static Reference

All obstacle shapes accessible through one static class:

### Basic Shapes

```java
// Circle
Obstacle circle = ObstacleFactory.circle(position, Meters.of(0.5));

// Rectangle
Obstacle rect = ObstacleFactory.rectangle(center, Meters.of(1.0), Meters.of(0.5));

// Square
Obstacle square = ObstacleFactory.square(center, Meters.of(0.8));

// Ellipse
Obstacle ellipse = ObstacleFactory.ellipse(center, Meters.of(1.0), Meters.of(0.5));

// Line (thin wall)
Obstacle line = ObstacleFactory.line(start, end, Meters.of(0.1));

// Polygon (approximated as circle)
Obstacle poly = ObstacleFactory.polygon(vertex1, vertex2, vertex3);
```

### Specialized Obstacles

```java
// Robot (FRC robot with bumpers)
Obstacle robot = ObstacleFactory.robot(pose, velocity, isDefensive);

// Wall between two points
Obstacle wall = ObstacleFactory.wall(startPoint, endPoint);

// Field boundary (high priority)
Obstacle boundary = ObstacleFactory.boundary(position, Meters.of(0.2));

// Zone to avoid
Obstacle zone = ObstacleFactory.zone(center, Meters.of(2.0));

// Soft obstacle (can pass through if needed)
Obstacle soft = ObstacleFactory.soft(position, Meters.of(0.3));

// Aggressive obstacle (actively blocking)
Obstacle aggressive = ObstacleFactory.aggressive(pose, velocity);

// Moving obstacle with destination
Obstacle moving = ObstacleFactory.moving(pos, vel, destination, 0.8);

// Temporary obstacle (disappears after time)
Obstacle temp = ObstacleFactory.temporary(pos, Meters.of(0.3), 5.0);
```

## Advanced Features

### 1. Collision Prediction

Time-to-collision calculation for smarter avoidance:

```java
Obstacle opponent = ObstacleFactory.robot(opponentPose, opponentVel, false);

double ttc = opponent.getTimeToCollision(myPosition, myVelocity);

if (ttc < 1.0) {
    // Collision in less than 1 second!
    // System automatically boosts avoidance
}
```

### 2. Smart Destination Prediction

Predict where obstacles are heading:

```java
Obstacle smart = ObstacleFactory.robot(pose, velocity, false)
    .withDestination(
        FieldConstants.GOAL_POSITION,
        0.9  // 90% confident they're going there
    );

// Prediction biases toward goal, not just linear
```

### 3. Velocity-Aware Avoidance

Buffer adjusts based on relative speed:

```java
config.useVelocityAwareAvoidance = true;

// Fast-moving obstacles automatically get larger buffer
// Approaching obstacles avoided more strongly
```

### 4. Path Intersection

Check if planned path intersects obstacle:

```java
Translation2d myPos = getPose().getTranslation();
Translation2d nextWaypoint = new Translation2d(5.0, 3.0);

if (obstacle.intersectsPath(myPos, nextWaypoint)) {
    // Obstacle blocks our path!
}
```

### 5. Priority System

Each obstacle has priority (0.0-1.0):

```java
obstacle.withPriority(1.0);  // Must avoid
obstacle.withPriority(0.3);  // Avoid if convenient
```

### 6. Path Smoothing

Eliminates jitter while staying responsive:

```java
config.enablePathSmoothing = true;
config.pathSmoothingFactor = 0.4;  // 0.0-1.0 (higher = smoother)
```

## DriveConfig Presets

### Aggressive (Opponent Play)

```java
DriveConfig config = DriveConfig.forOpponent();
// Fast, direct paths
// Moderate obstacle avoidance
// Good for scoring runs
```

### Precision (Careful Navigation)

```java
DriveConfig config = DriveConfig.forPrecision();
// Slower, smoother paths
// Strong obstacle avoidance
// Good for tight spaces
```

### Defense

```java
DriveConfig config = DriveConfig.forDefense();
// Moderate speed
// Strong collision prediction
// Good for defensive play
```

### Ultra-Aggressive

```java
DriveConfig config = DriveConfig.ultraAggressive();
// Maximum speed
// Minimal obstacle avoidance
// Rush to goal at all costs
```

### Ultra-Safe

```java
DriveConfig config = DriveConfig.ultraSafe();
// Very cautious
// Maximum avoidance
// Slow and steady
```

## Complete Example

```java
public class SmartNavigationExample
{
  private final AdvancedDriveController controller;
  private final SwerveDrive drive;
  private final PIDController rotationPID;
  private final DriveConfig config;

  public SmartNavigationExample(SwerveDrive drive)
  {
    this.controller = new AdvancedDriveController();
    this.drive = drive;

    this.rotationPID = new PIDController(10.0, 0.0, 0.5);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    this.config = DriveConfig.forOpponent();
    config.useCollisionPrediction = true;
    config.useVelocityAwareAvoidance = true;
    config.enablePathSmoothing = true;
  }

  public void navigateToGoal(Pose2d goal)
  {
    List<Obstacle> obstacles = buildObstacles();

    ChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
        drive.getPose(),
        goal,
        obstacles,
        rotationPID,
        config
    );

    drive.drive(speeds);
  }

  private List<Obstacle> buildObstacles()
  {
    List<Obstacle> obstacles = new ArrayList<>();

    // Opponent robots
    for (RobotInfo opp : getOpponents())
    {
      Translation2d vel = estimateVelocity(opp);

      Obstacle obs = ObstacleFactory.robot(opp.pose, vel, opp.isDefensive);

      // Add destination if known
      if (opp.hasKnownDestination())
      {
        obs.withDestination(opp.destination, 0.8);
      }

      // Mark aggressive if blocking
      if (opp.isBlockingUs())
      {
        obs.asAggressive();
      }

      obstacles.add(obs);
    }

    // Field boundaries
    obstacles.add(ObstacleFactory.boundary(
        new Translation2d(0, 0),
        Meters.of(0.2)
    ));

    // Game pieces (soft obstacles)
    for (Translation2d piece : getGamePieces())
    {
      obstacles.add(ObstacleFactory.soft(piece, Meters.of(0.25)));
    }

    return obstacles;
  }
}
```

## Tuning Guide

### For Rush to Goal

```java
config.aggressiveness = 2.0;        // Very fast
config.smoothness = 0.1;            // Responsive
config.goalBias = 3.0;              // Prefer direct path
config.softObstacleMultiplier = 0.2; // Ignore soft obstacles
```

### For Defensive Play

```java
config.baseAvoidanceStrength = 2.5;
config.collisionUrgencyMultiplier = 5.0;
config.dangerousObstacleMultiplier = 4.0;

// Mark all opponents as dangerous
obstacles.forEach(obs -> {
    obs.type = Obstacle.ObstacleType.DANGEROUS;
    obs.priority = 1.0;
});
```

### For Balanced Play

```java
DriveConfig config = DriveConfig.forOpponent();
// Already well-tuned, just add smart predictions

for (Obstacle obs : obstacles) {
    if (obs.type == Obstacle.ObstacleType.DYNAMIC) {
        obs.withDestination(findNearestGoal(obs.position), 0.7);
    }
}
```

## Features Comparison

| Feature | Old System | New System |
|---------|-----------|------------|
| Prediction | Linear only | Kinematic + destination |
| Collision Detection | Distance-based | Time-to-collision |
| Speed Adjustment | Fixed buffer | Velocity-aware |
| Path Quality | Can jitter | Smooth |
| Obstacle Intelligence | Uniform | Priority + type-based |
| Shaped Obstacles | Circle only | Circle, rect, robot, wall, etc. |

## File Structure

```
temp/
└── obstacles/
    ├── Obstacle.java                   # Core obstacle class
    ├── ObstacleFactory.java           # Factory for all shapes
    ├── DriveConfig.java               # Configuration presets
    ├── AdvancedDriveController.java   # Main drive algorithm
    └── README.md                      # This file
```

## API Reference

### ObstacleFactory Static Methods

- `circle(position, radius)` - Simple circular obstacle
- `rectangle(center, width, height)` - Rectangular obstacle
- `square(center, sideLength)` - Square obstacle
- `ellipse(center, radiusX, radiusY)` - Elliptical obstacle
- `line(start, end, thickness)` - Line/wall obstacle
- `polygon(vertices...)` - Polygon obstacle
- `robot(pose, velocity, isDefensive)` - Robot obstacle
- `wall(start, end)` - Wall between points
- `boundary(position, size)` - Field boundary
- `zone(center, radius)` - Avoidance zone
- `soft(position, radius)` - Soft obstacle
- `aggressive(pose, velocity)` - Aggressive blocker
- `moving(pos, vel, dest, confidence)` - Moving with destination
- `temporary(pos, radius, timeToLive)` - Temporary obstacle

### Obstacle Methods

- `getPredictedPosition(timeSeconds)` - Predict future position
- `getTimeToCollision(point, pointVel)` - Calculate collision time
- `getEffectiveRadius(robotVel)` - Get velocity-aware radius
- `intersectsPath(start, end)` - Check path intersection
- `withDestination(dest, confidence)` - Set likely destination
- `withPriority(priority)` - Set priority (0.0-1.0)
- `withAvoidanceWeight(weight)` - Set avoidance multiplier
- `asAggressive()` - Mark as aggressive

### DriveConfig Presets

- `DriveConfig.forOpponent()` - Aggressive opponent play
- `DriveConfig.forPrecision()` - Careful navigation
- `DriveConfig.forDefense()` - Defensive play
- `DriveConfig.ultraAggressive()` - Maximum speed
- `DriveConfig.ultraSafe()` - Maximum safety

## License

This is example code for FRC teams. Use and modify as needed.
