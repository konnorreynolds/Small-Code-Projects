import java.util.ArrayList;
import java.util.List;

/**
 * Simulation test for advanced obstacle avoidance system.
 * Tests multiple scenarios and outputs navigation results.
 */
public class ObstacleAvoidanceSimulation
{
  public static void main(String[] args)
  {
    System.out.println("=".repeat(70));
    System.out.println("ADVANCED OBSTACLE AVOIDANCE SYSTEM - SIMULATION TEST");
    System.out.println("=".repeat(70));
    System.out.println();

    // Run test scenarios
    testScenario1_DirectPath();
    testScenario2_SingleObstacle();
    testScenario3_MultipleStaticObstacles();
    testScenario4_MovingObstacles();
    testScenario5_AggressiveDefender();
    testScenario6_ComplexField();

    System.out.println("\n" + "=".repeat(70));
    System.out.println("SIMULATION COMPLETE");
    System.out.println("=".repeat(70));
  }

  static void testScenario1_DirectPath()
  {
    System.out.println("TEST 1: Direct Path (No Obstacles)");
    System.out.println("-".repeat(70));

    MockPose2d start = new MockPose2d(0, 0, 0);
    MockPose2d goal = new MockPose2d(5, 5, 0);
    List<MockObstacle> obstacles = new ArrayList<>();

    runSimulation("Direct Path", start, goal, obstacles, MockDriveConfig.forOpponent(), 10);
    System.out.println();
  }

  static void testScenario2_SingleObstacle()
  {
    System.out.println("TEST 2: Single Static Obstacle in Path");
    System.out.println("-".repeat(70));

    MockPose2d start = new MockPose2d(0, 0, 0);
    MockPose2d goal = new MockPose2d(5, 0, 0);
    List<MockObstacle> obstacles = new ArrayList<>();
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(2.5, 0), 0.8));

    runSimulation("Single Obstacle", start, goal, obstacles, MockDriveConfig.forOpponent(), 15);
    System.out.println();
  }

  static void testScenario3_MultipleStaticObstacles()
  {
    System.out.println("TEST 3: Multiple Static Obstacles");
    System.out.println("-".repeat(70));

    MockPose2d start = new MockPose2d(0, 0, 0);
    MockPose2d goal = new MockPose2d(8, 8, 0);
    List<MockObstacle> obstacles = new ArrayList<>();
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(2, 2), 0.6));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(4, 4), 0.6));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(6, 6), 0.6));

    runSimulation("Multiple Obstacles", start, goal, obstacles, MockDriveConfig.forOpponent(), 20);
    System.out.println();
  }

  static void testScenario4_MovingObstacles()
  {
    System.out.println("TEST 4: Moving Obstacles (Collision Prediction)");
    System.out.println("-".repeat(70));

    MockPose2d start = new MockPose2d(0, 0, 0);
    MockPose2d goal = new MockPose2d(5, 0, 0);
    List<MockObstacle> obstacles = new ArrayList<>();

    // Moving obstacle crossing path
    MockObstacle moving = MockObstacle.createRobot(
        new MockPose2d(2.5, 3, 0),
        new MockTranslation2d(0, -0.5),  // Moving toward path
        false
    );
    obstacles.add(moving);

    runSimulation("Moving Obstacle", start, goal, obstacles, MockDriveConfig.forOpponent(), 20);
    System.out.println();
  }

  static void testScenario5_AggressiveDefender()
  {
    System.out.println("TEST 5: Aggressive Defender Blocking");
    System.out.println("-".repeat(70));

    MockPose2d start = new MockPose2d(0, 0, 0);
    MockPose2d goal = new MockPose2d(8, 0, 0);
    List<MockObstacle> obstacles = new ArrayList<>();

    // Aggressive defender
    MockObstacle defender = MockObstacle.createAggressive(
        new MockPose2d(4, 0, 0),
        new MockTranslation2d(-0.2, 0)  // Moving toward us
    );
    obstacles.add(defender);

    runSimulation("Aggressive Defender", start, goal, obstacles, MockDriveConfig.forOpponent(), 25);
    System.out.println();
  }

  static void testScenario6_ComplexField()
  {
    System.out.println("TEST 6: Complex Field (Walls, Zones, Mixed Obstacles)");
    System.out.println("-".repeat(70));

    MockPose2d start = new MockPose2d(1, 1, 0);
    MockPose2d goal = new MockPose2d(7, 7, 0);
    List<MockObstacle> obstacles = new ArrayList<>();

    // Field boundaries
    obstacles.add(MockObstacle.createBoundary(new MockTranslation2d(0, 0), 0.3));
    obstacles.add(MockObstacle.createBoundary(new MockTranslation2d(8, 8), 0.3));

    // Wall
    obstacles.add(MockObstacle.createWall(
        new MockTranslation2d(3, 2),
        new MockTranslation2d(3, 6)
    ));

    // Avoidance zone
    obstacles.add(MockObstacle.createZone(new MockTranslation2d(5, 4), 1.5));

    // Game pieces (soft obstacles)
    obstacles.add(MockObstacle.createSoft(new MockTranslation2d(2, 3), 0.3));
    obstacles.add(MockObstacle.createSoft(new MockTranslation2d(6, 5), 0.3));

    runSimulation("Complex Field", start, goal, obstacles, MockDriveConfig.forPrecision(), 30);
    System.out.println();
  }

  static void runSimulation(String name, MockPose2d start, MockPose2d goal,
                           List<MockObstacle> obstacles, MockDriveConfig config, int steps)
  {
    MockAdvancedObstacleAvoidance controller = new MockAdvancedObstacleAvoidance();
    MockPIDController rotationPID = new MockPIDController(10.0, 0.0, 0.5);

    MockPose2d currentPose = start;
    MockTranslation2d currentVel = new MockTranslation2d(0, 0);
    double dt = 0.02; // 20ms timestep

    System.out.printf("Start: (%.2f, %.2f) -> Goal: (%.2f, %.2f)\n",
        start.x, start.y, goal.x, goal.y);
    System.out.printf("Obstacles: %d | Config: %s | Steps: %d\n",
        obstacles.size(), config.name, steps);
    System.out.println();

    boolean reachedGoal = false;
    boolean collision = false;
    double totalDistance = 0;
    double maxAvoidanceForce = 0;

    for (int i = 0; i < steps; i++)
    {
      // Update moving obstacles
      for (MockObstacle obs : obstacles)
      {
        if (obs.velocity.getNorm() > 0.01)
        {
          obs.position = obs.position.plus(obs.velocity.times(dt));
        }
      }

      // Run avoidance algorithm
      MockChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
          currentPose, goal, obstacles, rotationPID, config, currentVel
      );

      // Update robot position
      double vx = speeds.vx * dt;
      double vy = speeds.vy * dt;
      totalDistance += Math.sqrt(vx * vx + vy * vy);

      currentPose = new MockPose2d(
          currentPose.x + vx,
          currentPose.y + vy,
          currentPose.rotation + speeds.omega * dt
      );

      currentVel = new MockTranslation2d(speeds.vx, speeds.vy);

      // Check for collisions
      for (MockObstacle obs : obstacles)
      {
        double dist = currentPose.getTranslation().getDistance(obs.position);
        if (dist < obs.radius + 0.4) // Robot radius ~0.4m
        {
          collision = true;
          System.out.printf("  ⚠️  COLLISION at step %d with obstacle at (%.2f, %.2f)\n",
              i, obs.position.x, obs.position.y);
          break;
        }
      }

      if (collision) break;

      // Check if reached goal
      double distToGoal = currentPose.getTranslation().getDistance(goal.getTranslation());
      if (distToGoal < 0.15)
      {
        reachedGoal = true;
        System.out.printf("  ✓ Reached goal at step %d\n", i);
        break;
      }

      // Print progress every 5 steps
      if (i % 5 == 0)
      {
        System.out.printf("  Step %2d: Pos(%.2f, %.2f) Speed(%.2f m/s) DistToGoal(%.2fm)\n",
            i, currentPose.x, currentPose.y,
            Math.sqrt(speeds.vx * speeds.vx + speeds.vy * speeds.vy),
            distToGoal);
      }
    }

    System.out.println();
    System.out.println("Results:");
    System.out.printf("  Final Position: (%.2f, %.2f)\n", currentPose.x, currentPose.y);
    System.out.printf("  Total Distance: %.2f meters\n", totalDistance);
    System.out.printf("  Success: %s\n", reachedGoal ? "YES ✓" : "NO ✗");
    if (collision) System.out.println("  Collision: YES ⚠️");
  }
}

// ============================================================================
// MOCK CLASSES FOR SIMULATION
// ============================================================================

class MockTranslation2d
{
  double x, y;

  MockTranslation2d(double x, double y) { this.x = x; this.y = y; }

  MockTranslation2d plus(MockTranslation2d other) { return new MockTranslation2d(x + other.x, y + other.y); }
  MockTranslation2d minus(MockTranslation2d other) { return new MockTranslation2d(x - other.x, y - other.y); }
  MockTranslation2d times(double scalar) { return new MockTranslation2d(x * scalar, y * scalar); }
  MockTranslation2d div(double scalar) { return new MockTranslation2d(x / scalar, y / scalar); }
  double getNorm() { return Math.sqrt(x * x + y * y); }
  double getDistance(MockTranslation2d other) { return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2)); }
  double getX() { return x; }
  double getY() { return y; }
}

class MockPose2d
{
  double x, y, rotation;

  MockPose2d(double x, double y, double rotation) { this.x = x; this.y = y; this.rotation = rotation; }
  MockTranslation2d getTranslation() { return new MockTranslation2d(x, y); }
  double getRotation() { return rotation; }
}

class MockObstacle
{
  MockTranslation2d position;
  MockTranslation2d velocity;
  MockTranslation2d acceleration;
  double radius;
  String type;
  double priority;
  double avoidanceWeight;
  MockTranslation2d likelyDestination;
  double confidenceLevel;
  boolean isAggressive;

  MockObstacle(MockTranslation2d position, double radius)
  {
    this.position = position;
    this.radius = radius;
    this.velocity = new MockTranslation2d(0, 0);
    this.acceleration = new MockTranslation2d(0, 0);
    this.type = "STATIC";
    this.priority = 1.0;
    this.avoidanceWeight = 1.0;
    this.likelyDestination = position;
    this.confidenceLevel = 1.0;
    this.isAggressive = false;
  }

  MockTranslation2d getPredictedPosition(double timeSeconds)
  {
    if (velocity.getNorm() < 0.01) return position;
    return position.plus(velocity.times(timeSeconds))
                  .plus(acceleration.times(0.5 * timeSeconds * timeSeconds));
  }

  double getTimeToCollision(MockTranslation2d point, MockTranslation2d pointVel)
  {
    MockTranslation2d relPos = point.minus(position);
    MockTranslation2d relVel = pointVel.minus(velocity);
    if (relVel.getNorm() < 0.01) return Double.POSITIVE_INFINITY;

    double t = -(relPos.x * relVel.x + relPos.y * relVel.y) / (relVel.getNorm() * relVel.getNorm());
    if (t < 0) return Double.POSITIVE_INFINITY;

    MockTranslation2d closest = relPos.plus(relVel.times(t));
    return closest.getNorm() < radius ? t : Double.POSITIVE_INFINITY;
  }

  double getEffectiveRadius(MockTranslation2d robotVel)
  {
    MockTranslation2d relVel = velocity.minus(robotVel);
    return radius + relVel.getNorm() * 0.3;
  }

  static MockObstacle createCircle(MockTranslation2d pos, double radius)
  {
    return new MockObstacle(pos, radius);
  }

  static MockObstacle createRobot(MockPose2d pose, MockTranslation2d vel, boolean isDefensive)
  {
    MockObstacle obs = new MockObstacle(pose.getTranslation(), 0.5);
    obs.velocity = vel;
    obs.type = "DYNAMIC";
    obs.priority = isDefensive ? 1.0 : 0.7;
    obs.avoidanceWeight = isDefensive ? 2.0 : 1.3;
    obs.isAggressive = isDefensive;
    return obs;
  }

  static MockObstacle createWall(MockTranslation2d start, MockTranslation2d end)
  {
    MockTranslation2d center = start.plus(end).div(2.0);
    MockObstacle obs = new MockObstacle(center, 0.2);
    obs.type = "DANGEROUS";
    obs.priority = 1.0;
    obs.avoidanceWeight = 3.0;
    return obs;
  }

  static MockObstacle createBoundary(MockTranslation2d pos, double size)
  {
    MockObstacle obs = new MockObstacle(pos, size);
    obs.type = "DANGEROUS";
    obs.priority = 1.0;
    obs.avoidanceWeight = 5.0;
    return obs;
  }

  static MockObstacle createSoft(MockTranslation2d pos, double radius)
  {
    MockObstacle obs = new MockObstacle(pos, radius);
    obs.type = "SOFT";
    obs.priority = 0.3;
    obs.avoidanceWeight = 0.4;
    return obs;
  }

  static MockObstacle createZone(MockTranslation2d center, double radius)
  {
    MockObstacle obs = new MockObstacle(center, radius);
    obs.type = "ZONE";
    obs.priority = 0.5;
    obs.avoidanceWeight = 0.8;
    return obs;
  }

  static MockObstacle createAggressive(MockPose2d pose, MockTranslation2d vel)
  {
    MockObstacle obs = createRobot(pose, vel, true);
    obs.isAggressive = true;
    obs.avoidanceWeight *= 1.5;
    return obs;
  }
}

class MockDriveConfig
{
  String name;
  double maxVelocity = 4.0;
  double minVelocity = 0.3;
  double aggressiveness = 1.0;
  double smoothness = 0.7;
  double goalBias = 1.5;
  double baseAvoidanceStrength = 1.0;
  double defaultAvoidanceRadius = 1.2;
  double dangerRadius = 0.4;
  double predictionLookAhead = 0.8;
  boolean useCollisionPrediction = true;
  boolean useVelocityAwareAvoidance = true;
  boolean respectObstaclePriority = true;
  double collisionUrgencyMultiplier = 2.0;
  double staticObstacleMultiplier = 1.0;
  double dynamicObstacleMultiplier = 1.4;
  double dangerousObstacleMultiplier = 2.5;
  double softObstacleMultiplier = 0.4;
  double zoneObstacleMultiplier = 0.6;
  double aggressiveObstacleMultiplier = 1.8;
  double fastZoneDistance = 2.0;
  double slowZoneDistance = 0.5;
  double precisionDistance = 0.15;
  boolean useAdaptiveSpeed = true;
  boolean enableAngularAvoidance = true;
  boolean enablePathSmoothing = true;
  double pathSmoothingFactor = 0.3;

  static MockDriveConfig forOpponent()
  {
    MockDriveConfig c = new MockDriveConfig();
    c.name = "Opponent";
    c.maxVelocity = 4.5;
    c.aggressiveness = 1.6;
    c.smoothness = 0.25;
    c.goalBias = 2.0;
    c.collisionUrgencyMultiplier = 3.0;
    return c;
  }

  static MockDriveConfig forPrecision()
  {
    MockDriveConfig c = new MockDriveConfig();
    c.name = "Precision";
    c.maxVelocity = 2.0;
    c.aggressiveness = 0.7;
    c.smoothness = 0.8;
    c.baseAvoidanceStrength = 2.0;
    return c;
  }
}

class MockChassisSpeeds
{
  double vx, vy, omega;
  MockChassisSpeeds(double vx, double vy, double omega) { this.vx = vx; this.vy = vy; this.omega = omega; }
}

class MockPIDController
{
  double kP, kI, kD;
  MockPIDController(double kP, double kI, double kD) { this.kP = kP; this.kI = kI; this.kD = kD; }
  double calculate(double current, double setpoint) { return kP * (setpoint - current); }
}

class MockAdvancedObstacleAvoidance
{
  MockTranslation2d previousDirection = new MockTranslation2d(0, 0);

  MockChassisSpeeds driveToPoseWithAvoidance(MockPose2d current, MockPose2d target,
      List<MockObstacle> obstacles, MockPIDController rotPID, MockDriveConfig config,
      MockTranslation2d currentVel)
  {
    MockTranslation2d currentPos = current.getTranslation();
    MockTranslation2d targetPos = target.getTranslation();
    MockTranslation2d vectorToGoal = targetPos.minus(currentPos);
    double distanceToGoal = vectorToGoal.getNorm();

    MockTranslation2d desiredDirection = distanceToGoal > 0.01
        ? vectorToGoal.div(distanceToGoal)
        : new MockTranslation2d(0, 0);

    // Process obstacles
    MockTranslation2d avoidanceVector = new MockTranslation2d(0, 0);
    double closestObstacle = Double.POSITIVE_INFINITY;
    double minTimeToCollision = Double.POSITIVE_INFINITY;

    for (MockObstacle obs : obstacles)
    {
      MockTranslation2d obstaclePos = obs.getPredictedPosition(config.predictionLookAhead);
      double effectiveRadius = config.useVelocityAwareAvoidance
          ? obs.getEffectiveRadius(currentVel) + config.defaultAvoidanceRadius
          : obs.radius + config.defaultAvoidanceRadius;

      MockTranslation2d toObstacle = currentPos.minus(obstaclePos);
      double obstacleDistance = toObstacle.getNorm();

      if (obstacleDistance < closestObstacle) closestObstacle = obstacleDistance;

      if (config.useCollisionPrediction)
      {
        double ttc = obs.getTimeToCollision(currentPos, currentVel);
        if (ttc < minTimeToCollision) minTimeToCollision = ttc;
      }

      if (obstacleDistance > effectiveRadius) continue;

      if (obstacleDistance > 0.01)
      {
        double typeMultiplier = switch (obs.type)
        {
          case "STATIC" -> config.staticObstacleMultiplier;
          case "DYNAMIC" -> config.dynamicObstacleMultiplier;
          case "DANGEROUS" -> config.dangerousObstacleMultiplier;
          case "SOFT" -> config.softObstacleMultiplier;
          case "ZONE" -> config.zoneObstacleMultiplier;
          default -> 1.0;
        };

        if (obs.isAggressive) typeMultiplier *= config.aggressiveObstacleMultiplier;

        double normalizedDist = obstacleDistance / effectiveRadius;
        double repulsionMag = config.baseAvoidanceStrength * obs.avoidanceWeight * typeMultiplier *
            Math.pow(1.0 - normalizedDist, 2.5);

        if (config.respectObstaclePriority) repulsionMag *= obs.priority;
        if (obstacleDistance < config.dangerRadius + obs.radius) repulsionMag *= 4.0;

        if (config.useCollisionPrediction && minTimeToCollision < 2.0)
        {
          repulsionMag *= 1.0 + config.collisionUrgencyMultiplier * (2.0 - minTimeToCollision) / 2.0;
        }

        MockTranslation2d repulsion = toObstacle.div(obstacleDistance).times(repulsionMag);
        avoidanceVector = avoidanceVector.plus(repulsion);
      }
    }

    // Combine direction
    MockTranslation2d combinedDirection = desiredDirection.times(config.goalBias).plus(avoidanceVector);
    double combinedMag = combinedDirection.getNorm();
    if (combinedMag > 0.01) combinedDirection = combinedDirection.div(combinedMag);
    else combinedDirection = desiredDirection;

    // Path smoothing
    if (config.enablePathSmoothing && previousDirection.getNorm() > 0.01)
    {
      combinedDirection = combinedDirection.times(1.0 - config.pathSmoothingFactor)
          .plus(previousDirection.times(config.pathSmoothingFactor));
      double smoothedMag = combinedDirection.getNorm();
      if (smoothedMag > 0.01) combinedDirection = combinedDirection.div(smoothedMag);
    }
    previousDirection = combinedDirection;

    // Calculate speed
    double desiredSpeed;
    if (distanceToGoal < config.precisionDistance)
    {
      desiredSpeed = distanceToGoal * 2.5 * config.aggressiveness;
    }
    else if (distanceToGoal < config.slowZoneDistance)
    {
      double slowFactor = (distanceToGoal - config.precisionDistance) /
                         (config.slowZoneDistance - config.precisionDistance);
      desiredSpeed = config.minVelocity + (config.maxVelocity - config.minVelocity) *
          Math.pow(slowFactor, 0.7) * config.aggressiveness;
    }
    else
    {
      desiredSpeed = config.maxVelocity * config.aggressiveness;
    }

    // Adaptive speed
    if (config.useAdaptiveSpeed && !obstacles.isEmpty())
    {
      if (closestObstacle < config.defaultAvoidanceRadius)
      {
        desiredSpeed *= 0.35 + 0.65 * Math.pow(closestObstacle / config.defaultAvoidanceRadius, 0.5);
      }
      if (config.useCollisionPrediction && minTimeToCollision < 1.5)
      {
        desiredSpeed *= 0.4 + 0.6 * (minTimeToCollision / 1.5);
      }
    }

    desiredSpeed = Math.max(config.minVelocity, Math.min(config.maxVelocity, desiredSpeed));
    desiredSpeed *= (1.0 - config.smoothness);

    double vx = combinedDirection.x * desiredSpeed;
    double vy = combinedDirection.y * desiredSpeed;
    double omega = rotPID.calculate(current.rotation, target.rotation);

    return new MockChassisSpeeds(vx, vy, omega);
  }
}
