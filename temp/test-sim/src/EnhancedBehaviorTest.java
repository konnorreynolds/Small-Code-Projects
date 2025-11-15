import java.util.ArrayList;
import java.util.List;

/**
 * Test suite for Enhanced Obstacle Avoidance with Intent-Driven Behavior and Difficulty Control.
 *
 * Demonstrates:
 * 1. Intent-driven navigation (decisive, no floatiness)
 * 2. Difficulty parameter affecting opponent behavior
 * 3. Path commitment and momentum preferences
 */
public class EnhancedBehaviorTest
{
  public static void main(String[] args)
  {
    System.out.println("=".repeat(80));
    System.out.println("ENHANCED OBSTACLE AVOIDANCE - INTENT-DRIVEN BEHAVIOR TEST");
    System.out.println("=".repeat(80));
    System.out.println("\nNEW FEATURES:");
    System.out.println("✓ Intent-Driven Navigation (decisive movements, no floatiness)");
    System.out.println("✓ Difficulty Control (Easy/Medium/Hard opponent behavior)");
    System.out.println("✓ Path Commitment (reduced wavering)");
    System.out.println("✓ Momentum Preferences (continues chosen direction)");
    System.out.println();

    testDifficultyLevels();
    testIntentDriven();
    testPathCommitment();

    System.out.println("\n" + "=".repeat(80));
    System.out.println("ENHANCED BEHAVIOR TEST COMPLETE");
    System.out.println("=".repeat(80));
  }

  static void testDifficultyLevels()
  {
    System.out.println("TEST 1: Difficulty Levels (Easy → Medium → Hard)");
    System.out.println("-".repeat(80));

    MockPose2d start = new MockPose2d(0, 3, 0);
    MockPose2d goal = new MockPose2d(10, 3, 0);

    System.out.println("\n1A: EASY Difficulty (Slow, Predictable Opponents)");
    System.out.println("-".repeat(40));
    runWithDifficulty("Easy", start, goal, MockEnhancedAvoidance.Difficulty.EASY, 100);

    System.out.println("\n1B: MEDIUM Difficulty (Normal Opponents)");
    System.out.println("-".repeat(40));
    runWithDifficulty("Medium", start, goal, MockEnhancedAvoidance.Difficulty.MEDIUM, 100);

    System.out.println("\n1C: HARD Difficulty (Fast, Unpredictable Opponents)");
    System.out.println("-".repeat(40));
    runWithDifficulty("Hard", start, goal, MockEnhancedAvoidance.Difficulty.HARD, 100);

    System.out.println();
  }

  static void testIntentDriven()
  {
    System.out.println("TEST 2: Intent-Driven vs Standard Navigation");
    System.out.println("-".repeat(80));

    MockPose2d start = new MockPose2d(0, 0, 0);
    MockPose2d goal = new MockPose2d(8, 5, Math.toRadians(45));

    // Build obstacle field
    List<MockEnhancedObstacle> obstacles = new ArrayList<>();
    obstacles.add(MockEnhancedObstacle.createRobot(
        new MockPose2d(3, 2, 0),
        new MockTranslation2d(0.1, 0.2),
        true
    ));
    obstacles.add(MockEnhancedObstacle.createRobot(
        new MockPose2d(5, 4, 0),
        new MockTranslation2d(-0.2, -0.1),
        true
    ));

    System.out.println("\n2A: Standard Navigation (May waver/float)");
    System.out.println("-".repeat(40));
    MockEnhancedDriveConfig standardConfig = MockEnhancedDriveConfig.forOpponent();
    standardConfig.pathCommitment = 0.2;  // Low commitment
    standardConfig.momentumPreference = 0.1;  // Low momentum
    standardConfig.decisionThreshold = 0.05;  // Change direction easily
    runEnhancedSim("Standard", start, goal, obstacles, standardConfig, 120);

    System.out.println("\n2B: Intent-Driven Navigation (Decisive, committed)");
    System.out.println("-".repeat(40));
    MockEnhancedDriveConfig intentConfig = MockEnhancedDriveConfig.forOpponent();
    // Uses default high commitment values
    runEnhancedSim("Intent-Driven", start, goal, obstacles, intentConfig, 120);

    System.out.println();
  }

  static void testPathCommitment()
  {
    System.out.println("TEST 3: Path Commitment Demonstration");
    System.out.println("-".repeat(80));

    MockPose2d start = new MockPose2d(1, 3, 0);
    MockPose2d goal = new MockPose2d(9, 3, 0);

    // Moving obstacles that cross path
    List<MockEnhancedObstacle> obstacles = new ArrayList<>();
    MockEnhancedObstacle moving1 = MockEnhancedObstacle.createRobot(
        new MockPose2d(4, 5, 0),
        new MockTranslation2d(0, -0.5),  // Moving down across path
        true
    );
    moving1.likelyDestination = new MockTranslation2d(4, 1);
    moving1.confidenceLevel = 0.8;
    obstacles.add(moving1);

    MockEnhancedObstacle moving2 = MockEnhancedObstacle.createRobot(
        new MockPose2d(6, 1, 0),
        new MockTranslation2d(0, 0.5),  // Moving up across path
        true
    );
    moving2.likelyDestination = new MockTranslation2d(6, 5);
    moving2.confidenceLevel = 0.8;
    obstacles.add(moving2);

    MockEnhancedDriveConfig config = MockEnhancedDriveConfig.forOpponent();
    config.pathCommitment = 0.9;  // Very high commitment

    System.out.println("Moving obstacles crossing path...");
    System.out.println("High path commitment = Robot sticks with chosen route\n");

    runEnhancedSim("Path Commitment", start, goal, obstacles, config, 150);
    System.out.println();
  }

  static void runWithDifficulty(String name, MockPose2d start, MockPose2d goal,
                                MockEnhancedAvoidance.Difficulty difficulty, int maxSteps)
  {
    MockEnhancedSwerveDrive swerve = new MockEnhancedSwerveDrive(start, 4.0, Math.toRadians(720));
    MockEnhancedAvoidance controller = new MockEnhancedAvoidance();
    controller.setDifficulty(difficulty);

    MockPIDController rotPID = new MockPIDController(10.0, 0.0, 0.5);
    MockEnhancedDriveConfig config = MockEnhancedDriveConfig.forOpponent();

    // Create opponents scaled by difficulty
    List<MockEnhancedObstacle> obstacles = new ArrayList<>();

    MockEnhancedObstacle opp1 = MockEnhancedObstacle.createAggressive(
        new MockPose2d(4, 3, 0),
        new MockTranslation2d(0.3, 0)  // Base velocity
    ).withDifficulty(difficulty);
    obstacles.add(opp1);

    MockEnhancedObstacle opp2 = MockEnhancedObstacle.createAggressive(
        new MockPose2d(7, 3, 0),
        new MockTranslation2d(-0.3, 0)
    ).withDifficulty(difficulty);
    obstacles.add(opp2);

    double dt = 0.02;
    int step = 0;
    double totalDistance = 0;
    double maxSpeed = 0;
    boolean success = false;
    int directionChanges = 0;
    MockTranslation2d lastDir = new MockTranslation2d(0, 0);

    System.out.printf("Difficulty: %s (%.1f) | Opponents: %d\n", difficulty, difficulty.value, obstacles.size());
    System.out.printf("Opponent Speed Scale: %.1fx | Aggression Scale: %.1fx\n",
        0.5 + difficulty.value * 0.5,
        1.0 + difficulty.value * 0.5);
    System.out.println();

    for (step = 0; step < maxSteps; step++)
    {
      MockPose2d currentPose = swerve.getPose();
      MockTranslation2d currentVel = swerve.getVelocity();

      // Update moving obstacles
      for (MockEnhancedObstacle obs : obstacles)
      {
        if (obs.velocity.getNorm() > 0.01)
        {
          obs.position = obs.position.plus(obs.velocity.times(dt));
        }
      }

      MockChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
          currentPose, goal, obstacles, rotPID, config, currentVel
      );

      swerve.drive(speeds, dt);

      double stepDist = Math.sqrt(Math.pow(currentVel.x * dt, 2) + Math.pow(currentVel.y * dt, 2));
      totalDistance += stepDist;

      double speed = currentVel.getNorm();
      if (speed > maxSpeed) maxSpeed = speed;

      // Track direction changes
      MockTranslation2d currentDir = new MockTranslation2d(speeds.vx, speeds.vy);
      if (lastDir.getNorm() > 0.1 && currentDir.getNorm() > 0.1)
      {
        double angle = Math.acos(Math.max(-1.0, Math.min(1.0,
            (currentDir.x * lastDir.x + currentDir.y * lastDir.y) /
            (currentDir.getNorm() * lastDir.getNorm())
        )));
        if (angle > 0.3) directionChanges++;  // 17 degrees
      }
      lastDir = currentDir;

      // Check goal
      double distToGoal = currentPose.getTranslation().getDistance(goal.getTranslation());
      if (distToGoal < 0.15)
      {
        success = true;
        break;
      }

      if (step % 25 == 0)
      {
        System.out.printf("  t=%.2fs: Pos(%.2f, %.2f) Speed(%.2f m/s) ToGoal(%.2fm)\n",
            step * dt, currentPose.x, currentPose.y, speed, distToGoal);
      }
    }

    MockPose2d finalPose = swerve.getPose();
    double posError = finalPose.getTranslation().getDistance(goal.getTranslation());

    System.out.println("\nResults:");
    System.out.printf("  Success:             %s\n", success ? "YES ✓" : "NO ✗");
    System.out.printf("  Final Position:      (%.2f, %.2f)\n", finalPose.x, finalPose.y);
    System.out.printf("  Position Error:      %.2f meters\n", posError);
    System.out.printf("  Total Distance:      %.2f meters\n", totalDistance);
    System.out.printf("  Max Speed:           %.2f m/s\n", maxSpeed);
    System.out.printf("  Direction Changes:   %d (lower = more decisive)\n", directionChanges);
    System.out.printf("  Time:                %.2f seconds\n", step * dt);
  }

  static void runEnhancedSim(String name, MockPose2d start, MockPose2d goal,
                            List<MockEnhancedObstacle> obstacles, MockEnhancedDriveConfig config, int maxSteps)
  {
    MockEnhancedSwerveDrive swerve = new MockEnhancedSwerveDrive(start, 4.0, Math.toRadians(720));
    MockEnhancedAvoidance controller = new MockEnhancedAvoidance();
    MockPIDController rotPID = new MockPIDController(10.0, 0.0, 0.5);

    double dt = 0.02;
    int step = 0;
    double totalDistance = 0;
    double maxSpeed = 0;
    boolean success = false;
    int directionChanges = 0;
    MockTranslation2d lastDir = new MockTranslation2d(0, 0);
    double totalWavering = 0;  // Track path smoothness

    System.out.printf("Config: %s | Path Commitment: %.1f | Momentum Pref: %.1f\n",
        name, config.pathCommitment, config.momentumPreference);
    System.out.printf("Decision Threshold: %.2f rad (%.1f°)\n\n",
        config.decisionThreshold, Math.toDegrees(config.decisionThreshold));

    for (step = 0; step < maxSteps; step++)
    {
      MockPose2d currentPose = swerve.getPose();
      MockTranslation2d currentVel = swerve.getVelocity();

      // Update obstacles
      for (MockEnhancedObstacle obs : obstacles)
      {
        if (obs.velocity.getNorm() > 0.01)
        {
          obs.position = obs.position.plus(obs.velocity.times(dt));
        }
      }

      MockChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
          currentPose, goal, obstacles, rotPID, config, currentVel
      );

      swerve.drive(speeds, dt);

      double stepDist = Math.sqrt(Math.pow(currentVel.x * dt, 2) + Math.pow(currentVel.y * dt, 2));
      totalDistance += stepDist;

      double speed = currentVel.getNorm();
      if (speed > maxSpeed) maxSpeed = speed;

      // Track direction changes and wavering
      MockTranslation2d currentDir = new MockTranslation2d(speeds.vx, speeds.vy);
      if (lastDir.getNorm() > 0.1 && currentDir.getNorm() > 0.1)
      {
        double angle = Math.acos(Math.max(-1.0, Math.min(1.0,
            (currentDir.x * lastDir.x + currentDir.y * lastDir.y) /
            (currentDir.getNorm() * lastDir.getNorm())
        )));
        totalWavering += Math.abs(angle);
        if (angle > config.decisionThreshold) directionChanges++;
      }
      lastDir = currentDir;

      // Check goal
      double distToGoal = currentPose.getTranslation().getDistance(goal.getTranslation());
      if (distToGoal < 0.15)
      {
        success = true;
        break;
      }

      if (step % 30 == 0)
      {
        System.out.printf("  t=%.2fs: Pos(%.2f, %.2f) Speed(%.2f m/s)\n",
            step * dt, currentPose.x, currentPose.y, speed);
      }
    }

    MockPose2d finalPose = swerve.getPose();
    double posError = finalPose.getTranslation().getDistance(goal.getTranslation());
    double straightDist = start.getTranslation().getDistance(goal.getTranslation());
    double efficiency = straightDist / Math.max(totalDistance, 0.01);

    System.out.println("\nResults:");
    System.out.printf("  Success:             %s\n", success ? "YES ✓" : "NO ✗");
    System.out.printf("  Position Error:      %.2f meters\n", posError);
    System.out.printf("  Total Distance:      %.2f meters\n", totalDistance);
    System.out.printf("  Path Efficiency:     %.1f%%\n", efficiency * 100);
    System.out.printf("  Max Speed:           %.2f m/s\n", maxSpeed);
    System.out.printf("  Direction Changes:   %d\n", directionChanges);
    System.out.printf("  Total Wavering:      %.2f rad (%.1f°) - Lower = Smoother\n",
        totalWavering, Math.toDegrees(totalWavering));
    System.out.printf("  Decisiveness Score:  %.1f%% (higher = more decisive)\n",
        100.0 * (1.0 - Math.min(1.0, totalWavering / (step * 0.1))));
  }
}

// Mock classes for enhanced testing
class MockEnhancedSwerveDrive
{
  private MockPose2d pose;
  private MockTranslation2d velocity;
  private double maxLinearVelocity;
  private double maxAngularVelocity;

  public MockEnhancedSwerveDrive(MockPose2d initialPose, double maxLinVel, double maxAngVel)
  {
    this.pose = initialPose;
    this.velocity = new MockTranslation2d(0, 0);
    this.maxLinearVelocity = maxLinVel;
    this.maxAngularVelocity = maxAngVel;
  }

  public void drive(MockChassisSpeeds speeds, double dt)
  {
    double vx = Math.max(-maxLinearVelocity, Math.min(maxLinearVelocity, speeds.vx));
    double vy = Math.max(-maxLinearVelocity, Math.min(maxLinearVelocity, speeds.vy));
    double omega = Math.max(-maxAngularVelocity, Math.min(maxAngularVelocity, speeds.omega));

    double newX = pose.x + vx * dt;
    double newY = pose.y + vy * dt;
    double newRotation = pose.rotation + omega * dt;

    while (newRotation > Math.PI) newRotation -= 2 * Math.PI;
    while (newRotation < -Math.PI) newRotation += 2 * Math.PI;

    pose = new MockPose2d(newX, newY, newRotation);
    velocity = new MockTranslation2d(vx, vy);
  }

  public MockPose2d getPose() { return pose; }
  public MockTranslation2d getVelocity() { return velocity; }
}

class MockEnhancedObstacle
{
  public MockTranslation2d position;
  public MockTranslation2d velocity;
  public MockTranslation2d acceleration;
  public double radius;
  public String type;
  public double priority;
  public double avoidanceWeight;
  public MockTranslation2d likelyDestination;
  public double confidenceLevel;
  public boolean isAggressive;
  public double difficultyScale = 1.0;

  public MockEnhancedObstacle(MockTranslation2d position, double radius)
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

  public MockEnhancedObstacle withDifficulty(MockEnhancedAvoidance.Difficulty difficulty)
  {
    this.difficultyScale = difficulty.value;
    // Scale velocity
    if (velocity.getNorm() > 0)
    {
      double speedMultiplier = 0.5 + (difficulty.value * 0.5);
      velocity = velocity.times(speedMultiplier);
    }
    // Scale aggression
    if (isAggressive)
    {
      avoidanceWeight *= (1.0 + difficulty.value * 0.5);
    }
    // Reduce predictability
    if (difficulty == MockEnhancedAvoidance.Difficulty.HARD)
    {
      confidenceLevel *= 0.7;
    }
    return this;
  }

  public MockTranslation2d getPredictedPosition(double timeSeconds)
  {
    if (velocity.getNorm() < 0.01) return position;
    return position.plus(velocity.times(timeSeconds))
                  .plus(acceleration.times(0.5 * timeSeconds * timeSeconds));
  }

  public double getTimeToCollision(MockTranslation2d point, MockTranslation2d pointVel)
  {
    MockTranslation2d relPos = point.minus(position);
    MockTranslation2d relVel = pointVel.minus(velocity);
    if (relVel.getNorm() < 0.01) return Double.POSITIVE_INFINITY;

    double t = -(relPos.x * relVel.x + relPos.y * relVel.y) / (relVel.getNorm() * relVel.getNorm());
    if (t < 0) return Double.POSITIVE_INFINITY;

    MockTranslation2d closest = relPos.plus(relVel.times(t));
    return closest.getNorm() < radius ? t : Double.POSITIVE_INFINITY;
  }

  public double getEffectiveRadius(MockTranslation2d robotVel)
  {
    MockTranslation2d relVel = velocity.minus(robotVel);
    return radius + relVel.getNorm() * 0.3;
  }

  public static MockEnhancedObstacle createRobot(MockPose2d pose, MockTranslation2d vel, boolean isDefensive)
  {
    MockEnhancedObstacle obs = new MockEnhancedObstacle(pose.getTranslation(), 0.5);
    obs.velocity = vel;
    obs.type = "DYNAMIC";
    obs.priority = isDefensive ? 1.0 : 0.7;
    obs.avoidanceWeight = isDefensive ? 2.0 : 1.3;
    obs.isAggressive = isDefensive;
    return obs;
  }

  public static MockEnhancedObstacle createAggressive(MockPose2d pose, MockTranslation2d vel)
  {
    MockEnhancedObstacle obs = createRobot(pose, vel, true);
    obs.isAggressive = true;
    obs.avoidanceWeight *= 1.5;
    return obs;
  }
}

class MockEnhancedDriveConfig
{
  String name;
  double maxVelocity = 4.0;
  double minVelocity = 0.3;
  double aggressiveness = 1.0;
  double smoothness = 0.7;
  double goalBias = 1.5;

  // Intent-driven parameters
  double pathCommitment = 0.7;
  double momentumPreference = 0.6;
  double decisionThreshold = 0.25;
  double directBias = 2.0;

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

  static MockEnhancedDriveConfig forOpponent()
  {
    MockEnhancedDriveConfig c = new MockEnhancedDriveConfig();
    c.name = "Opponent";
    c.maxVelocity = 4.5;
    c.aggressiveness = 1.6;
    c.smoothness = 0.25;
    c.goalBias = 2.0;
    c.collisionUrgencyMultiplier = 3.0;
    c.pathCommitment = 0.8;
    c.momentumPreference = 0.7;
    c.decisionThreshold = 0.3;
    c.directBias = 2.5;
    return c;
  }
}

class MockEnhancedAvoidance
{
  public enum Difficulty
  {
    EASY(0.0), MEDIUM(0.5), HARD(1.0);
    public final double value;
    Difficulty(double value) { this.value = value; }
  }

  private MockTranslation2d previousDirection = new MockTranslation2d(0, 0);
  private MockTranslation2d committedDirection = new MockTranslation2d(0, 0);
  private double pathCommitmentTimer = 0;
  private Difficulty currentDifficulty = Difficulty.MEDIUM;

  public void setDifficulty(Difficulty difficulty) { this.currentDifficulty = difficulty; }

  MockChassisSpeeds driveToPoseWithAvoidance(MockPose2d current, MockPose2d target,
      List<MockEnhancedObstacle> obstacles, MockPIDController rotPID, MockEnhancedDriveConfig config,
      MockTranslation2d currentVel)
  {
    MockTranslation2d currentPos = current.getTranslation();
    MockTranslation2d targetPos = target.getTranslation();
    MockTranslation2d vectorToGoal = targetPos.minus(currentPos);
    double distanceToGoal = vectorToGoal.getNorm();

    MockTranslation2d desiredDirection = distanceToGoal > 0.01
        ? vectorToGoal.div(distanceToGoal)
        : new MockTranslation2d(0, 0);

    // Process obstacles (simplified for mock)
    MockTranslation2d avoidanceVector = new MockTranslation2d(0, 0);
    boolean clearPath = true;

    for (MockEnhancedObstacle obs : obstacles)
    {
      MockTranslation2d obstaclePos = obs.getPredictedPosition(config.predictionLookAhead);
      double effectiveRadius = obs.getEffectiveRadius(currentVel) + config.defaultAvoidanceRadius;

      MockTranslation2d toObstacle = currentPos.minus(obstaclePos);
      double obstacleDistance = toObstacle.getNorm();

      if (obstacleDistance > effectiveRadius) continue;
      clearPath = false;

      if (obstacleDistance > 0.01)
      {
        double normalizedDist = obstacleDistance / effectiveRadius;
        double repulsionMag = config.baseAvoidanceStrength * obs.avoidanceWeight *
            (1.0 + obs.difficultyScale * 0.3) * Math.pow(1.0 - normalizedDist, 2.5);

        avoidanceVector = avoidanceVector.plus(toObstacle.div(obstacleDistance).times(repulsionMag));
      }
    }

    // Intent-driven behavior
    MockTranslation2d rawCombinedDirection;
    if (clearPath)
    {
      rawCombinedDirection = desiredDirection.times(config.directBias);
    }
    else
    {
      rawCombinedDirection = desiredDirection.times(config.goalBias).plus(avoidanceVector);
    }

    double combinedMag = rawCombinedDirection.getNorm();
    if (combinedMag > 0.01) rawCombinedDirection = rawCombinedDirection.div(combinedMag);
    else rawCombinedDirection = desiredDirection;

    // Apply momentum preference
    MockTranslation2d combinedDirection;
    if (previousDirection.getNorm() > 0.01 && currentVel.getNorm() > 0.1)
    {
      double directionChange = Math.acos(Math.max(-1.0, Math.min(1.0,
          rawCombinedDirection.x * previousDirection.x + rawCombinedDirection.y * previousDirection.y
      )));

      if (directionChange < config.decisionThreshold && pathCommitmentTimer > 0)
      {
        combinedDirection = committedDirection;
        pathCommitmentTimer -= 0.02;
      }
      else if (directionChange > config.decisionThreshold)
      {
        combinedDirection = rawCombinedDirection.times(1.0 - config.momentumPreference)
                          .plus(previousDirection.times(config.momentumPreference));
        double mag = combinedDirection.getNorm();
        if (mag > 0.01) combinedDirection = combinedDirection.div(mag);

        committedDirection = combinedDirection;
        pathCommitmentTimer = 0.3 * config.pathCommitment;
      }
      else
      {
        combinedDirection = rawCombinedDirection.times(1.0 - config.momentumPreference * 0.5)
                          .plus(previousDirection.times(config.momentumPreference * 0.5));
        double mag = combinedDirection.getNorm();
        if (mag > 0.01) combinedDirection = combinedDirection.div(mag);
      }
    }
    else
    {
      combinedDirection = rawCombinedDirection;
      committedDirection = combinedDirection;
      pathCommitmentTimer = 0.3 * config.pathCommitment;
    }

    previousDirection = combinedDirection;

    // Calculate speed
    double desiredSpeed = distanceToGoal < config.slowZoneDistance
        ? config.minVelocity + (config.maxVelocity - config.minVelocity) *
          Math.pow(distanceToGoal / config.slowZoneDistance, 0.7)
        : config.maxVelocity;

    desiredSpeed *= config.aggressiveness * (1.0 - config.smoothness);

    double vx = combinedDirection.x * desiredSpeed;
    double vy = combinedDirection.y * desiredSpeed;
    double omega = rotPID.calculate(current.rotation, target.rotation);

    return new MockChassisSpeeds(vx, vy, omega);
  }
}
