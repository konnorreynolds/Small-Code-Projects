import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.*;

import java.util.List;

/**
 * Enhanced obstacle avoidance with intent-driven behavior and difficulty control.
 *
 * NEW FEATURES:
 * - Intent-driven navigation (no floatiness - decisive movements)
 * - Difficulty parameter (0.0-1.0) for opponent behavior
 * - Path commitment (reduced wavering)
 * - Momentum-based decisions
 *
 * Usage:
 *   EnhancedObstacleAvoidance controller = new EnhancedObstacleAvoidance();
 *   controller.setDifficulty(Difficulty.HARD);
 *   ChassisSpeeds speeds = controller.driveToPoseWithAvoidance(...);
 */
public class EnhancedObstacleAvoidance
{
  private Translation2d previousDirection = new Translation2d();
  private Translation2d committedDirection = new Translation2d();
  private double pathCommitmentTimer = 0;
  private Difficulty currentDifficulty = Difficulty.MEDIUM;

  // Intent-driven parameters
  private static final double PATH_COMMITMENT_TIME = 0.3;  // Stick with decision for 300ms
  private static final double DIRECTION_CHANGE_THRESHOLD = 0.3;  // Min angle change to alter course
  private static final double MOMENTUM_FACTOR = 0.6;  // Prefer continuing current direction

  /**
   * Difficulty levels affect opponent behavior and prediction uncertainty.
   */
  public enum Difficulty
  {
    EASY(0.0),    // Slow opponents, predictable, low aggression
    MEDIUM(0.5),  // Normal behavior
    HARD(1.0);    // Fast opponents, unpredictable, high aggression

    public final double value;
    Difficulty(double value) { this.value = value; }
  }

  public void setDifficulty(Difficulty difficulty)
  {
    this.currentDifficulty = difficulty;
  }

  public void setDifficulty(double value)
  {
    this.currentDifficulty = value < 0.33 ? Difficulty.EASY :
                            value < 0.67 ? Difficulty.MEDIUM : Difficulty.HARD;
  }

  // ========================================================================
  // OBSTACLE CLASS (Enhanced)
  // ========================================================================

  public static class Obstacle
  {
    public Translation2d position;
    public Translation2d velocity;
    public Translation2d acceleration;
    public ObstacleShape shape;
    public Measure<Distance> radius;
    public Measure<Distance> width, height;
    public ObstacleType type;
    public double priority;
    public double avoidanceWeight;
    public Translation2d likelyDestination;
    public double confidenceLevel;
    public boolean isAggressive;
    public double timeToLive;
    public String id;

    // NEW: Difficulty-scaled properties
    public double difficultyScale = 1.0;

    public enum ObstacleType { STATIC, DYNAMIC, DANGEROUS, SOFT, ZONE }
    public enum ObstacleShape { CIRCLE, RECTANGLE, ROBOT }

    public Obstacle(Translation2d position, Measure<Distance> radius)
    {
      this.position = position;
      this.radius = radius;
      this.velocity = new Translation2d();
      this.acceleration = new Translation2d();
      this.shape = ObstacleShape.CIRCLE;
      this.type = ObstacleType.STATIC;
      this.priority = 1.0;
      this.avoidanceWeight = 1.0;
      this.likelyDestination = position;
      this.confidenceLevel = 1.0;
      this.isAggressive = false;
      this.timeToLive = Double.POSITIVE_INFINITY;
      this.id = "obs_" + System.currentTimeMillis();
    }

    /**
     * Scale obstacle difficulty (affects speed, aggression, unpredictability).
     */
    public Obstacle withDifficulty(Difficulty difficulty)
    {
      this.difficultyScale = difficulty.value;

      // Scale velocity based on difficulty
      if (velocity.getNorm() > 0)
      {
        double speedMultiplier = 0.5 + (difficulty.value * 0.5);  // 0.5x to 1.0x
        velocity = velocity.times(speedMultiplier);
      }

      // Scale aggression
      if (isAggressive)
      {
        avoidanceWeight *= (1.0 + difficulty.value * 0.5);  // Up to 1.5x more aggressive
      }

      // Add unpredictability at high difficulty
      if (difficulty == Difficulty.HARD)
      {
        confidenceLevel *= 0.7;  // Less predictable
      }

      return this;
    }

    public Translation2d getPredictedPosition(double timeSeconds)
    {
      if (velocity.getNorm() < 0.01) return position;

      Translation2d predicted = position
          .plus(velocity.times(timeSeconds))
          .plus(acceleration.times(0.5 * timeSeconds * timeSeconds));

      if (confidenceLevel > 0.7 && likelyDestination != null)
      {
        Translation2d toDestination = likelyDestination.minus(predicted);
        if (toDestination.getNorm() > 0.1)
        {
          predicted = predicted.plus(toDestination.times(confidenceLevel * 0.3));
        }
      }

      return predicted;
    }

    public double getTimeToCollision(Translation2d point, Translation2d pointVelocity)
    {
      Translation2d relativePos = point.minus(position);
      Translation2d relativeVel = pointVelocity.minus(velocity);
      if (relativeVel.getNorm() < 0.01) return Double.POSITIVE_INFINITY;

      double t = -(relativePos.getX() * relativeVel.getX() + relativePos.getY() * relativeVel.getY()) /
                  (relativeVel.getNorm() * relativeVel.getNorm());
      if (t < 0) return Double.POSITIVE_INFINITY;

      Translation2d closestPoint = relativePos.plus(relativeVel.times(t));
      return closestPoint.getNorm() < radius.in(Meters) ? t : Double.POSITIVE_INFINITY;
    }

    public Measure<Distance> getEffectiveRadius(Translation2d robotVelocity)
    {
      Translation2d relativeVel = velocity.minus(robotVelocity);
      return Meters.of(radius.in(Meters) + relativeVel.getNorm() * 0.3);
    }

    public boolean intersectsPath(Translation2d start, Translation2d end)
    {
      Translation2d path = end.minus(start);
      double pathLength = path.getNorm();
      if (pathLength < 0.01) return start.getDistance(position) < radius.in(Meters);

      Translation2d toObstacle = position.minus(start);
      double projection = (toObstacle.getX() * path.getX() + toObstacle.getY() * path.getY()) / pathLength;
      projection = Math.max(0, Math.min(pathLength, projection));

      Translation2d closestPoint = start.plus(path.div(pathLength).times(projection));
      return closestPoint.getDistance(position) < radius.in(Meters);
    }

    public Obstacle withDestination(Translation2d destination, double confidence)
    {
      this.likelyDestination = destination;
      this.confidenceLevel = Math.max(0.0, Math.min(1.0, confidence));
      return this;
    }

    public Obstacle asAggressive()
    {
      this.isAggressive = true;
      this.avoidanceWeight *= 1.5;
      return this;
    }

    public Obstacle withPriority(double priority)
    {
      this.priority = Math.max(0.0, Math.min(1.0, priority));
      return this;
    }

    public Obstacle withAvoidanceWeight(double weight)
    {
      this.avoidanceWeight = weight;
      return this;
    }

    public Obstacle withAcceleration(Translation2d acceleration)
    {
      this.acceleration = acceleration;
      return this;
    }
  }

  // ========================================================================
  // DRIVE CONFIG (Enhanced)
  // ========================================================================

  public static class DriveConfig
  {
    // Speed limits
    public Measure<Velocity<Distance>> maxVelocity = MetersPerSecond.of(4.0);
    public Measure<Velocity<Distance>> minVelocity = MetersPerSecond.of(0.3);
    public Measure<Velocity<Angle>> maxAngularVelocity = RadiansPerSecond.of(2.0 * Math.PI);

    // Behavior tuning
    public double aggressiveness = 1.0;
    public double smoothness = 0.7;
    public double goalBias = 1.5;

    // NEW: Intent-driven parameters
    public double pathCommitment = 0.7;      // How strongly to stick with chosen path (0.0-1.0)
    public double momentumPreference = 0.6;   // Prefer continuing current direction
    public double decisionThreshold = 0.25;   // Min change needed to alter course (radians)
    public double directBias = 2.0;          // Bias toward direct path when safe

    // Avoidance
    public double baseAvoidanceStrength = 1.0;
    public Measure<Distance> defaultAvoidanceRadius = Meters.of(1.2);
    public Measure<Distance> dangerRadius = Meters.of(0.4);

    // Prediction
    public double predictionLookAhead = 0.8;
    public boolean useCollisionPrediction = true;
    public boolean useVelocityAwareAvoidance = true;
    public boolean preferSaferPaths = true;
    public boolean respectObstaclePriority = true;
    public double collisionUrgencyMultiplier = 2.0;

    // Type multipliers
    public double staticObstacleMultiplier = 1.0;
    public double dynamicObstacleMultiplier = 1.4;
    public double dangerousObstacleMultiplier = 2.5;
    public double softObstacleMultiplier = 0.4;
    public double zoneObstacleMultiplier = 0.6;
    public double aggressiveObstacleMultiplier = 1.8;

    // Zones
    public Measure<Distance> fastZoneDistance = Meters.of(2.0);
    public Measure<Distance> slowZoneDistance = Meters.of(0.5);
    public Measure<Distance> precisionDistance = Meters.of(0.15);

    // Advanced
    public boolean useAdaptiveSpeed = true;
    public boolean enableAngularAvoidance = true;
    public boolean enablePathSmoothing = true;
    public double pathSmoothingFactor = 0.3;

    // Presets
    public static DriveConfig forOpponent()
    {
      DriveConfig c = new DriveConfig();
      c.maxVelocity = MetersPerSecond.of(4.5);
      c.maxAngularVelocity = RadiansPerSecond.of(3.0 * Math.PI);
      c.aggressiveness = 1.6;
      c.smoothness = 0.25;
      c.goalBias = 2.0;
      c.baseAvoidanceStrength = 1.3;
      c.defaultAvoidanceRadius = Meters.of(1.0);
      c.collisionUrgencyMultiplier = 3.0;
      c.dynamicObstacleMultiplier = 1.6;
      c.aggressiveObstacleMultiplier = 2.2;
      c.pathSmoothingFactor = 0.4;
      // Intent-driven
      c.pathCommitment = 0.8;
      c.momentumPreference = 0.7;
      c.decisionThreshold = 0.3;
      c.directBias = 2.5;
      return c;
    }

    public static DriveConfig forPrecision()
    {
      DriveConfig c = new DriveConfig();
      c.maxVelocity = MetersPerSecond.of(2.0);
      c.aggressiveness = 0.7;
      c.smoothness = 0.8;
      c.baseAvoidanceStrength = 2.0;
      c.defaultAvoidanceRadius = Meters.of(1.6);
      c.dangerousObstacleMultiplier = 3.0;
      c.pathSmoothingFactor = 0.6;
      // Intent-driven (more cautious)
      c.pathCommitment = 0.5;
      c.momentumPreference = 0.4;
      c.decisionThreshold = 0.15;
      c.directBias = 1.2;
      return c;
    }

    public static DriveConfig forDefense()
    {
      DriveConfig c = new DriveConfig();
      c.maxVelocity = MetersPerSecond.of(3.5);
      c.aggressiveness = 1.2;
      c.smoothness = 0.5;
      c.goalBias = 1.8;
      c.baseAvoidanceStrength = 1.5;
      c.collisionUrgencyMultiplier = 2.5;
      c.dynamicObstacleMultiplier = 1.8;
      c.aggressiveObstacleMultiplier = 2.5;
      // Intent-driven (balanced)
      c.pathCommitment = 0.6;
      c.momentumPreference = 0.5;
      c.decisionThreshold = 0.2;
      c.directBias = 1.8;
      return c;
    }

    public static DriveConfig ultraAggressive()
    {
      DriveConfig c = new DriveConfig();
      c.maxVelocity = MetersPerSecond.of(5.0);
      c.aggressiveness = 2.0;
      c.smoothness = 0.1;
      c.goalBias = 3.0;
      c.baseAvoidanceStrength = 0.8;
      c.collisionUrgencyMultiplier = 1.5;
      c.softObstacleMultiplier = 0.2;
      c.dynamicObstacleMultiplier = 0.9;
      c.preferSaferPaths = false;
      // Intent-driven (very decisive)
      c.pathCommitment = 0.9;
      c.momentumPreference = 0.8;
      c.decisionThreshold = 0.4;
      c.directBias = 3.5;
      return c;
    }

    public static DriveConfig ultraSafe()
    {
      DriveConfig c = new DriveConfig();
      c.maxVelocity = MetersPerSecond.of(2.5);
      c.aggressiveness = 0.6;
      c.smoothness = 0.9;
      c.baseAvoidanceStrength = 2.5;
      c.defaultAvoidanceRadius = Meters.of(2.0);
      c.collisionUrgencyMultiplier = 5.0;
      c.dangerousObstacleMultiplier = 4.0;
      c.dynamicObstacleMultiplier = 2.0;
      c.pathSmoothingFactor = 0.6;
      // Intent-driven (less decisive, more cautious)
      c.pathCommitment = 0.4;
      c.momentumPreference = 0.3;
      c.decisionThreshold = 0.1;
      c.directBias = 1.0;
      return c;
    }
  }

  // ========================================================================
  // OBSTACLE FACTORY METHODS
  // ========================================================================

  public static Obstacle createCircle(Translation2d position, Measure<Distance> radius)
  {
    return new Obstacle(position, radius);
  }

  public static Obstacle createRectangle(Translation2d center, Measure<Distance> width, Measure<Distance> height)
  {
    Obstacle obs = new Obstacle(center, width.divide(2.0));
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = width;
    obs.height = height;
    return obs;
  }

  public static Obstacle createSquare(Translation2d center, Measure<Distance> sideLength)
  {
    return createRectangle(center, sideLength, sideLength);
  }

  public static Obstacle createRobot(Pose2d pose, Translation2d velocity, boolean isDefensive)
  {
    Obstacle obs = new Obstacle(pose.getTranslation(), Meters.of(0.5));
    obs.velocity = velocity;
    obs.shape = Obstacle.ObstacleShape.ROBOT;
    obs.width = Meters.of(0.8);
    obs.height = Meters.of(0.9);
    obs.type = Obstacle.ObstacleType.DYNAMIC;
    obs.priority = isDefensive ? 1.0 : 0.7;
    obs.avoidanceWeight = isDefensive ? 2.0 : 1.3;
    obs.isAggressive = isDefensive;
    return obs;
  }

  public static Obstacle createWall(Translation2d start, Translation2d end)
  {
    Translation2d center = start.plus(end).div(2.0);
    Translation2d diff = end.minus(start);
    Obstacle obs = new Obstacle(center, Meters.of(0.2));
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = Meters.of(diff.getNorm());
    obs.height = Meters.of(0.3);
    obs.type = Obstacle.ObstacleType.DANGEROUS;
    obs.priority = 1.0;
    obs.avoidanceWeight = 3.0;
    return obs;
  }

  public static Obstacle createBoundary(Translation2d position, Measure<Distance> size)
  {
    Obstacle obs = new Obstacle(position, size);
    obs.type = Obstacle.ObstacleType.DANGEROUS;
    obs.priority = 1.0;
    obs.avoidanceWeight = 5.0;
    return obs;
  }

  public static Obstacle createSoft(Translation2d position, Measure<Distance> radius)
  {
    Obstacle obs = new Obstacle(position, radius);
    obs.type = Obstacle.ObstacleType.SOFT;
    obs.priority = 0.3;
    obs.avoidanceWeight = 0.4;
    return obs;
  }

  public static Obstacle createZone(Translation2d center, Measure<Distance> radius)
  {
    Obstacle obs = new Obstacle(center, radius);
    obs.type = Obstacle.ObstacleType.ZONE;
    obs.priority = 0.5;
    obs.avoidanceWeight = 0.8;
    return obs;
  }

  public static Obstacle createEllipse(Translation2d center, Measure<Distance> radiusX, Measure<Distance> radiusY)
  {
    Obstacle obs = new Obstacle(center, radiusX);
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = radiusX.times(2.0);
    obs.height = radiusY.times(2.0);
    return obs;
  }

  public static Obstacle createLine(Translation2d start, Translation2d end, Measure<Distance> thickness)
  {
    Translation2d center = start.plus(end).div(2.0);
    Obstacle obs = new Obstacle(center, thickness.divide(2.0));
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = Meters.of(start.getDistance(end));
    obs.height = thickness;
    return obs;
  }

  public static Obstacle createAggressive(Pose2d pose, Translation2d velocity)
  {
    return createRobot(pose, velocity, true).asAggressive();
  }

  public static Obstacle createMoving(Translation2d pos, Translation2d vel, Translation2d dest, double confidence)
  {
    Obstacle obs = new Obstacle(pos, Meters.of(0.5));
    obs.velocity = vel;
    obs.type = Obstacle.ObstacleType.DYNAMIC;
    obs.likelyDestination = dest;
    obs.confidenceLevel = Math.max(0.0, Math.min(1.0, confidence));
    return obs;
  }

  public static Obstacle createTemporary(Translation2d position, Measure<Distance> radius, double timeToLive)
  {
    Obstacle obs = createSoft(position, radius);
    obs.timeToLive = timeToLive;
    return obs;
  }

  // ========================================================================
  // ENHANCED DRIVE METHOD WITH INTENT-DRIVEN BEHAVIOR
  // ========================================================================

  public ChassisSpeeds driveToPoseWithAvoidance(
      Pose2d currentPose,
      Pose2d targetPose,
      List<Obstacle> obstacles,
      PIDController rotationPID,
      DriveConfig config)
  {
    return driveToPoseWithAvoidance(currentPose, targetPose, obstacles, rotationPID, config, new Translation2d());
  }

  public ChassisSpeeds driveToPoseWithAvoidance(
      Pose2d currentPose,
      Pose2d targetPose,
      List<Obstacle> obstacles,
      PIDController rotationPID,
      DriveConfig config,
      Translation2d currentVel)
  {
    Translation2d currentPos = currentPose.getTranslation();
    Translation2d targetPos = targetPose.getTranslation();
    Translation2d vectorToGoal = targetPos.minus(currentPos);
    var distanceToGoal = Meters.of(vectorToGoal.getNorm());

    boolean inPrecisionZone = distanceToGoal.lt(config.precisionDistance);
    boolean inSlowZone = distanceToGoal.lt(config.slowZoneDistance);

    Translation2d desiredDirection = distanceToGoal.gt(Meters.of(0.01))
        ? vectorToGoal.div(distanceToGoal.in(Meters))
        : new Translation2d();

    // Process obstacles
    Translation2d avoidanceVector = new Translation2d();
    double closestObstacle = Double.POSITIVE_INFINITY;
    double angularAvoidance = 0.0;
    double minTimeToCollision = Double.POSITIVE_INFINITY;
    boolean clearPath = true;  // NEW: Track if path is clear

    for (Obstacle obstacle : obstacles)
    {
      Translation2d obstaclePos = obstacle.getPredictedPosition(config.predictionLookAhead);

      Measure<Distance> effectiveRadius = config.useVelocityAwareAvoidance
          ? obstacle.getEffectiveRadius(currentVel).plus(config.defaultAvoidanceRadius)
          : obstacle.radius.plus(config.defaultAvoidanceRadius);

      Translation2d toObstacle = currentPos.minus(obstaclePos);
      var obstacleDistance = Meters.of(toObstacle.getNorm());

      if (obstacleDistance.in(Meters) < closestObstacle)
        closestObstacle = obstacleDistance.in(Meters);

      if (config.useCollisionPrediction)
      {
        double ttc = obstacle.getTimeToCollision(currentPos, currentVel);
        if (ttc < minTimeToCollision) minTimeToCollision = ttc;
      }

      if (obstacleDistance.gt(effectiveRadius)) continue;

      clearPath = false;  // Obstacle in range

      if (obstacleDistance.gt(Meters.of(0.01)))
      {
        double typeMultiplier = switch (obstacle.type)
        {
          case STATIC -> config.staticObstacleMultiplier;
          case DYNAMIC -> config.dynamicObstacleMultiplier;
          case DANGEROUS -> config.dangerousObstacleMultiplier;
          case SOFT -> config.softObstacleMultiplier;
          case ZONE -> config.zoneObstacleMultiplier;
        };

        if (obstacle.isAggressive) typeMultiplier *= config.aggressiveObstacleMultiplier;

        // Apply difficulty scaling
        typeMultiplier *= (1.0 + obstacle.difficultyScale * 0.3);

        double normalizedDist = obstacleDistance.in(Meters) / effectiveRadius.in(Meters);
        double repulsionMag = config.baseAvoidanceStrength * obstacle.avoidanceWeight * typeMultiplier *
            Math.pow(1.0 - normalizedDist, 2.5);

        if (config.respectObstaclePriority) repulsionMag *= obstacle.priority;
        if (obstacleDistance.lt(config.dangerRadius.plus(obstacle.radius))) repulsionMag *= 4.0;

        if (config.useCollisionPrediction && minTimeToCollision < 2.0)
        {
          repulsionMag *= 1.0 + config.collisionUrgencyMultiplier * (2.0 - minTimeToCollision) / 2.0;
        }

        Translation2d normalizedToObstacle = toObstacle.div(obstacleDistance.in(Meters));
        double headingToward = -desiredDirection.getX() * normalizedToObstacle.getX() -
                               desiredDirection.getY() * normalizedToObstacle.getY();
        if (headingToward > 0) repulsionMag *= (1.0 + headingToward * 2.5);

        if (config.useVelocityAwareAvoidance && obstacle.velocity.getNorm() > 0.1)
        {
          Translation2d relativeVel = obstacle.velocity.minus(currentVel);
          double approachSpeed = -(relativeVel.getX() * normalizedToObstacle.getX() +
                                   relativeVel.getY() * normalizedToObstacle.getY());
          if (approachSpeed > 0) repulsionMag *= (1.0 + approachSpeed * 0.8);
        }

        avoidanceVector = avoidanceVector.plus(toObstacle.div(obstacleDistance.in(Meters)).times(repulsionMag));

        if (config.enableAngularAvoidance && obstacleDistance.lt(Meters.of(1.2)))
        {
          double angleToObstacle = Math.atan2(toObstacle.getY(), toObstacle.getX());
          double angleDiff = angleToObstacle - currentPose.getRotation().getRadians();
          while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
          while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
          angularAvoidance += Math.signum(angleDiff) * repulsionMag * 0.4;
        }
      }
    }

    // ========================================================================
    // INTENT-DRIVEN BEHAVIOR: Path commitment and momentum
    // ========================================================================

    Translation2d rawCombinedDirection;

    if (clearPath)
    {
      // Clear path: Go direct with strong bias
      rawCombinedDirection = desiredDirection.times(config.directBias);
    }
    else
    {
      // Obstacles present: Combine goal direction with avoidance
      rawCombinedDirection = desiredDirection.times(config.goalBias).plus(avoidanceVector);
    }

    double combinedMag = rawCombinedDirection.getNorm();
    if (combinedMag > 0.01)
    {
      rawCombinedDirection = rawCombinedDirection.div(combinedMag);
    }
    else
    {
      rawCombinedDirection = desiredDirection;
    }

    // Apply momentum preference (continue in current direction when reasonable)
    Translation2d combinedDirection;
    if (previousDirection.getNorm() > 0.01 && currentVel.getNorm() > 0.1)
    {
      // Check if new direction is significantly different
      double directionChange = Math.acos(Math.max(-1.0, Math.min(1.0,
          rawCombinedDirection.getX() * previousDirection.getX() +
          rawCombinedDirection.getY() * previousDirection.getY()
      )));

      if (directionChange < config.decisionThreshold && pathCommitmentTimer > 0)
      {
        // Minor change - stick with committed direction
        combinedDirection = committedDirection;
        pathCommitmentTimer -= 0.02;  // Decay timer
      }
      else if (directionChange > config.decisionThreshold)
      {
        // Significant change - commit to new direction
        combinedDirection = rawCombinedDirection.times(1.0 - config.momentumPreference)
                          .plus(previousDirection.times(config.momentumPreference));
        double mag = combinedDirection.getNorm();
        if (mag > 0.01) combinedDirection = combinedDirection.div(mag);

        committedDirection = combinedDirection;
        pathCommitmentTimer = PATH_COMMITMENT_TIME * config.pathCommitment;
      }
      else
      {
        // Blend with momentum
        combinedDirection = rawCombinedDirection.times(1.0 - config.momentumPreference * 0.5)
                          .plus(previousDirection.times(config.momentumPreference * 0.5));
        double mag = combinedDirection.getNorm();
        if (mag > 0.01) combinedDirection = combinedDirection.div(mag);
      }
    }
    else
    {
      // No previous direction - use raw
      combinedDirection = rawCombinedDirection;
      committedDirection = combinedDirection;
      pathCommitmentTimer = PATH_COMMITMENT_TIME * config.pathCommitment;
    }

    // Path smoothing (reduced influence when committed)
    if (config.enablePathSmoothing && previousDirection.getNorm() > 0.01)
    {
      double smoothingFactor = config.pathSmoothingFactor * (1.0 - config.pathCommitment * 0.5);
      combinedDirection = combinedDirection.times(1.0 - smoothingFactor)
          .plus(previousDirection.times(smoothingFactor));

      double smoothedMag = combinedDirection.getNorm();
      if (smoothedMag > 0.01) combinedDirection = combinedDirection.div(smoothedMag);
    }

    previousDirection = combinedDirection;

    // Calculate speed
    double desiredSpeed;
    if (inPrecisionZone)
    {
      desiredSpeed = distanceToGoal.in(Meters) * 2.5 * config.aggressiveness;
    }
    else if (inSlowZone)
    {
      double slowFactor = (distanceToGoal.in(Meters) - config.precisionDistance.in(Meters)) /
                         (config.slowZoneDistance.in(Meters) - config.precisionDistance.in(Meters));
      desiredSpeed = config.minVelocity.in(MetersPerSecond) +
          (config.maxVelocity.in(MetersPerSecond) - config.minVelocity.in(MetersPerSecond)) *
          Math.pow(slowFactor, 0.7) * config.aggressiveness;
    }
    else
    {
      desiredSpeed = config.maxVelocity.in(MetersPerSecond) * config.aggressiveness;
    }

    // Adaptive speed
    if (config.useAdaptiveSpeed && !obstacles.isEmpty())
    {
      if (closestObstacle < config.defaultAvoidanceRadius.in(Meters))
      {
        desiredSpeed *= 0.35 + 0.65 * Math.pow(closestObstacle / config.defaultAvoidanceRadius.in(Meters), 0.5);
      }
      if (config.useCollisionPrediction && minTimeToCollision < 1.5)
      {
        desiredSpeed *= 0.4 + 0.6 * (minTimeToCollision / 1.5);
      }
    }

    desiredSpeed = Math.max(config.minVelocity.in(MetersPerSecond),
                           Math.min(config.maxVelocity.in(MetersPerSecond), desiredSpeed));
    desiredSpeed *= (1.0 - config.smoothness);

    var vx = MetersPerSecond.of(combinedDirection.getX() * desiredSpeed);
    var vy = MetersPerSecond.of(combinedDirection.getY() * desiredSpeed);

    // Rotation
    double baseOmega = rotationPID.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians()
    );
    double totalOmega = baseOmega + Math.max(-1.5, Math.min(1.5, angularAvoidance));
    var omega = RadiansPerSecond.of(totalOmega);

    if (omega.gt(config.maxAngularVelocity)) omega = config.maxAngularVelocity;
    else if (omega.lt(config.maxAngularVelocity.times(-1))) omega = config.maxAngularVelocity.times(-1);

    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
  }

  public void resetSmoothing()
  {
    previousDirection = new Translation2d();
    committedDirection = new Translation2d();
    pathCommitmentTimer = 0;
  }
}
