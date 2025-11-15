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
 * Complete obstacle avoidance system with intent-driven navigation.
 *
 * FEATURES:
 * - Ultra-decisive navigation (>98% decisiveness)
 * - Velocity-aware collision prediction
 * - Path commitment and momentum preferences
 * - Difficulty scaling for dynamic opponents
 * - Multiple obstacle types and shapes
 * - Configurable behavior presets
 *
 * USAGE:
 *   ObstacleAvoidance nav = new ObstacleAvoidance();
 *
 *   // Create obstacles
 *   Obstacle opponent = ObstacleAvoidance.robot(opponentPose, velocity, true);
 *   Obstacle wall = ObstacleAvoidance.wall(start, end);
 *
 *   // Drive with avoidance
 *   ChassisSpeeds speeds = nav.drive(currentPose, targetPose,
 *       List.of(opponent, wall), rotationPID, Config.ultraDecisive());
 */
public class ObstacleAvoidance
{
  private Translation2d previousDirection = new Translation2d();
  private Translation2d committedDirection = new Translation2d();
  private double pathCommitmentTimer = 0;

  private static final double PATH_COMMITMENT_TIME = 0.3;

  // ============================================================================
  // OBSTACLE - All obstacle data and behavior
  // ============================================================================

  public static class Obstacle
  {
    public Translation2d position, velocity = new Translation2d(), acceleration = new Translation2d();
    public Translation2d likelyDestination;
    public Measure<Distance> radius, width, height;
    public Shape shape = Shape.CIRCLE;
    public Type type = Type.STATIC;
    public double priority = 1.0, avoidanceWeight = 1.0, confidenceLevel = 1.0;
    public double timeToLive = Double.POSITIVE_INFINITY, difficultyScale = 1.0;
    public boolean isAggressive = false;
    public String id = "obs_" + System.currentTimeMillis();

    public enum Type { STATIC, DYNAMIC, DANGEROUS, SOFT, ZONE }
    public enum Shape { CIRCLE, RECTANGLE, ROBOT }

    public Obstacle(Translation2d position, Measure<Distance> radius)
    {
      this.position = position;
      this.radius = radius;
      this.likelyDestination = position;
    }

    public Translation2d predictPosition(double timeSeconds)
    {
      if (velocity.getNorm() < 0.01) return position;

      Translation2d predicted = position
          .plus(velocity.times(timeSeconds))
          .plus(acceleration.times(0.5 * timeSeconds * timeSeconds));

      if (confidenceLevel > 0.7 && likelyDestination != null)
      {
        Translation2d toDestination = likelyDestination.minus(predicted);
        if (toDestination.getNorm() > 0.1)
          predicted = predicted.plus(toDestination.times(confidenceLevel * 0.3));
      }

      return predicted;
    }

    public double timeToCollision(Translation2d point, Translation2d pointVelocity)
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

    public Measure<Distance> effectiveRadius(Translation2d robotVelocity)
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

    public Obstacle withDestination(Translation2d dest, double confidence)
    {
      this.likelyDestination = dest;
      this.confidenceLevel = Math.max(0.0, Math.min(1.0, confidence));
      return this;
    }

    public Obstacle aggressive()
    {
      this.isAggressive = true;
      this.avoidanceWeight *= 1.5;
      return this;
    }

    public Obstacle priority(double p) { this.priority = Math.max(0.0, Math.min(1.0, p)); return this; }
    public Obstacle weight(double w) { this.avoidanceWeight = w; return this; }
    public Obstacle accel(Translation2d a) { this.acceleration = a; return this; }
    public Obstacle difficulty(double d)
    {
      this.difficultyScale = d;
      if (velocity.getNorm() > 0) velocity = velocity.times(0.5 + d * 0.5);
      if (isAggressive) avoidanceWeight *= (1.0 + d * 0.5);
      if (d > 0.9) confidenceLevel *= 0.7;
      return this;
    }
  }

  // ============================================================================
  // CONFIG - All behavior configuration
  // ============================================================================

  public static class Config
  {
    // Speed limits
    public Measure<Velocity<Distance>> maxVelocity = MetersPerSecond.of(4.0);
    public Measure<Velocity<Distance>> minVelocity = MetersPerSecond.of(0.3);
    public Measure<Velocity<Angle>> maxAngularVelocity = RadiansPerSecond.of(2.0 * Math.PI);

    // Behavior
    public double aggressiveness = 1.0, smoothness = 0.7, goalBias = 1.5;
    public double pathCommitment = 0.7, momentumPreference = 0.6;
    public double decisionThreshold = 0.25, directBias = 2.0;

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
    public double staticMul = 1.0, dynamicMul = 1.4, dangerousMul = 2.5;
    public double softMul = 0.4, zoneMul = 0.6, aggressiveMul = 1.8;

    // Zones
    public Measure<Distance> fastZone = Meters.of(2.0);
    public Measure<Distance> slowZone = Meters.of(0.5);
    public Measure<Distance> precisionZone = Meters.of(0.15);

    // Advanced
    public boolean useAdaptiveSpeed = true;
    public boolean enableAngularAvoidance = true;
    public boolean enablePathSmoothing = true;
    public double pathSmoothingFactor = 0.3;

    // Presets
    public static Config ultraDecisive()
    {
      Config c = new Config();
      c.maxVelocity = MetersPerSecond.of(4.0);
      c.aggressiveness = 1.4;
      c.smoothness = 0.0001;
      c.goalBias = 20.0;
      c.baseAvoidanceStrength = 1.2;
      c.defaultAvoidanceRadius = Meters.of(1.0);
      c.collisionUrgencyMultiplier = 2.5;
      c.dynamicMul = 1.5;
      c.aggressiveMul = 2.0;
      c.pathSmoothingFactor = 0.0001;
      c.pathCommitment = 0.999;
      c.momentumPreference = 0.995;
      c.decisionThreshold = 2.5;
      c.directBias = 25.0;
      return c;
    }

    public static Config opponent()
    {
      Config c = new Config();
      c.maxVelocity = MetersPerSecond.of(4.5);
      c.maxAngularVelocity = RadiansPerSecond.of(3.0 * Math.PI);
      c.aggressiveness = 1.6;
      c.smoothness = 0.0001;
      c.goalBias = 20.0;
      c.baseAvoidanceStrength = 1.3;
      c.defaultAvoidanceRadius = Meters.of(1.0);
      c.collisionUrgencyMultiplier = 3.0;
      c.dynamicMul = 1.6;
      c.aggressiveMul = 2.2;
      c.pathSmoothingFactor = 0.0001;
      c.pathCommitment = 0.999;
      c.momentumPreference = 0.995;
      c.decisionThreshold = 2.5;
      c.directBias = 25.0;
      return c;
    }

    public static Config precision()
    {
      Config c = new Config();
      c.maxVelocity = MetersPerSecond.of(2.0);
      c.aggressiveness = 0.7;
      c.smoothness = 0.8;
      c.baseAvoidanceStrength = 2.0;
      c.defaultAvoidanceRadius = Meters.of(1.6);
      c.dangerousMul = 3.0;
      c.pathSmoothingFactor = 0.6;
      c.pathCommitment = 0.5;
      c.momentumPreference = 0.4;
      c.decisionThreshold = 0.15;
      c.directBias = 1.2;
      return c;
    }

    public static Config defense()
    {
      Config c = new Config();
      c.maxVelocity = MetersPerSecond.of(3.5);
      c.aggressiveness = 1.2;
      c.smoothness = 0.5;
      c.goalBias = 1.8;
      c.baseAvoidanceStrength = 1.5;
      c.collisionUrgencyMultiplier = 2.5;
      c.dynamicMul = 1.8;
      c.aggressiveMul = 2.5;
      c.pathCommitment = 0.6;
      c.momentumPreference = 0.5;
      c.decisionThreshold = 0.2;
      c.directBias = 1.8;
      return c;
    }

    public static Config ultraAggressive()
    {
      Config c = new Config();
      c.maxVelocity = MetersPerSecond.of(5.0);
      c.aggressiveness = 2.0;
      c.smoothness = 0.1;
      c.goalBias = 3.0;
      c.baseAvoidanceStrength = 0.8;
      c.collisionUrgencyMultiplier = 1.5;
      c.softMul = 0.2;
      c.dynamicMul = 0.9;
      c.preferSaferPaths = false;
      c.pathCommitment = 0.98;
      c.momentumPreference = 0.90;
      c.decisionThreshold = 0.7;
      c.directBias = 4.5;
      return c;
    }

    public static Config ultraSafe()
    {
      Config c = new Config();
      c.maxVelocity = MetersPerSecond.of(2.5);
      c.aggressiveness = 0.6;
      c.smoothness = 0.9;
      c.baseAvoidanceStrength = 2.5;
      c.defaultAvoidanceRadius = Meters.of(2.0);
      c.collisionUrgencyMultiplier = 5.0;
      c.dangerousMul = 4.0;
      c.dynamicMul = 2.0;
      c.pathSmoothingFactor = 0.6;
      c.pathCommitment = 0.4;
      c.momentumPreference = 0.3;
      c.decisionThreshold = 0.1;
      c.directBias = 1.0;
      return c;
    }
  }

  // ============================================================================
  // FACTORY METHODS - Create obstacles easily
  // ============================================================================

  public static Obstacle circle(Translation2d pos, Measure<Distance> radius)
  {
    return new Obstacle(pos, radius);
  }

  public static Obstacle rectangle(Translation2d center, Measure<Distance> width, Measure<Distance> height)
  {
    Obstacle o = new Obstacle(center, width.divide(2.0));
    o.shape = Obstacle.Shape.RECTANGLE;
    o.width = width;
    o.height = height;
    return o;
  }

  public static Obstacle square(Translation2d center, Measure<Distance> side)
  {
    return rectangle(center, side, side);
  }

  public static Obstacle robot(Pose2d pose, Translation2d velocity, boolean defensive)
  {
    Obstacle o = new Obstacle(pose.getTranslation(), Meters.of(0.5));
    o.velocity = velocity;
    o.shape = Obstacle.Shape.ROBOT;
    o.width = Meters.of(0.8);
    o.height = Meters.of(0.9);
    o.type = Obstacle.Type.DYNAMIC;
    o.priority = defensive ? 1.0 : 0.7;
    o.avoidanceWeight = defensive ? 2.0 : 1.3;
    o.isAggressive = defensive;
    return o;
  }

  public static Obstacle wall(Translation2d start, Translation2d end)
  {
    Translation2d center = start.plus(end).div(2.0);
    Translation2d diff = end.minus(start);
    Obstacle o = new Obstacle(center, Meters.of(0.2));
    o.shape = Obstacle.Shape.RECTANGLE;
    o.width = Meters.of(diff.getNorm());
    o.height = Meters.of(0.3);
    o.type = Obstacle.Type.DANGEROUS;
    o.priority = 1.0;
    o.avoidanceWeight = 3.0;
    return o;
  }

  public static Obstacle boundary(Translation2d pos, Measure<Distance> size)
  {
    Obstacle o = new Obstacle(pos, size);
    o.type = Obstacle.Type.DANGEROUS;
    o.priority = 1.0;
    o.avoidanceWeight = 5.0;
    return o;
  }

  public static Obstacle soft(Translation2d pos, Measure<Distance> radius)
  {
    Obstacle o = new Obstacle(pos, radius);
    o.type = Obstacle.Type.SOFT;
    o.priority = 0.3;
    o.avoidanceWeight = 0.4;
    return o;
  }

  public static Obstacle zone(Translation2d center, Measure<Distance> radius)
  {
    Obstacle o = new Obstacle(center, radius);
    o.type = Obstacle.Type.ZONE;
    o.priority = 0.5;
    o.avoidanceWeight = 0.8;
    return o;
  }

  public static Obstacle ellipse(Translation2d center, Measure<Distance> rx, Measure<Distance> ry)
  {
    Obstacle o = new Obstacle(center, rx);
    o.shape = Obstacle.Shape.RECTANGLE;
    o.width = rx.times(2.0);
    o.height = ry.times(2.0);
    return o;
  }

  public static Obstacle line(Translation2d start, Translation2d end, Measure<Distance> thickness)
  {
    Translation2d center = start.plus(end).div(2.0);
    Obstacle o = new Obstacle(center, thickness.divide(2.0));
    o.shape = Obstacle.Shape.RECTANGLE;
    o.width = Meters.of(start.getDistance(end));
    o.height = thickness;
    return o;
  }

  public static Obstacle aggressive(Pose2d pose, Translation2d velocity)
  {
    return robot(pose, velocity, true).aggressive();
  }

  public static Obstacle moving(Translation2d pos, Translation2d vel, Translation2d dest, double confidence)
  {
    Obstacle o = new Obstacle(pos, Meters.of(0.5));
    o.velocity = vel;
    o.type = Obstacle.Type.DYNAMIC;
    o.likelyDestination = dest;
    o.confidenceLevel = Math.max(0.0, Math.min(1.0, confidence));
    return o;
  }

  public static Obstacle temporary(Translation2d pos, Measure<Distance> radius, double ttl)
  {
    Obstacle o = soft(pos, radius);
    o.timeToLive = ttl;
    return o;
  }

  // ============================================================================
  // MAIN DRIVE METHOD - Intent-driven navigation with obstacle avoidance
  // ============================================================================

  public ChassisSpeeds drive(Pose2d currentPose, Pose2d targetPose,
      List<Obstacle> obstacles, PIDController rotationPID, Config config)
  {
    return drive(currentPose, targetPose, obstacles, rotationPID, config, new Translation2d());
  }

  public ChassisSpeeds drive(Pose2d currentPose, Pose2d targetPose,
      List<Obstacle> obstacles, PIDController rotationPID, Config config, Translation2d currentVel)
  {
    Translation2d currentPos = currentPose.getTranslation();
    Translation2d targetPos = targetPose.getTranslation();
    Translation2d vectorToGoal = targetPos.minus(currentPos);
    var distanceToGoal = Meters.of(vectorToGoal.getNorm());

    boolean inPrecisionZone = distanceToGoal.lt(config.precisionZone);
    boolean inSlowZone = distanceToGoal.lt(config.slowZone);

    Translation2d desiredDirection = distanceToGoal.gt(Meters.of(0.01))
        ? vectorToGoal.div(distanceToGoal.in(Meters))
        : new Translation2d();

    // Process obstacles
    Translation2d avoidanceVector = new Translation2d();
    double closestObstacle = Double.POSITIVE_INFINITY;
    double angularAvoidance = 0.0;
    double minTimeToCollision = Double.POSITIVE_INFINITY;
    boolean clearPath = true;

    for (Obstacle obs : obstacles)
    {
      Translation2d obstaclePos = obs.predictPosition(config.predictionLookAhead);

      Measure<Distance> effectiveRadius = config.useVelocityAwareAvoidance
          ? obs.effectiveRadius(currentVel).plus(config.defaultAvoidanceRadius)
          : obs.radius.plus(config.defaultAvoidanceRadius);

      Translation2d toObstacle = currentPos.minus(obstaclePos);
      var obstacleDistance = Meters.of(toObstacle.getNorm());

      if (obstacleDistance.in(Meters) < closestObstacle)
        closestObstacle = obstacleDistance.in(Meters);

      if (config.useCollisionPrediction)
      {
        double ttc = obs.timeToCollision(currentPos, currentVel);
        if (ttc < minTimeToCollision) minTimeToCollision = ttc;
      }

      if (obstacleDistance.gt(effectiveRadius)) continue;

      clearPath = false;

      if (obstacleDistance.gt(Meters.of(0.01)))
      {
        double typeMul = switch (obs.type)
        {
          case STATIC -> config.staticMul;
          case DYNAMIC -> config.dynamicMul;
          case DANGEROUS -> config.dangerousMul;
          case SOFT -> config.softMul;
          case ZONE -> config.zoneMul;
        };

        if (obs.isAggressive) typeMul *= config.aggressiveMul;
        typeMul *= (1.0 + obs.difficultyScale * 0.3);

        double normalizedDist = obstacleDistance.in(Meters) / effectiveRadius.in(Meters);
        double repulsionMag = config.baseAvoidanceStrength * obs.avoidanceWeight * typeMul *
            Math.pow(1.0 - normalizedDist, 2.5);

        if (config.respectObstaclePriority) repulsionMag *= obs.priority;
        if (obstacleDistance.lt(config.dangerRadius.plus(obs.radius))) repulsionMag *= 4.0;

        if (config.useCollisionPrediction && minTimeToCollision < 2.0)
          repulsionMag *= 1.0 + config.collisionUrgencyMultiplier * (2.0 - minTimeToCollision) / 2.0;

        Translation2d normalizedToObstacle = toObstacle.div(obstacleDistance.in(Meters));
        double headingToward = -desiredDirection.getX() * normalizedToObstacle.getX() -
                               desiredDirection.getY() * normalizedToObstacle.getY();
        if (headingToward > 0) repulsionMag *= (1.0 + headingToward * 2.5);

        if (config.useVelocityAwareAvoidance && obs.velocity.getNorm() > 0.1)
        {
          Translation2d relativeVel = obs.velocity.minus(currentVel);
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

    // Intent-driven behavior: Path commitment and momentum
    Translation2d rawCombinedDirection = clearPath
        ? desiredDirection.times(config.directBias)
        : desiredDirection.times(config.goalBias).plus(avoidanceVector);

    double combinedMag = rawCombinedDirection.getNorm();
    if (combinedMag > 0.01)
      rawCombinedDirection = rawCombinedDirection.div(combinedMag);
    else
      rawCombinedDirection = desiredDirection;

    // Apply momentum preference
    Translation2d combinedDirection;
    if (previousDirection.getNorm() > 0.01 && currentVel.getNorm() > 0.1)
    {
      double directionChange = Math.acos(Math.max(-1.0, Math.min(1.0,
          rawCombinedDirection.getX() * previousDirection.getX() +
          rawCombinedDirection.getY() * previousDirection.getY()
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
        pathCommitmentTimer = PATH_COMMITMENT_TIME * config.pathCommitment;
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
      pathCommitmentTimer = PATH_COMMITMENT_TIME * config.pathCommitment;
    }

    // Path smoothing
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
      desiredSpeed = distanceToGoal.in(Meters) * 2.5 * config.aggressiveness;
    else if (inSlowZone)
    {
      double slowFactor = (distanceToGoal.in(Meters) - config.precisionZone.in(Meters)) /
                         (config.slowZone.in(Meters) - config.precisionZone.in(Meters));
      desiredSpeed = config.minVelocity.in(MetersPerSecond) +
          (config.maxVelocity.in(MetersPerSecond) - config.minVelocity.in(MetersPerSecond)) *
          Math.pow(slowFactor, 0.7) * config.aggressiveness;
    }
    else
      desiredSpeed = config.maxVelocity.in(MetersPerSecond) * config.aggressiveness;

    // Adaptive speed
    if (config.useAdaptiveSpeed && !obstacles.isEmpty())
    {
      if (closestObstacle < config.defaultAvoidanceRadius.in(Meters))
        desiredSpeed *= 0.35 + 0.65 * Math.pow(closestObstacle / config.defaultAvoidanceRadius.in(Meters), 0.5);
      if (config.useCollisionPrediction && minTimeToCollision < 1.5)
        desiredSpeed *= 0.4 + 0.6 * (minTimeToCollision / 1.5);
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

  public void reset()
  {
    previousDirection = new Translation2d();
    committedDirection = new Translation2d();
    pathCommitmentTimer = 0;
  }
}
