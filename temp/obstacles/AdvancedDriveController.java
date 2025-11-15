package obstacles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

import java.util.List;

/**
 * Advanced drive controller with intelligent obstacle avoidance.
 *
 * Features:
 * - Collision prediction using time-to-collision
 * - Velocity-aware avoidance (faster obstacles get larger buffers)
 * - Smart prediction with destination biasing
 * - Path smoothing to eliminate jitter
 * - Priority-based obstacle handling
 * - Adaptive speed based on obstacle proximity
 * - Angular avoidance for rotation control
 *
 * Usage:
 * <pre>
 * AdvancedDriveController controller = new AdvancedDriveController();
 * List<Obstacle> obstacles = buildObstacles();
 * ChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
 *     currentPose, targetPose, obstacles, rotationPID, config
 * );
 * swerveDrive.drive(speeds);
 * </pre>
 */
public class AdvancedDriveController
{
  // Field for path smoothing (store previous direction)
  private Translation2d previousDirection = new Translation2d();

  /**
   * Enhanced drive-to-pose with intelligent obstacle avoidance.
   *
   * @param currentPose Robot's current pose
   * @param targetPose Desired target pose
   * @param obstacles List of obstacles to avoid
   * @param rotationPID PID controller for rotation
   * @param config Drive configuration
   * @return Chassis speeds to apply
   */
  public ChassisSpeeds driveToPoseWithAvoidance(
      Pose2d          currentPose,
      Pose2d          targetPose,
      List<Obstacle>  obstacles,
      PIDController   rotationPID,
      DriveConfig     config)
  {
    return driveToPoseWithAvoidance(
        currentPose, targetPose, obstacles, rotationPID, config, new Translation2d()
    );
  }

  /**
   * Enhanced drive-to-pose with intelligent obstacle avoidance.
   *
   * @param currentPose Robot's current pose
   * @param targetPose Desired target pose
   * @param obstacles List of obstacles to avoid
   * @param rotationPID PID controller for rotation
   * @param config Drive configuration
   * @param currentVel Current velocity (for better prediction)
   * @return Chassis speeds to apply
   */
  public ChassisSpeeds driveToPoseWithAvoidance(
      Pose2d          currentPose,
      Pose2d          targetPose,
      List<Obstacle>  obstacles,
      PIDController   rotationPID,
      DriveConfig     config,
      Translation2d   currentVel)
  {
    Translation2d currentPos = currentPose.getTranslation();
    Translation2d targetPos  = targetPose.getTranslation();

    // Calculate distance and zones
    Translation2d vectorToGoal   = targetPos.minus(currentPos);
    var           distanceToGoal = Meters.of(vectorToGoal.getNorm());

    boolean inFastZone      = distanceToGoal.gt(config.fastZoneDistance);
    boolean inSlowZone      = distanceToGoal.lt(config.slowZoneDistance);
    boolean inPrecisionZone = distanceToGoal.lt(config.precisionDistance);

    // Base direction to goal
    Translation2d desiredDirection = distanceToGoal.gt(Meters.of(0.01))
        ? vectorToGoal.div(distanceToGoal.in(Meters))
        : new Translation2d();

    // ========================================================================
    // Process obstacles with advanced features
    // ========================================================================

    Translation2d avoidanceVector = new Translation2d();
    double closestObstacle = Double.POSITIVE_INFINITY;
    double angularAvoidance = 0.0;
    double minTimeToCollision = Double.POSITIVE_INFINITY;

    for (Obstacle obstacle : obstacles)
    {
      // Get predicted position
      Translation2d obstaclePos = obstacle.getPredictedPosition(config.predictionLookAhead);

      // Get velocity-aware radius
      Measure<Distance> effectiveRadius = config.useVelocityAwareAvoidance
          ? obstacle.getEffectiveRadius(currentVel).plus(config.defaultAvoidanceRadius)
          : obstacle.radius.plus(config.defaultAvoidanceRadius);

      Translation2d toObstacle = currentPos.minus(obstaclePos);
      var obstacleDistance = Meters.of(toObstacle.getNorm());

      // Track closest
      if (obstacleDistance.in(Meters) < closestObstacle)
      {
        closestObstacle = obstacleDistance.in(Meters);
      }

      // Calculate time to collision
      if (config.useCollisionPrediction)
      {
        double ttc = obstacle.getTimeToCollision(currentPos, currentVel);
        if (ttc < minTimeToCollision)
        {
          minTimeToCollision = ttc;
        }
      }

      // Skip if too far
      if (obstacleDistance.gt(effectiveRadius))
      {
        continue;
      }

      if (obstacleDistance.gt(Meters.of(0.01)))
      {
        // Get type multiplier
        double typeMultiplier = switch (obstacle.type)
        {
          case STATIC    -> config.staticObstacleMultiplier;
          case DYNAMIC   -> config.dynamicObstacleMultiplier;
          case DANGEROUS -> config.dangerousObstacleMultiplier;
          case SOFT      -> config.softObstacleMultiplier;
          case ZONE      -> config.zoneObstacleMultiplier;
        };

        // Apply aggressive multiplier
        if (obstacle.isAggressive)
        {
          typeMultiplier *= config.aggressiveObstacleMultiplier;
        }

        // Base repulsion (inverse square with smoother falloff)
        double normalizedDist = obstacleDistance.in(Meters) / effectiveRadius.in(Meters);
        double repulsionMag = config.baseAvoidanceStrength *
            obstacle.avoidanceWeight *
            typeMultiplier *
            Math.pow(1.0 - normalizedDist, 2.5);  // Smoother falloff

        // Apply priority
        if (config.respectObstaclePriority)
        {
          repulsionMag *= obstacle.priority;
        }

        // Emergency boost in danger zone
        if (obstacleDistance.lt(config.dangerRadius.plus(obstacle.radius)))
        {
          repulsionMag *= 4.0;
        }

        // Collision urgency boost
        if (config.useCollisionPrediction && minTimeToCollision < 2.0)
        {
          double urgencyFactor = 1.0 + config.collisionUrgencyMultiplier * (2.0 - minTimeToCollision) / 2.0;
          repulsionMag *= urgencyFactor;
        }

        // Predictive: heading toward obstacle?
        Translation2d normalizedToObstacle = toObstacle.div(obstacleDistance.in(Meters));
        double headingTowardObstacle = -desiredDirection.getX() * normalizedToObstacle.getX() -
                                       desiredDirection.getY() * normalizedToObstacle.getY();

        if (headingTowardObstacle > 0)
        {
          repulsionMag *= (1.0 + headingTowardObstacle * 2.5);
        }

        // Velocity-based avoidance
        if (config.useVelocityAwareAvoidance && obstacle.velocity.getNorm() > 0.1)
        {
          Translation2d relativeVel = obstacle.velocity.minus(currentVel);
          double approachSpeed = -(relativeVel.getX() * normalizedToObstacle.getX() +
                                   relativeVel.getY() * normalizedToObstacle.getY());

          if (approachSpeed > 0)
          {
            repulsionMag *= (1.0 + approachSpeed * 0.8);
          }
        }

        // Apply repulsion
        Translation2d repulsion = toObstacle.div(obstacleDistance.in(Meters)).times(repulsionMag);
        avoidanceVector = avoidanceVector.plus(repulsion);

        // Angular avoidance
        if (config.enableAngularAvoidance && obstacleDistance.lt(Meters.of(1.2)))
        {
          double angleToObstacle = Math.atan2(toObstacle.getY(), toObstacle.getX());
          double currentHeading = currentPose.getRotation().getRadians();
          double angleDiff = angleToObstacle - currentHeading;

          while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
          while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

          angularAvoidance += Math.signum(angleDiff) * repulsionMag * 0.4;
        }
      }
    }

    // ========================================================================
    // Combine direction with avoidance
    // ========================================================================

    Translation2d combinedDirection = desiredDirection.times(config.goalBias).plus(avoidanceVector);
    double combinedMagnitude = combinedDirection.getNorm();

    if (combinedMagnitude > 0.01)
    {
      combinedDirection = combinedDirection.div(combinedMagnitude);
    }
    else
    {
      combinedDirection = desiredDirection;
    }

    // Path smoothing (blend with previous direction)
    if (config.enablePathSmoothing && previousDirection.getNorm() > 0.01)
    {
      combinedDirection = combinedDirection.times(1.0 - config.pathSmoothingFactor)
          .plus(previousDirection.times(config.pathSmoothingFactor));

      double smoothedMag = combinedDirection.getNorm();
      if (smoothedMag > 0.01)
      {
        combinedDirection = combinedDirection.div(smoothedMag);
      }
    }

    previousDirection = combinedDirection;

    // ========================================================================
    // Calculate speed
    // ========================================================================

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

    // Adaptive speed based on obstacles
    if (config.useAdaptiveSpeed && !obstacles.isEmpty())
    {
      // Slow down based on closest obstacle
      if (closestObstacle < config.defaultAvoidanceRadius.in(Meters))
      {
        double obstacleFactor = closestObstacle / config.defaultAvoidanceRadius.in(Meters);
        desiredSpeed *= (0.35 + 0.65 * Math.pow(obstacleFactor, 0.5));
      }

      // Slow down if collision imminent
      if (config.useCollisionPrediction && minTimeToCollision < 1.5)
      {
        double collisionFactor = minTimeToCollision / 1.5;
        desiredSpeed *= (0.4 + 0.6 * collisionFactor);
      }
    }

    // Clamp and apply smoothness
    desiredSpeed = Math.max(config.minVelocity.in(MetersPerSecond),
                           Math.min(config.maxVelocity.in(MetersPerSecond), desiredSpeed));
    desiredSpeed *= (1.0 - config.smoothness);

    // ========================================================================
    // Calculate velocity
    // ========================================================================

    var vx = MetersPerSecond.of(combinedDirection.getX() * desiredSpeed);
    var vy = MetersPerSecond.of(combinedDirection.getY() * desiredSpeed);

    // ========================================================================
    // Calculate rotation
    // ========================================================================

    double baseOmega = rotationPID.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians()
    );

    double totalOmega = baseOmega + Math.max(-1.5, Math.min(1.5, angularAvoidance));
    var omega = RadiansPerSecond.of(totalOmega);

    if (omega.gt(config.maxAngularVelocity))
    {
      omega = config.maxAngularVelocity;
    }
    else if (omega.lt(config.maxAngularVelocity.times(-1)))
    {
      omega = config.maxAngularVelocity.times(-1);
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
  }

  /**
   * Reset the path smoothing state.
   * Call this when starting a new path or when teleporting.
   */
  public void resetSmoothing()
  {
    previousDirection = new Translation2d();
  }

  /**
   * Get debug info about the current state.
   *
   * @param currentPose Current robot pose
   * @param targetPose Target pose
   * @param obstacles List of obstacles
   * @param config Drive configuration
   * @return Debug string
   */
  public String getDebugInfo(
      Pose2d currentPose,
      Pose2d targetPose,
      List<Obstacle> obstacles,
      DriveConfig config)
  {
    Translation2d currentPos = currentPose.getTranslation();
    double distToGoal = currentPos.getDistance(targetPose.getTranslation());

    int staticCount = 0;
    int dynamicCount = 0;
    int dangerousCount = 0;
    double closestDist = Double.POSITIVE_INFINITY;

    for (Obstacle obs : obstacles)
    {
      switch (obs.type)
      {
        case STATIC -> staticCount++;
        case DYNAMIC -> dynamicCount++;
        case DANGEROUS -> dangerousCount++;
        default -> {}
      }

      double dist = currentPos.getDistance(obs.position);
      if (dist < closestDist)
      {
        closestDist = dist;
      }
    }

    return String.format(
        "Distance to goal: %.2fm | Obstacles: %d (S:%d D:%d X:%d) | Closest: %.2fm",
        distToGoal, obstacles.size(), staticCount, dynamicCount, dangerousCount, closestDist
    );
  }
}
