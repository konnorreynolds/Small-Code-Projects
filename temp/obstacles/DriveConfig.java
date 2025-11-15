package obstacles;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.*;

/**
 * Configuration for obstacle-avoiding drive system.
 * Contains all tunable parameters for speed, avoidance, and behavior.
 */
public class DriveConfig
{
  // ========================================================================
  // Speed Limits
  // ========================================================================

  /** Maximum linear velocity of the robot */
  public Measure<Velocity<Distance>> maxVelocity = MetersPerSecond.of(4.0);

  /** Minimum linear velocity (below this, robot stops) */
  public Measure<Velocity<Distance>> minVelocity = MetersPerSecond.of(0.3);

  /** Maximum angular velocity (rotation speed) */
  public Measure<Velocity<Angle>> maxAngularVelocity = RadiansPerSecond.of(2.0 * Math.PI);

  // ========================================================================
  // Behavior
  // ========================================================================

  /**
   * Aggressiveness factor (higher = faster, more direct paths).
   * Range: 0.5 - 2.0
   * - 0.7 = Careful/defensive
   * - 1.0 = Normal
   * - 1.6 = Aggressive/offensive
   */
  public double aggressiveness = 1.0;

  /**
   * Smoothness factor (higher = smoother but slower response).
   * Range: 0.0 - 1.0
   * - 0.1 = Very responsive, can be jerky
   * - 0.7 = Smooth and stable
   */
  public double smoothness = 0.7;

  /**
   * Goal bias (how strongly to prefer direct path to goal).
   * Range: 1.0 - 3.0
   * - 1.0 = Equal weighting with avoidance
   * - 2.0 = Prefer goal path strongly
   */
  public double goalBias = 1.5;

  // ========================================================================
  // Obstacle Avoidance
  // ========================================================================

  /** Base strength of obstacle avoidance */
  public double baseAvoidanceStrength = 1.0;

  /** Default radius to start avoiding obstacles */
  public Measure<Distance> defaultAvoidanceRadius = Meters.of(1.2);

  /** Danger radius - inside this, apply emergency avoidance */
  public Measure<Distance> dangerRadius = Meters.of(0.4);

  // ========================================================================
  // Smart Prediction
  // ========================================================================

  /** How far ahead to predict obstacle positions (seconds) */
  public double predictionLookAhead = 0.8;

  /** Use time-to-collision for smarter avoidance */
  public boolean useCollisionPrediction = true;

  /** Adjust avoidance based on relative velocity */
  public boolean useVelocityAwareAvoidance = true;

  /** Prefer paths that stay farther from obstacles */
  public boolean preferSaferPaths = true;

  // ========================================================================
  // Obstacle Priorities
  // ========================================================================

  /** Respect obstacle.priority field (0.0-1.0) */
  public boolean respectObstaclePriority = true;

  /** Multiplier for avoidance when collision is imminent */
  public double collisionUrgencyMultiplier = 2.0;

  // ========================================================================
  // Type Multipliers
  // ========================================================================

  /** Avoidance multiplier for static obstacles */
  public double staticObstacleMultiplier = 1.0;

  /** Avoidance multiplier for dynamic (moving) obstacles */
  public double dynamicObstacleMultiplier = 1.4;

  /** Avoidance multiplier for dangerous obstacles */
  public double dangerousObstacleMultiplier = 2.5;

  /** Avoidance multiplier for soft obstacles (can pass through if needed) */
  public double softObstacleMultiplier = 0.4;

  /** Avoidance multiplier for zone obstacles */
  public double zoneObstacleMultiplier = 0.6;

  /** Avoidance multiplier for aggressive/blocking obstacles */
  public double aggressiveObstacleMultiplier = 1.8;

  // ========================================================================
  // Distance Zones
  // ========================================================================

  /** Distance at which to use full speed */
  public Measure<Distance> fastZoneDistance = Meters.of(2.0);

  /** Distance at which to start slowing down */
  public Measure<Distance> slowZoneDistance = Meters.of(0.5);

  /** Distance at which to use precision control */
  public Measure<Distance> precisionDistance = Meters.of(0.15);

  // ========================================================================
  // Advanced Features
  // ========================================================================

  /** Automatically adjust speed based on nearby obstacles */
  public boolean useAdaptiveSpeed = true;

  /** Apply angular (rotation) avoidance when near obstacles */
  public boolean enableAngularAvoidance = true;

  /** Smooth out direction changes to prevent jitter */
  public boolean enablePathSmoothing = true;

  /**
   * Path smoothing factor (0.0-1.0).
   * - 0.0 = No smoothing, instant direction changes
   * - 0.5 = Moderate smoothing
   * - 1.0 = Maximum smoothing (very gradual turns)
   */
  public double pathSmoothingFactor = 0.3;

  // ========================================================================
  // Factory Methods
  // ========================================================================

  /**
   * Create config optimized for aggressive opponent play.
   *
   * @return Aggressive drive config
   */
  public static DriveConfig forOpponent()
  {
    DriveConfig config = new DriveConfig();
    config.maxVelocity                   = MetersPerSecond.of(4.5);
    config.maxAngularVelocity            = RadiansPerSecond.of(3.0 * Math.PI);
    config.aggressiveness                = 1.6;
    config.smoothness                    = 0.25;
    config.goalBias                      = 2.0;
    config.baseAvoidanceStrength         = 1.3;
    config.defaultAvoidanceRadius        = Meters.of(1.0);
    config.predictionLookAhead           = 0.8;
    config.useCollisionPrediction        = true;
    config.useVelocityAwareAvoidance     = true;
    config.preferSaferPaths              = true;
    config.collisionUrgencyMultiplier    = 3.0;
    config.dynamicObstacleMultiplier     = 1.6;
    config.aggressiveObstacleMultiplier  = 2.2;
    config.enablePathSmoothing           = true;
    config.pathSmoothingFactor           = 0.4;
    return config;
  }

  /**
   * Create config optimized for careful, precise navigation.
   *
   * @return Precision drive config
   */
  public static DriveConfig forPrecision()
  {
    DriveConfig config = new DriveConfig();
    config.maxVelocity                = MetersPerSecond.of(2.0);
    config.aggressiveness             = 0.7;
    config.smoothness                 = 0.8;
    config.baseAvoidanceStrength      = 2.0;
    config.defaultAvoidanceRadius     = Meters.of(1.6);
    config.dangerousObstacleMultiplier = 3.0;
    config.preferSaferPaths           = true;
    config.enablePathSmoothing        = true;
    config.pathSmoothingFactor        = 0.6;
    return config;
  }

  /**
   * Create config optimized for defensive play.
   *
   * @return Defensive drive config
   */
  public static DriveConfig forDefense()
  {
    DriveConfig config = new DriveConfig();
    config.maxVelocity                   = MetersPerSecond.of(3.5);
    config.aggressiveness                = 1.2;
    config.smoothness                    = 0.5;
    config.goalBias                      = 1.8;
    config.baseAvoidanceStrength         = 1.5;
    config.useCollisionPrediction        = true;
    config.useVelocityAwareAvoidance     = true;
    config.collisionUrgencyMultiplier    = 2.5;
    config.dynamicObstacleMultiplier     = 1.8;
    config.aggressiveObstacleMultiplier  = 2.5;
    return config;
  }

  /**
   * Create ultra-aggressive config for rushing to goal.
   * Ignores most obstacles except dangerous ones.
   *
   * @return Ultra-aggressive drive config
   */
  public static DriveConfig ultraAggressive()
  {
    DriveConfig config = new DriveConfig();
    config.maxVelocity                   = MetersPerSecond.of(5.0);
    config.aggressiveness                = 2.0;
    config.smoothness                    = 0.1;
    config.goalBias                      = 3.0;
    config.baseAvoidanceStrength         = 0.8;
    config.collisionUrgencyMultiplier    = 1.5;  // Less cautious
    config.softObstacleMultiplier        = 0.2;  // Almost ignore soft obstacles
    config.dynamicObstacleMultiplier     = 0.9;  // Don't worry much about moving obstacles
    config.preferSaferPaths              = false;
    return config;
  }

  /**
   * Create ultra-safe config for defensive/careful play.
   * Maximum avoidance, slow and steady.
   *
   * @return Ultra-safe drive config
   */
  public static DriveConfig ultraSafe()
  {
    DriveConfig config = new DriveConfig();
    config.maxVelocity                   = MetersPerSecond.of(2.5);
    config.aggressiveness                = 0.6;
    config.smoothness                    = 0.9;
    config.baseAvoidanceStrength         = 2.5;
    config.defaultAvoidanceRadius        = Meters.of(2.0);
    config.collisionUrgencyMultiplier    = 5.0;  // Very cautious
    config.useCollisionPrediction        = true;
    config.useVelocityAwareAvoidance     = true;
    config.dangerousObstacleMultiplier   = 4.0;
    config.dynamicObstacleMultiplier     = 2.0;
    config.preferSaferPaths              = true;
    config.enablePathSmoothing           = true;
    config.pathSmoothingFactor           = 0.6;
    return config;
  }

  @Override
  public String toString()
  {
    return String.format("DriveConfig[maxVel=%.2f m/s, aggr=%.2f, smooth=%.2f, goalBias=%.2f]",
        maxVelocity.in(MetersPerSecond), aggressiveness, smoothness, goalBias);
  }
}
