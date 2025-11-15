package obstacles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Meters;

/**
 * Advanced obstacle representation with collision prediction and smart avoidance.
 *
 * Features:
 * - Collision prediction using time-to-collision
 * - Smart kinematic prediction with destination biasing
 * - Velocity-aware avoidance radii
 * - Path intersection detection
 * - Priority system for obstacle importance
 * - Support for multiple shapes (circle, rectangle, robot)
 */
public class Obstacle
{
  // Basic properties
  public Translation2d position;
  public Translation2d velocity;
  public Translation2d acceleration;

  // Shape (for better collision detection)
  public ObstacleShape shape;
  public Measure<Distance> radius;         // For circular
  public Measure<Distance> width, height;  // For rectangular

  // Behavior
  public ObstacleType type;
  public double priority;           // 0.0-1.0: how important to avoid
  public double avoidanceWeight;    // Multiplier for avoidance force

  // Smart prediction
  public Translation2d likelyDestination;  // Where obstacle is probably going
  public double confidenceLevel;           // 0.0-1.0: confidence in prediction

  // Advanced features
  public boolean isAggressive;      // Does obstacle actively try to block us?
  public double timeToLive;         // How long until obstacle disappears (game pieces)
  public String id;                 // For tracking/debugging

  public enum ObstacleType
  {
    STATIC,        // Stationary
    DYNAMIC,       // Moving
    DANGEROUS,     // Must avoid
    SOFT,          // Can pass through if needed
    ZONE           // Area to avoid (like defense zone)
  }

  public enum ObstacleShape
  {
    CIRCLE,
    RECTANGLE,
    ROBOT          // Robot-shaped (oriented rectangle)
  }

  /**
   * Create a simple circular obstacle.
   *
   * @param position Center position of the obstacle
   * @param radius Radius of the obstacle
   */
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
   * Get predicted position at future time using kinematic model.
   * Uses position + velocity*t + 0.5*acceleration*t^2
   * If likely destination is set with high confidence, biases prediction toward it.
   *
   * @param timeSeconds Time into the future (seconds)
   * @return Predicted position
   */
  public Translation2d getPredictedPosition(double timeSeconds)
  {
    if (velocity.getNorm() < 0.01)
    {
      return position;
    }

    // Use kinematic equation: p = p0 + v*t + 0.5*a*t^2
    Translation2d predicted = position
        .plus(velocity.times(timeSeconds))
        .plus(acceleration.times(0.5 * timeSeconds * timeSeconds));

    // If we have a likely destination and high confidence, bias toward it
    if (confidenceLevel > 0.7 && likelyDestination != null)
    {
      Translation2d toDestination = likelyDestination.minus(predicted);
      double distToDestination = toDestination.getNorm();

      if (distToDestination > 0.1)
      {
        // Blend kinematic prediction with destination
        double blendFactor = confidenceLevel * 0.3;
        predicted = predicted.plus(toDestination.times(blendFactor));
      }
    }

    return predicted;
  }

  /**
   * Calculate time to collision with a moving point.
   * Returns Double.POSITIVE_INFINITY if no collision.
   *
   * @param point Position of the moving point
   * @param pointVelocity Velocity of the moving point
   * @return Time to collision in seconds, or infinity if no collision
   */
  public double getTimeToCollision(Translation2d point, Translation2d pointVelocity)
  {
    Translation2d relativePos = point.minus(position);
    Translation2d relativeVel = pointVelocity.minus(velocity);

    // If relative velocity is tiny, no collision
    if (relativeVel.getNorm() < 0.01)
    {
      return Double.POSITIVE_INFINITY;
    }

    // Closest approach time
    double t = -(relativePos.getX() * relativeVel.getX() +
                 relativePos.getY() * relativeVel.getY()) /
                (relativeVel.getNorm() * relativeVel.getNorm());

    // Only consider future collisions
    if (t < 0)
    {
      return Double.POSITIVE_INFINITY;
    }

    // Calculate closest distance
    Translation2d closestPoint = relativePos.plus(relativeVel.times(t));
    double closestDistance = closestPoint.getNorm();

    // Check if it's a collision
    if (closestDistance < radius.in(Meters))
    {
      return t;
    }

    return Double.POSITIVE_INFINITY;
  }

  /**
   * Get effective avoidance radius based on relative velocity.
   * Faster relative motion = larger buffer needed.
   *
   * @param robotVelocity Velocity of the robot
   * @return Effective radius for avoidance
   */
  public Measure<Distance> getEffectiveRadius(Translation2d robotVelocity)
  {
    Translation2d relativeVel = velocity.minus(robotVelocity);
    double relativeSpeed = relativeVel.getNorm();

    // Base radius + speed-dependent buffer
    double effectiveRadius = radius.in(Meters) + (relativeSpeed * 0.3);

    return Meters.of(effectiveRadius);
  }

  /**
   * Check if robot path intersects this obstacle.
   * Useful for multi-step path planning.
   *
   * @param start Start position of the path
   * @param end End position of the path
   * @return True if the path intersects this obstacle
   */
  public boolean intersectsPath(Translation2d start, Translation2d end)
  {
    // Vector from start to end
    Translation2d path = end.minus(start);
    double pathLength = path.getNorm();

    if (pathLength < 0.01)
    {
      return start.getDistance(position) < radius.in(Meters);
    }

    // Vector from start to obstacle
    Translation2d toObstacle = position.minus(start);

    // Project onto path
    double projection = (toObstacle.getX() * path.getX() +
                        toObstacle.getY() * path.getY()) / pathLength;

    // Clamp to path segment
    projection = Math.max(0, Math.min(pathLength, projection));

    // Closest point on path
    Translation2d closestPoint = start.plus(path.div(pathLength).times(projection));

    // Check distance
    return closestPoint.getDistance(position) < radius.in(Meters);
  }

  /**
   * Set likely destination for smarter prediction.
   *
   * @param destination Where the obstacle is likely heading
   * @param confidence Confidence level (0.0-1.0)
   * @return This obstacle (for chaining)
   */
  public Obstacle withDestination(Translation2d destination, double confidence)
  {
    this.likelyDestination = destination;
    this.confidenceLevel = Math.max(0.0, Math.min(1.0, confidence));
    return this;
  }

  /**
   * Mark as aggressive (actively blocking).
   *
   * @return This obstacle (for chaining)
   */
  public Obstacle asAggressive()
  {
    this.isAggressive = true;
    this.avoidanceWeight *= 1.5;
    return this;
  }

  /**
   * Set the priority of this obstacle.
   *
   * @param priority Priority level (0.0-1.0)
   * @return This obstacle (for chaining)
   */
  public Obstacle withPriority(double priority)
  {
    this.priority = Math.max(0.0, Math.min(1.0, priority));
    return this;
  }

  /**
   * Set the avoidance weight multiplier.
   *
   * @param weight Avoidance weight multiplier
   * @return This obstacle (for chaining)
   */
  public Obstacle withAvoidanceWeight(double weight)
  {
    this.avoidanceWeight = weight;
    return this;
  }

  /**
   * Set acceleration for more accurate prediction.
   *
   * @param acceleration Acceleration vector
   * @return This obstacle (for chaining)
   */
  public Obstacle withAcceleration(Translation2d acceleration)
  {
    this.acceleration = acceleration;
    return this;
  }

  /**
   * Set time-to-live for temporary obstacles.
   *
   * @param timeToLive Time in seconds until obstacle disappears
   * @return This obstacle (for chaining)
   */
  public Obstacle withTimeToLive(double timeToLive)
  {
    this.timeToLive = timeToLive;
    return this;
  }

  /**
   * Set custom ID for tracking.
   *
   * @param id Custom ID string
   * @return This obstacle (for chaining)
   */
  public Obstacle withId(String id)
  {
    this.id = id;
    return this;
  }

  @Override
  public String toString()
  {
    return String.format("Obstacle[id=%s, type=%s, pos=(%.2f, %.2f), radius=%.2f, priority=%.2f]",
        id, type, position.getX(), position.getY(), radius.in(Meters), priority);
  }
}
