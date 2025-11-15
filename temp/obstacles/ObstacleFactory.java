package obstacles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Meters;

/**
 * Factory class for creating various shaped obstacles with a single static reference.
 * Provides convenient methods for creating common obstacle types.
 *
 * Usage:
 * - ObstacleFactory.circle(position, radius)
 * - ObstacleFactory.rectangle(center, width, height)
 * - ObstacleFactory.robot(pose, velocity, isDefensive)
 * - ObstacleFactory.wall(start, end)
 * - ObstacleFactory.boundary(position, size)
 * - ObstacleFactory.zone(center, radius)
 * - ObstacleFactory.soft(position, radius)
 */
public class ObstacleFactory
{
  /**
   * Create a simple circular obstacle at a position.
   *
   * @param position Center position of the obstacle
   * @param radius Radius of the circular obstacle
   * @return New circular obstacle
   */
  public static Obstacle circle(Translation2d position, Measure<Distance> radius)
  {
    return new Obstacle(position, radius);
  }

  /**
   * Create a rectangular obstacle.
   *
   * @param center Center position of the rectangle
   * @param width Width of the rectangle
   * @param height Height of the rectangle
   * @return New rectangular obstacle
   */
  public static Obstacle rectangle(Translation2d center, Measure<Distance> width, Measure<Distance> height)
  {
    Obstacle obs = new Obstacle(center, width.divide(2.0));
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = width;
    obs.height = height;
    obs.type = Obstacle.ObstacleType.STATIC;
    return obs;
  }

  /**
   * Create an obstacle from another robot's pose.
   * Automatically sized for typical FRC robot with bumpers.
   *
   * @param pose Robot's current pose
   * @param velocity Robot's velocity vector
   * @param isDefensive Whether the robot is playing defensively
   * @return New robot-shaped obstacle
   */
  public static Obstacle robot(Pose2d pose, Translation2d velocity, boolean isDefensive)
  {
    Obstacle obs = new Obstacle(pose.getTranslation(), Meters.of(0.5));
    obs.velocity = velocity;
    obs.shape = Obstacle.ObstacleShape.ROBOT;
    obs.width = Meters.of(0.8);   // Typical robot width with bumpers
    obs.height = Meters.of(0.9);  // Typical robot length with bumpers
    obs.type = Obstacle.ObstacleType.DYNAMIC;
    obs.priority = isDefensive ? 1.0 : 0.7;
    obs.avoidanceWeight = isDefensive ? 2.0 : 1.3;
    obs.isAggressive = isDefensive;
    return obs;
  }

  /**
   * Create a wall obstacle between two points.
   *
   * @param start Start point of the wall
   * @param end End point of the wall
   * @return New wall obstacle
   */
  public static Obstacle wall(Translation2d start, Translation2d end)
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

  /**
   * Create a field boundary obstacle.
   * These are high-priority obstacles that should never be hit.
   *
   * @param position Position of the boundary
   * @param size Size of the boundary
   * @return New boundary obstacle
   */
  public static Obstacle boundary(Translation2d position, Measure<Distance> size)
  {
    Obstacle obs = new Obstacle(position, size);
    obs.type = Obstacle.ObstacleType.DANGEROUS;
    obs.priority = 1.0;
    obs.avoidanceWeight = 5.0;  // Never hit boundaries
    return obs;
  }

  /**
   * Create a soft obstacle (game piece, low-priority object).
   * Can be passed through if necessary.
   *
   * @param position Position of the soft obstacle
   * @param radius Radius of the soft obstacle
   * @return New soft obstacle
   */
  public static Obstacle soft(Translation2d position, Measure<Distance> radius)
  {
    Obstacle obs = new Obstacle(position, radius);
    obs.type = Obstacle.ObstacleType.SOFT;
    obs.priority = 0.3;
    obs.avoidanceWeight = 0.4;
    return obs;
  }

  /**
   * Create a zone obstacle (area to avoid, like a defense zone).
   *
   * @param center Center of the zone
   * @param radius Radius of the zone
   * @return New zone obstacle
   */
  public static Obstacle zone(Translation2d center, Measure<Distance> radius)
  {
    Obstacle obs = new Obstacle(center, radius);
    obs.type = Obstacle.ObstacleType.ZONE;
    obs.priority = 0.5;
    obs.avoidanceWeight = 0.8;
    obs.shape = Obstacle.ObstacleShape.CIRCLE;
    return obs;
  }

  /**
   * Create an elliptical obstacle.
   *
   * @param center Center of the ellipse
   * @param radiusX Horizontal radius
   * @param radiusY Vertical radius
   * @return New elliptical obstacle (approximated as rectangle)
   */
  public static Obstacle ellipse(Translation2d center, Measure<Distance> radiusX, Measure<Distance> radiusY)
  {
    Obstacle obs = new Obstacle(center, radiusX);
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = radiusX.times(2.0);
    obs.height = radiusY.times(2.0);
    return obs;
  }

  /**
   * Create a polygon obstacle (approximated as circle with radius = max distance from center).
   *
   * @param vertices Vertices of the polygon
   * @return New polygon obstacle (approximated as circle)
   */
  public static Obstacle polygon(Translation2d... vertices)
  {
    if (vertices.length == 0)
    {
      throw new IllegalArgumentException("Polygon must have at least one vertex");
    }

    // Calculate centroid
    Translation2d center = new Translation2d();
    for (Translation2d vertex : vertices)
    {
      center = center.plus(vertex);
    }
    center = center.div(vertices.length);

    // Find maximum distance from center
    double maxRadius = 0.0;
    for (Translation2d vertex : vertices)
    {
      double dist = vertex.getDistance(center);
      if (dist > maxRadius)
      {
        maxRadius = dist;
      }
    }

    return new Obstacle(center, Meters.of(maxRadius));
  }

  /**
   * Create a temporary obstacle (game piece that will disappear).
   *
   * @param position Position of the temporary obstacle
   * @param radius Radius of the obstacle
   * @param timeToLive How long until the obstacle disappears (seconds)
   * @return New temporary obstacle
   */
  public static Obstacle temporary(Translation2d position, Measure<Distance> radius, double timeToLive)
  {
    Obstacle obs = soft(position, radius);
    obs.timeToLive = timeToLive;
    return obs;
  }

  /**
   * Create an aggressive obstacle (actively blocking).
   *
   * @param pose Obstacle's pose
   * @param velocity Obstacle's velocity
   * @return New aggressive obstacle
   */
  public static Obstacle aggressive(Pose2d pose, Translation2d velocity)
  {
    return robot(pose, velocity, true).asAggressive();
  }

  /**
   * Create a moving obstacle with predicted destination.
   *
   * @param position Current position
   * @param velocity Current velocity
   * @param destination Where the obstacle is likely heading
   * @param confidence Confidence level (0.0-1.0) that it's heading there
   * @return New moving obstacle with destination prediction
   */
  public static Obstacle moving(Translation2d position, Translation2d velocity,
                                Translation2d destination, double confidence)
  {
    Obstacle obs = new Obstacle(position, Meters.of(0.5));
    obs.velocity = velocity;
    obs.type = Obstacle.ObstacleType.DYNAMIC;
    obs.likelyDestination = destination;
    obs.confidenceLevel = Math.max(0.0, Math.min(1.0, confidence));
    return obs;
  }

  /**
   * Create a line obstacle (thin wall).
   *
   * @param start Start point of the line
   * @param end End point of the line
   * @param thickness Thickness of the line
   * @return New line obstacle
   */
  public static Obstacle line(Translation2d start, Translation2d end, Measure<Distance> thickness)
  {
    Translation2d center = start.plus(end).div(2.0);
    double length = start.getDistance(end);

    Obstacle obs = new Obstacle(center, thickness.divide(2.0));
    obs.shape = Obstacle.ObstacleShape.RECTANGLE;
    obs.width = Meters.of(length);
    obs.height = thickness;
    obs.type = Obstacle.ObstacleType.STATIC;
    return obs;
  }

  /**
   * Create a square obstacle.
   *
   * @param center Center of the square
   * @param sideLength Length of each side
   * @return New square obstacle
   */
  public static Obstacle square(Translation2d center, Measure<Distance> sideLength)
  {
    return rectangle(center, sideLength, sideLength);
  }
}
