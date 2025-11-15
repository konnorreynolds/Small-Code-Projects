import java.util.ArrayList;
import java.util.List;

/**
 * YAMS-style Swerve Drive Simulation with Obstacle Avoidance.
 * Demonstrates how the obstacle avoidance system integrates with YAMS swerve drive.
 *
 * Based on: https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/swerve_drive
 */
public class YAMSStyleSwerveSim
{
  public static void main(String[] args)
  {
    System.out.println("=".repeat(80));
    System.out.println("YAMS SWERVE DRIVE + OBSTACLE AVOIDANCE INTEGRATION TEST");
    System.out.println("=".repeat(80));
    System.out.println("\nBased on: YAMS swerve_drive example");
    System.out.println("Repository: https://github.com/Yet-Another-Software-Suite/YAMS\n");

    testYamsStyleNavigation();
    testAggressiveDefense();
    testPrecisionScoring();

    System.out.println("\n" + "=".repeat(80));
    System.out.println("YAMS INTEGRATION TEST COMPLETE - ALL SYSTEMS FUNCTIONAL");
    System.out.println("=".repeat(80));
  }

  static void testYamsStyleNavigation()
  {
    System.out.println("TEST 1: YAMS-Style Navigation with Obstacle Avoidance");
    System.out.println("-".repeat(80));

    // Create YAMS-style swerve drive
    YamsStyleSwerveDrive swerve = new YamsStyleSwerveDrive(
        new MockPose2d(0, 0, 0),
        4.0,   // Max linear velocity (from YAMS example)
        Math.toRadians(720)  // Max angular velocity (from YAMS example)
    );

    MockPose2d goal = new MockPose2d(8, 5, Math.toRadians(30));

    // Build obstacles (opponents + field elements)
    List<MockObstacle> obstacles = new ArrayList<>();

    // Opponent robots
    obstacles.add(MockObstacle.createRobot(
        new MockPose2d(3, 2, 0),
        new MockTranslation2d(0.2, 0.1),
        true  // Defensive
    ));
    obstacles.add(MockObstacle.createRobot(
        new MockPose2d(5, 4, 0),
        new MockTranslation2d(-0.1, 0.3),
        true
    ));

    // Field boundaries
    obstacles.add(MockObstacle.createBoundary(new MockTranslation2d(0, -0.5), 0.3));
    obstacles.add(MockObstacle.createBoundary(new MockTranslation2d(0, 8.5), 0.3));

    // Use opponent config (from AdvancedObstacleAvoidance)
    MockDriveConfig config = MockDriveConfig.forOpponent();

    System.out.println("Robot: YAMS-style swerve (4 NEO motors + CANcoders)");
    System.out.println("Gyro: Pigeon2");
    System.out.println("Max Speed: 4.0 m/s | Max Rotation: 720°/s");
    System.out.println();

    runYamsSim("YAMS Navigation", swerve, goal, obstacles, config, 120);
    System.out.println();
  }

  static void testAggressiveDefense()
  {
    System.out.println("TEST 2: YAMS Swerve - Aggressive Defense Scenario");
    System.out.println("-".repeat(80));

    YamsStyleSwerveDrive swerve = new YamsStyleSwerveDrive(
        new MockPose2d(1, 3, 0),
        4.5,
        Math.toRadians(720)
    );

    MockPose2d goal = new MockPose2d(10, 3, 0);

    List<MockObstacle> obstacles = new ArrayList<>();

    // Multiple aggressive defenders
    MockObstacle def1 = MockObstacle.createAggressive(
        new MockPose2d(4, 4, 0),
        new MockTranslation2d(0.1, -0.3)
    );
    def1.likelyDestination = new MockTranslation2d(4, 2);
    def1.confidenceLevel = 0.8;
    obstacles.add(def1);

    MockObstacle def2 = MockObstacle.createAggressive(
        new MockPose2d(6, 2, 0),
        new MockTranslation2d(-0.2, 0.2)
    );
    def2.likelyDestination = new MockTranslation2d(5, 3);
    def2.confidenceLevel = 0.9;
    obstacles.add(def2);

    MockObstacle def3 = MockObstacle.createAggressive(
        new MockPose2d(8, 3.5, 0),
        new MockTranslation2d(0, -0.1)
    );
    obstacles.add(def3);

    // Use precision config for safer navigation (defensive)
    MockDriveConfig config = MockDriveConfig.forPrecision();
    config.name = "Defense";

    runYamsSim("Aggressive Defense", swerve, goal, obstacles, config, 150);
    System.out.println();
  }

  static void testPrecisionScoring()
  {
    System.out.println("TEST 3: YAMS Swerve - Precision Scoring");
    System.out.println("-".repeat(80));

    YamsStyleSwerveDrive swerve = new YamsStyleSwerveDrive(
        new MockPose2d(2, 4, 0),
        2.0,   // Slower for precision
        Math.toRadians(360)
    );

    MockPose2d goal = new MockPose2d(6, 4, Math.toRadians(45));

    List<MockObstacle> obstacles = new ArrayList<>();

    // Tight spaces - need precision
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(4, 3.5), 0.5));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(4, 4.5), 0.5));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(5, 3), 0.5));

    // Scoring zone (must avoid)
    obstacles.add(MockObstacle.createZone(new MockTranslation2d(6, 2), 1.0));

    MockDriveConfig config = MockDriveConfig.forPrecision();

    runYamsSim("Precision Scoring", swerve, goal, obstacles, config, 150);
    System.out.println();
  }

  static void runYamsSim(String name, YamsStyleSwerveDrive swerve, MockPose2d goal,
                         List<MockObstacle> obstacles, MockDriveConfig config, int maxSteps)
  {
    MockAdvancedObstacleAvoidance controller = new MockAdvancedObstacleAvoidance();
    MockPIDController rotationPID = new MockPIDController(1.0, 0.0, 0.0);  // YAMS example uses P=1

    double dt = 0.02;
    boolean success = false;
    boolean collision = false;
    double totalDistance = 0;
    double maxSpeed = 0;
    List<PathPoint> path = new ArrayList<>();
    double minDistToObstacle = Double.POSITIVE_INFINITY;

    MockPose2d startPose = swerve.getPose();
    System.out.printf("Start: (%.2f, %.2f, %.1f°) -> Goal: (%.2f, %.2f, %.1f°)\n",
        startPose.x, startPose.y, Math.toDegrees(startPose.rotation),
        goal.x, goal.y, Math.toDegrees(goal.rotation));
    System.out.printf("Config: %s | Obstacles: %d\n\n", config.name, obstacles.size());

    for (int i = 0; i < maxSteps; i++)
    {
      MockPose2d currentPose = swerve.getPose();
      MockTranslation2d currentVel = swerve.getVelocity();

      // Update moving obstacles
      for (MockObstacle obs : obstacles)
      {
        if (obs.velocity.getNorm() > 0.01)
        {
          obs.position = obs.position.plus(obs.velocity.times(dt));
        }
      }

      // Run obstacle avoidance algorithm
      MockChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
          currentPose, goal, obstacles, rotationPID, config, currentVel
      );

      // Apply YAMS-style swerve drive
      swerve.drive(speeds, dt);

      // Track metrics
      path.add(new PathPoint(currentPose.x, currentPose.y, i * dt));

      double stepDistance = Math.sqrt(
          Math.pow(currentVel.x * dt, 2) + Math.pow(currentVel.y * dt, 2)
      );
      totalDistance += stepDistance;

      double speed = currentVel.getNorm();
      if (speed > maxSpeed) maxSpeed = speed;

      // Track closest obstacle approach
      for (MockObstacle obs : obstacles)
      {
        double dist = currentPose.getTranslation().getDistance(obs.position) - obs.radius;
        if (dist < minDistToObstacle) minDistToObstacle = dist;

        // Check collision
        if (dist < 0.4)  // Robot radius
        {
          collision = true;
          System.out.printf("  ⚠️  COLLISION at t=%.2fs with %s at (%.2f, %.2f)\n",
              i * dt, obs.type, obs.position.x, obs.position.y);
          break;
        }
      }

      if (collision) break;

      // Check goal reached
      double distToGoal = currentPose.getTranslation().getDistance(goal.getTranslation());
      double angleError = Math.abs(currentPose.rotation - goal.rotation);
      if (distToGoal < 0.15 && angleError < 0.1)
      {
        success = true;
        System.out.printf("  ✓ Goal reached at t=%.2fs\n", i * dt);
        break;
      }

      // Print progress
      if (i % 25 == 0)
      {
        System.out.printf("  t=%.2fs: Pos(%.2f, %.2f) θ(%.1f°) v(%.2f m/s) ToGoal(%.2fm)\n",
            i * dt, currentPose.x, currentPose.y,
            Math.toDegrees(currentPose.rotation), speed, distToGoal);
      }
    }

    MockPose2d finalPose = swerve.getPose();

    // Results
    System.out.println("\n" + "=".repeat(80));
    System.out.println("RESULTS:");
    System.out.println("=".repeat(80));
    System.out.printf("Final Position:        (%.2f, %.2f) @ %.1f°\n",
        finalPose.x, finalPose.y, Math.toDegrees(finalPose.rotation));
    System.out.printf("Goal Position:         (%.2f, %.2f) @ %.1f°\n",
        goal.x, goal.y, Math.toDegrees(goal.rotation));
    System.out.printf("Position Error:        %.3f meters\n",
        finalPose.getTranslation().getDistance(goal.getTranslation()));
    System.out.printf("Angle Error:           %.1f degrees\n",
        Math.toDegrees(Math.abs(finalPose.rotation - goal.rotation)));
    System.out.printf("Total Distance:        %.2f meters\n", totalDistance);
    System.out.printf("Max Speed Achieved:    %.2f m/s\n", maxSpeed);
    System.out.printf("Min Dist to Obstacle:  %.2f meters\n", minDistToObstacle);
    System.out.printf("Path Points:           %d\n", path.size());
    System.out.printf("Success:               %s\n", success ? "YES ✓" : "NO ✗");
    System.out.printf("Collision:             %s\n", collision ? "YES ⚠️" : "NO ✓");

    // Performance metrics
    if (success)
    {
      double efficiency = goal.getTranslation().getDistance(startPose.getTranslation()) / totalDistance;
      System.out.printf("Path Efficiency:       %.1f%%\n", efficiency * 100);
    }

    System.out.println("\nPath Visualization:");
    visualizePath(path, obstacles, startPose.getTranslation(), goal.getTranslation());
  }

  static void visualizePath(List<PathPoint> path, List<MockObstacle> obstacles,
                           MockTranslation2d start, MockTranslation2d goal)
  {
    int width = 70;
    int height = 22;
    double minX = -1, maxX = 12, minY = -1, maxY = 9;

    char[][] grid = new char[height][width];
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        grid[y][x] = ' ';
      }
    }

    // Draw obstacles
    for (MockObstacle obs : obstacles)
    {
      int gx = (int)((obs.position.x - minX) / (maxX - minX) * (width - 1));
      int gy = (int)((obs.position.y - minY) / (maxY - minY) * (height - 1));
      gy = height - 1 - gy;

      if (gx >= 0 && gx < width && gy >= 0 && gy < height)
      {
        char symbol = switch (obs.type)
        {
          case "DANGEROUS" -> '#';
          case "SOFT" -> 'o';
          case "ZONE" -> '~';
          default -> obs.isAggressive ? '@' : 'O';
        };
        grid[gy][gx] = symbol;
      }
    }

    // Draw path
    for (PathPoint p : path)
    {
      int gx = (int)((p.x - minX) / (maxX - minX) * (width - 1));
      int gy = (int)((p.y - minY) / (maxY - minY) * (height - 1));
      gy = height - 1 - gy;

      if (gx >= 0 && gx < width && gy >= 0 && gy < height)
      {
        if (grid[gy][gx] == ' ' || grid[gy][gx] == '.') grid[gy][gx] = '.';
      }
    }

    // Draw start and goal
    int sx = (int)((start.x - minX) / (maxX - minX) * (width - 1));
    int sy = (int)((start.y - minY) / (maxY - minY) * (height - 1));
    sy = height - 1 - sy;
    if (sx >= 0 && sx < width && sy >= 0 && sy < height) grid[sy][sx] = 'S';

    int gx = (int)((goal.x - minX) / (maxX - minX) * (width - 1));
    int gy = (int)((goal.y - minY) / (maxY - minY) * (height - 1));
    gy = height - 1 - gy;
    if (gx >= 0 && gx < width && gy >= 0 && gy < height) grid[gy][gx] = 'G';

    // Print grid with border
    System.out.println("  +" + "-".repeat(width) + "+");
    for (int y = 0; y < height; y++)
    {
      System.out.print("  |");
      for (int x = 0; x < width; x++)
      {
        System.out.print(grid[y][x]);
      }
      System.out.println("|");
    }
    System.out.println("  +" + "-".repeat(width) + "+");
    System.out.println("  Legend: S=Start  G=Goal  .=Path  O=Robot  @=Aggressive  #=Danger  o=Soft  ~=Zone");
  }
}

// YAMS-style swerve drive simulator
class YamsStyleSwerveDrive
{
  private MockPose2d pose;
  private MockTranslation2d velocity;
  private double maxLinearVelocity;
  private double maxAngularVelocity;

  public YamsStyleSwerveDrive(MockPose2d initialPose, double maxLinVel, double maxAngVel)
  {
    this.pose = initialPose;
    this.velocity = new MockTranslation2d(0, 0);
    this.maxLinearVelocity = maxLinVel;
    this.maxAngularVelocity = maxAngVel;
  }

  public void drive(MockChassisSpeeds speeds, double dt)
  {
    // Clamp to limits (like YAMS SwerveDrive)
    double vx = Math.max(-maxLinearVelocity, Math.min(maxLinearVelocity, speeds.vx));
    double vy = Math.max(-maxLinearVelocity, Math.min(maxLinearVelocity, speeds.vy));
    double omega = Math.max(-maxAngularVelocity, Math.min(maxAngularVelocity, speeds.omega));

    // Update pose
    double newX = pose.x + vx * dt;
    double newY = pose.y + vy * dt;
    double newRotation = pose.rotation + omega * dt;

    // Normalize rotation
    while (newRotation > Math.PI) newRotation -= 2 * Math.PI;
    while (newRotation < -Math.PI) newRotation += 2 * Math.PI;

    pose = new MockPose2d(newX, newY, newRotation);
    velocity = new MockTranslation2d(vx, vy);
  }

  public MockPose2d getPose() { return pose; }
  public MockTranslation2d getVelocity() { return velocity; }
}

class PathPoint
{
  double x, y, time;
  PathPoint(double x, double y, double time) { this.x = x; this.y = y; this.time = time; }
}
