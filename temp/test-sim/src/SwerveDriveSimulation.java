import java.util.ArrayList;
import java.util.List;

/**
 * Advanced swerve drive simulation with obstacle avoidance.
 * Simulates a real swerve drive robot navigating through obstacle fields.
 */
public class SwerveDriveSimulation
{
  public static void main(String[] args)
  {
    System.out.println("=".repeat(80));
    System.out.println("SWERVE DRIVE + OBSTACLE AVOIDANCE - INTEGRATION TEST");
    System.out.println("=".repeat(80));
    System.out.println();

    // Note: YAMS doesn't include swerve drive (it's for mechanisms like arms/elevators)
    // Using WPILib-style swerve drive simulation instead
    System.out.println("NOTE: Using WPILib-based swerve drive simulation");
    System.out.println("      (YAMS is for mechanisms, not drivetrain)");
    System.out.println();

    testSwerveWithObstacles();
    testSwerveSlalomCourse();
    testSwerveDefensiveManeuver();

    System.out.println("\n" + "=".repeat(80));
    System.out.println("SWERVE DRIVE SIMULATION COMPLETE");
    System.out.println("=".repeat(80));
  }

  static void testSwerveWithObstacles()
  {
    System.out.println("TEST 1: Swerve Drive Through Obstacle Field");
    System.out.println("-".repeat(80));

    SwerveDriveSimulator swerve = new SwerveDriveSimulator(
        new MockPose2d(0, 0, 0),
        4.5,  // Max velocity m/s
        3.0 * Math.PI  // Max angular velocity rad/s
    );

    MockPose2d goal = new MockPose2d(10, 5, Math.PI / 4);

    List<MockObstacle> obstacles = new ArrayList<>();
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(3, 2), 0.8));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(5, 3), 0.8));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(7, 4), 0.8));
    obstacles.add(MockObstacle.createBoundary(new MockTranslation2d(0, -1), 0.5));
    obstacles.add(MockObstacle.createBoundary(new MockTranslation2d(0, 7), 0.5));

    MockDriveConfig config = MockDriveConfig.forOpponent();

    runSwerveSim("Obstacle Field", swerve, goal, obstacles, config, 100);
    System.out.println();
  }

  static void testSwerveSlalomCourse()
  {
    System.out.println("TEST 2: Swerve Drive Slalom Course");
    System.out.println("-".repeat(80));

    SwerveDriveSimulator swerve = new SwerveDriveSimulator(
        new MockPose2d(0, 3, 0),
        4.0,
        2.5 * Math.PI
    );

    MockPose2d goal = new MockPose2d(12, 3, 0);

    // Create slalom pattern
    List<MockObstacle> obstacles = new ArrayList<>();
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(2, 3), 0.6));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(4, 4.5), 0.6));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(6, 1.5), 0.6));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(8, 4.5), 0.6));
    obstacles.add(MockObstacle.createCircle(new MockTranslation2d(10, 1.5), 0.6));

    MockDriveConfig config = MockDriveConfig.forOpponent();

    runSwerveSim("Slalom Course", swerve, goal, obstacles, config, 150);
    System.out.println();
  }

  static void testSwerveDefensiveManeuver()
  {
    System.out.println("TEST 3: Swerve Drive Defensive Maneuver (Moving Obstacles)");
    System.out.println("-".repeat(80));

    SwerveDriveSimulator swerve = new SwerveDriveSimulator(
        new MockPose2d(0, 3, 0),
        4.5,
        3.0 * Math.PI
    );

    MockPose2d goal = new MockPose2d(8, 3, 0);

    List<MockObstacle> obstacles = new ArrayList<>();

    // Aggressive defender moving toward us
    MockObstacle defender1 = MockObstacle.createAggressive(
        new MockPose2d(4, 5, 0),
        new MockTranslation2d(0, -0.3)  // Moving down
    );
    obstacles.add(defender1);

    // Another defender crossing path
    MockObstacle defender2 = MockObstacle.createAggressive(
        new MockPose2d(6, 1, 0),
        new MockTranslation2d(0, 0.4)  // Moving up
    );
    obstacles.add(defender2);

    MockDriveConfig config = MockDriveConfig.forDefense();

    runSwerveSim("Defensive Maneuver", swerve, goal, obstacles, config, 120);
    System.out.println();
  }

  static void runSwerveSim(String name, SwerveDriveSimulator swerve, MockPose2d goal,
                          List<MockObstacle> obstacles, MockDriveConfig config, int maxSteps)
  {
    MockAdvancedObstacleAvoidance controller = new MockAdvancedObstacleAvoidance();
    MockPIDController rotationPID = new MockPIDController(10.0, 0.0, 0.5);

    double dt = 0.02;  // 20ms timestep
    boolean success = false;
    boolean collision = false;
    double totalDistance = 0;
    double maxSpeed = 0;
    List<PathPoint> path = new ArrayList<>();

    MockPose2d startPose = swerve.getPose();
    System.out.printf("Start: (%.2f, %.2f, %.1f°) -> Goal: (%.2f, %.2f, %.1f°)\n",
        startPose.x, startPose.y, Math.toDegrees(startPose.rotation),
        goal.x, goal.y, Math.toDegrees(goal.rotation));
    System.out.printf("Swerve Max Speed: %.2f m/s | Max Rotation: %.2f rad/s\n",
        swerve.maxVelocity, swerve.maxAngularVelocity);
    System.out.printf("Obstacles: %d | Config: %s\n\n", obstacles.size(), config.name);

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

      // Calculate chassis speeds with obstacle avoidance
      MockChassisSpeeds speeds = controller.driveToPoseWithAvoidance(
          currentPose, goal, obstacles, rotationPID, config, currentVel
      );

      // Apply swerve drive kinematics
      swerve.drive(speeds, dt);

      // Track path
      path.add(new PathPoint(currentPose.x, currentPose.y, i * dt));

      // Calculate metrics
      double stepDistance = Math.sqrt(
          Math.pow(currentVel.x * dt, 2) +
          Math.pow(currentVel.y * dt, 2)
      );
      totalDistance += stepDistance;

      double speed = currentVel.getNorm();
      if (speed > maxSpeed) maxSpeed = speed;

      // Check collisions
      for (MockObstacle obs : obstacles)
      {
        double dist = currentPose.getTranslation().getDistance(obs.position);
        if (dist < obs.radius + 0.4)  // Robot radius ~0.4m
        {
          collision = true;
          System.out.printf("  ⚠️  COLLISION at t=%.2fs with obstacle at (%.2f, %.2f)\n",
              i * dt, obs.position.x, obs.position.y);
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
        System.out.printf("  t=%.2fs: Pos(%.2f, %.2f) Angle(%.1f°) Speed(%.2f m/s) ToGoal(%.2fm)\n",
            i * dt, currentPose.x, currentPose.y,
            Math.toDegrees(currentPose.rotation),
            speed, distToGoal);
      }
    }

    MockPose2d finalPose = swerve.getPose();

    System.out.println("\n" + "=".repeat(80));
    System.out.println("RESULTS:");
    System.out.println("=".repeat(80));
    System.out.printf("Final Position:    (%.2f, %.2f) @ %.1f°\n",
        finalPose.x, finalPose.y, Math.toDegrees(finalPose.rotation));
    System.out.printf("Total Distance:    %.2f meters\n", totalDistance);
    System.out.printf("Max Speed:         %.2f m/s\n", maxSpeed);
    System.out.printf("Path Points:       %d\n", path.size());
    System.out.printf("Success:           %s\n", success ? "YES ✓" : "NO ✗");
    if (collision) System.out.println("Collision:         YES ⚠️");

    // Print path visualization
    System.out.println("\nPath Visualization (ASCII):");
    visualizePath(path, obstacles, goal);
  }

  static void visualizePath(List<PathPoint> path, List<MockObstacle> obstacles, MockPose2d goal)
  {
    int width = 60;
    int height = 20;

    // Find bounds
    double minX = 0, maxX = 12, minY = 0, maxY = 6;

    char[][] grid = new char[height][width];
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        grid[y][x] = '.';
      }
    }

    // Draw obstacles
    for (MockObstacle obs : obstacles)
    {
      int gx = (int)((obs.position.x - minX) / (maxX - minX) * (width - 1));
      int gy = (int)((obs.position.y - minY) / (maxY - minY) * (height - 1));
      gy = height - 1 - gy;  // Flip Y

      if (gx >= 0 && gx < width && gy >= 0 && gy < height)
      {
        if (obs.type.equals("DANGEROUS")) grid[gy][gx] = 'X';
        else if (obs.isAggressive) grid[gy][gx] = 'A';
        else grid[gy][gx] = 'O';
      }
    }

    // Draw goal
    int goalX = (int)((goal.x - minX) / (maxX - minX) * (width - 1));
    int goalY = (int)((goal.y - minY) / (maxY - minY) * (height - 1));
    goalY = height - 1 - goalY;
    if (goalX >= 0 && goalX < width && goalY >= 0 && goalY < height)
    {
      grid[goalY][goalX] = 'G';
    }

    // Draw path
    for (PathPoint p : path)
    {
      int gx = (int)((p.x - minX) / (maxX - minX) * (width - 1));
      int gy = (int)((p.y - minY) / (maxY - minY) * (height - 1));
      gy = height - 1 - gy;

      if (gx >= 0 && gx < width && gy >= 0 && gy < height)
      {
        if (grid[gy][gx] == '.') grid[gy][gx] = '*';
      }
    }

    // Draw start
    if (!path.isEmpty())
    {
      PathPoint start = path.get(0);
      int sx = (int)((start.x - minX) / (maxX - minX) * (width - 1));
      int sy = (int)((start.y - minY) / (maxY - minY) * (height - 1));
      sy = height - 1 - sy;
      if (sx >= 0 && sx < width && sy >= 0 && sy < height)
      {
        grid[sy][sx] = 'S';
      }
    }

    // Print grid
    System.out.println("  " + "-".repeat(width));
    for (int y = 0; y < height; y++)
    {
      System.out.print("  ");
      for (int x = 0; x < width; x++)
      {
        System.out.print(grid[y][x]);
      }
      System.out.println();
    }
    System.out.println("  " + "-".repeat(width));
    System.out.println("  Legend: S=Start  G=Goal  *=Path  O=Obstacle  X=Danger  A=Aggressive");
  }
}

// ============================================================================
// SWERVE DRIVE SIMULATOR
// ============================================================================

class SwerveDriveSimulator
{
  private MockPose2d pose;
  private MockTranslation2d velocity;
  public double maxVelocity;
  public double maxAngularVelocity;

  public SwerveDriveSimulator(MockPose2d initialPose, double maxVel, double maxAngVel)
  {
    this.pose = initialPose;
    this.velocity = new MockTranslation2d(0, 0);
    this.maxVelocity = maxVel;
    this.maxAngularVelocity = maxAngVel;
  }

  public void drive(MockChassisSpeeds speeds, double dt)
  {
    // Clamp speeds to max
    double vx = Math.max(-maxVelocity, Math.min(maxVelocity, speeds.vx));
    double vy = Math.max(-maxVelocity, Math.min(maxVelocity, speeds.vy));
    double omega = Math.max(-maxAngularVelocity, Math.min(maxAngularVelocity, speeds.omega));

    // Convert field-relative to robot-relative if needed (simplified)
    // For simulation, assuming speeds are already in field coordinates

    // Update position
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
