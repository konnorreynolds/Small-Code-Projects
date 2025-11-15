package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * YAMS-style Swerve Drive Subsystem with Obstacle Avoidance
 *
 * Integrates obstacle avoidance into swerve drive control.
 * Maps controller A button to drive to pose with opponent avoidance.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  // Swerve drive components (would be real YAGSL in production)
  private Pose2d currentPose;
  private ChassisSpeeds currentSpeeds;
  private ObstacleNavigator navigator;

  // Obstacle configuration
  private Config config;
  private List<Obstacle> obstacles;

  public SwerveDriveSubsystem() {
    currentPose = new Pose2d(2.0, 2.0, new Rotation2d());
    currentSpeeds = new ChassisSpeeds(0, 0, 0);
    navigator = new ObstacleNavigator();
    config = Config.forOpponent();
    obstacles = new ArrayList<>();

    System.out.println("SwerveDriveSubsystem initialized");
    System.out.println("  Starting pose: " + currentPose);
  }

  /**
   * Drive to pose with opponent avoidance (A button command)
   * Creates opponent obstacle and uses opponent config internally.
   */
  public void driveToWithOpponentAvoidance() {
    // Target pose
    Pose2d target = new Pose2d(8.0, 4.0, new Rotation2d());

    // Create opponent obstacle
    Obstacle opponent = Obstacle.robot(
        new Pose2d(5.0, 3.0, new Rotation2d()),
        new Translation2d(0.2, 0.1),
        true
    ).aggressive().difficulty(1.0);  // Hard difficulty opponent

    System.out.println("\n=== DRIVE TO POSE WITH OPPONENT AVOIDANCE ===");
    System.out.println("Current: " + currentPose);
    System.out.println("Target:  " + target);
    System.out.println("Opponent: " + opponent.position + " moving at " + opponent.velocity.getNorm() + " m/s");
    System.out.println("Config: Opponent (aggressive, difficulty=" + opponent.difficultyLevel + ")");

    // Create obstacle list
    List<Obstacle> obstacleList = createObstacles(opponent);

    // Simulate navigation
    int steps = 0;
    Pose2d current = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
    Translation2d velocity = new Translation2d(0, 0);

    System.out.println("\nNavigation:");

    while (distanceTo(current, target) > 0.3 && steps < 20) {
      double dist = distanceTo(current, target);

      // Get navigation command
      ChassisSpeeds speeds = navigator.drive(
          current, target, obstacleList, config, velocity
      );

      // Update position
      double dt = 0.2;
      Translation2d newTranslation = current.getTranslation().plus(
          new Translation2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt)
      );
      current = new Pose2d(newTranslation, current.getRotation());
      velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      double elapsed = steps * dt;
      double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      if (steps % 2 == 0) {
        System.out.printf("  t=%.2fs: Pos(%.2f, %.2f) Speed(%.2f m/s) Dist(%.2fm)%n",
            elapsed, current.getX(), current.getY(), speed, dist);
      }

      steps++;
    }

    double totalTime = steps * 0.2;
    double finalDist = distanceTo(current, target);

    System.out.println("\nResults:");
    System.out.println("  Final position: " + current);
    System.out.println("  Distance to goal: " + String.format("%.2fm", finalDist));
    System.out.println("  Time: " + String.format("%.2fs", totalTime));
    System.out.println("  Success: " + (finalDist < 0.3 ? "YES ✓" : "NO"));
    System.out.println("===========================================\n");

    // Update robot pose
    currentPose = current;
  }

  /**
   * Create obstacles including opponent and field boundaries
   */
  private List<Obstacle> createObstacles(Obstacle opponent) {
    List<Obstacle> obstacleList = new ArrayList<>();

    // Add opponent
    obstacleList.add(opponent);

    // Add field boundaries (FRC field is 54' x 27' = 16.54m x 8.23m)
    obstacleList.add(Obstacle.wall(new Translation2d(0, 0), new Translation2d(16.54, 0)));
    obstacleList.add(Obstacle.wall(new Translation2d(0, 8.23), new Translation2d(16.54, 8.23)));

    return obstacleList;
  }

  private double distanceTo(Pose2d from, Pose2d to) {
    return from.getTranslation().getDistance(to.getTranslation());
  }

  public Pose2d getPose() {
    return currentPose;
  }

  /**
   * Obstacle representation
   */
  public static class Obstacle {
    public Pose2d position;
    public Translation2d velocity;
    public boolean isDynamic;
    public double avoidanceWeight = 1.0;
    public double difficultyLevel = 0.5;

    private Obstacle(Pose2d pos, Translation2d vel, boolean dynamic) {
      this.position = pos;
      this.velocity = vel;
      this.isDynamic = dynamic;
    }

    public static Obstacle robot(Pose2d pose, Translation2d velocity, boolean dynamic) {
      return new Obstacle(pose, velocity, dynamic);
    }

    public static Obstacle wall(Translation2d start, Translation2d end) {
      Translation2d mid = start.plus(end).div(2.0);
      return new Obstacle(
          new Pose2d(mid, new Rotation2d()),
          new Translation2d(0, 0),
          false
      );
    }

    public Obstacle aggressive() {
      this.avoidanceWeight = 2.5;
      return this;
    }

    public Obstacle difficulty(double level) {
      this.difficultyLevel = level;
      return this;
    }
  }

  /**
   * Configuration for obstacle avoidance
   */
  public static class Config {
    public double maxVelocity = 4.0;
    public double avoidanceRadius = 2.0;
    public double avoidanceStrength = 1.5;
    public double goalAttraction = 10.0;

    /**
     * Opponent config - aggressive avoidance for dynamic opponents
     */
    public static Config forOpponent() {
      Config c = new Config();
      c.maxVelocity = 4.5;
      c.avoidanceRadius = 2.5;      // Larger safety bubble
      c.avoidanceStrength = 2.0;     // Strong repulsion
      c.goalAttraction = 12.0;       // High goal bias
      return c;
    }
  }

  /**
   * Obstacle navigation using APF algorithm
   */
  private static class ObstacleNavigator {
    public ChassisSpeeds drive(Pose2d current, Pose2d target,
                               List<Obstacle> obstacles, Config config,
                               Translation2d currentVel) {
      Translation2d currentPos = current.getTranslation();
      Translation2d targetPos = target.getTranslation();

      double dist = currentPos.getDistance(targetPos);

      if (dist < 0.01) return new ChassisSpeeds(0, 0, 0);

      // Goal attraction
      Translation2d toGoal = targetPos.minus(currentPos);
      Translation2d desiredDir = toGoal.div(toGoal.getNorm());
      Translation2d avoidance = new Translation2d(0, 0);

      // Obstacle repulsion
      for (Obstacle obs : obstacles) {
        Translation2d toObstacle = currentPos.minus(obs.position.getTranslation());
        double obsDist = toObstacle.getNorm();

        if (obsDist < config.avoidanceRadius && obsDist > 0.01) {
          double repulsion = config.avoidanceStrength * obs.avoidanceWeight *
              Math.pow(1.0 - obsDist / config.avoidanceRadius, 2.0);
          Translation2d repulsionDir = toObstacle.div(obsDist);
          avoidance = avoidance.plus(repulsionDir.times(repulsion));
        }
      }

      // Combine goal attraction and obstacle repulsion
      Translation2d combined = desiredDir.times(config.goalAttraction).plus(avoidance);
      double mag = combined.getNorm();

      if (mag > 0.01) {
        combined = combined.div(mag);
      }

      // Scale by max velocity
      double speed = Math.min(config.maxVelocity, dist * 2.0);
      Translation2d velocity = combined.times(speed);

      return new ChassisSpeeds(velocity.getX(), velocity.getY(), 0);
    }
  }
}
