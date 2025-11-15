import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * INTEGRATION EXAMPLE: Advanced Obstacle Avoidance + YAMS Swerve Drive
 *
 * This shows how to integrate AdvancedObstacleAvoidance with YAMS SwerveDrive.
 * Based on the YAMS swerve_drive example.
 */
public class YAMS_INTEGRATION_EXAMPLE
{
  /**
   * Enhanced SwerveSubsystem with obstacle avoidance capabilities.
   * Drop-in replacement for the YAMS example SwerveSubsystem.
   */
  public static class EnhancedSwerveSubsystem extends SubsystemBase
  {
    // YAMS swerve drive (from example)
    private final yams.mechanisms.swerve.SwerveDrive drive;

    // Obstacle avoidance controller
    private final AdvancedObstacleAvoidance avoidanceController;
    private final PIDController rotationPID;
    private final AdvancedObstacleAvoidance.DriveConfig avoidanceConfig;

    // Obstacle tracking
    private List<AdvancedObstacleAvoidance.Obstacle> obstacles;

    public EnhancedSwerveSubsystem()
    {
      // Initialize YAMS swerve drive (from YAMS example)
      // ... (same as YAMS example - see yams-repo/examples/swerve_drive)
      this.drive = createYamsSwerveDrive();  // See YAMS example for full code

      // Initialize obstacle avoidance
      this.avoidanceController = new AdvancedObstacleAvoidance();
      this.rotationPID = new PIDController(10.0, 0.0, 0.5);
      this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);

      // Use aggressive config for match play
      this.avoidanceConfig = AdvancedObstacleAvoidance.DriveConfig.forOpponent();

      // Initialize empty obstacle list
      this.obstacles = new ArrayList<>();
    }

    /**
     * Update obstacles from vision/network tables/sensors.
     * Call this in periodic() to update obstacle positions.
     */
    public void updateObstacles(List<AdvancedObstacleAvoidance.Obstacle> newObstacles)
    {
      this.obstacles = newObstacles;
    }

    /**
     * Drive to pose with obstacle avoidance.
     * This is the key integration point!
     *
     * @param targetPose Where to go
     * @return Command that drives with avoidance
     */
    public Command driveToPoseWithAvoidance(Pose2d targetPose)
    {
      return run(() -> {
        // Get current robot state
        Pose2d currentPose = drive.getPose();
        Translation2d currentVel = drive.getChassisSpeeds().toTranslation2d();

        // Calculate chassis speeds with obstacle avoidance
        ChassisSpeeds speeds = avoidanceController.driveToPoseWithAvoidance(
            currentPose,
            targetPose,
            obstacles,
            rotationPID,
            avoidanceConfig,
            currentVel
        );

        // Drive using YAMS
        drive.setRobotRelativeChassisSpeeds(speeds);
      });
    }

    /**
     * Example: Navigate through opponent defense.
     */
    public Command navigateThroughDefense(Pose2d goal)
    {
      return runOnce(() -> {
        // Build obstacles from opponent robots
        List<AdvancedObstacleAvoidance.Obstacle> defenseObstacles = new ArrayList<>();

        // Example: Get opponent poses from vision/network tables
        List<Pose2d> opponentPoses = getOpponentPoses();  // Your vision code here
        for (Pose2d opponentPose : opponentPoses)
        {
          // Estimate opponent velocity (from previous poses)
          Translation2d opponentVel = estimateOpponentVelocity(opponentPose);

          // Create aggressive obstacle for defensive opponent
          AdvancedObstacleAvoidance.Obstacle opponent =
              AdvancedObstacleAvoidance.createAggressive(opponentPose, opponentVel);

          // Add predicted destination if known
          Translation2d predictedDest = predictOpponentDestination(opponentPose);
          if (predictedDest != null)
          {
            opponent.withDestination(predictedDest, 0.7);
          }

          defenseObstacles.add(opponent);
        }

        // Add field boundaries
        defenseObstacles.add(AdvancedObstacleAvoidance.createBoundary(
            new Translation2d(0, 0), Meters.of(0.3)
        ));

        updateObstacles(defenseObstacles);
      }).andThen(driveToPoseWithAvoidance(goal));
    }

    /**
     * Example: Auto navigate through note field.
     */
    public Command autoCollectNotes(List<Translation2d> notePoses)
    {
      return runOnce(() -> {
        List<AdvancedObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

        // Notes are soft obstacles (can drive through if needed)
        for (Translation2d notePos : notePoses)
        {
          obstacles.add(AdvancedObstacleAvoidance.createSoft(notePos, Meters.of(0.25)));
        }

        // Opponents are dangerous
        for (Pose2d opp : getOpponentPoses())
        {
          obstacles.add(AdvancedObstacleAvoidance.createAggressive(
              opp,
              estimateOpponentVelocity(opp)
          ));
        }

        updateObstacles(obstacles);
      }).andThen(() -> {
        // Your note collection logic here
      });
    }

    // Dummy methods - replace with your actual implementations
    private yams.mechanisms.swerve.SwerveDrive createYamsSwerveDrive()
    {
      // See YAMS example for full code
      return null;
    }

    private List<Pose2d> getOpponentPoses()
    {
      // Get from vision/network tables
      return new ArrayList<>();
    }

    private Translation2d estimateOpponentVelocity(Pose2d pose)
    {
      // Track previous poses and calculate velocity
      return new Translation2d();
    }

    private Translation2d predictOpponentDestination(Pose2d pose)
    {
      // Predict where opponent is going (e.g., toward goal, note, etc.)
      return null;
    }

    @Override
    public void periodic()
    {
      drive.updateTelemetry();
    }

    @Override
    public void simulationPeriodic()
    {
      drive.simIterate();
    }
  }

  /**
   * Usage example in RobotContainer.
   */
  public static class ExampleRobotContainer
  {
    private final EnhancedSwerveSubsystem swerve = new EnhancedSwerveSubsystem();

    public ExampleRobotContainer()
    {
      // Bind commands to buttons
      configureBindings();
    }

    private void configureBindings()
    {
      // Button 1: Drive to scoring position with avoidance
      // xboxController.button(1).onTrue(
      //     swerve.driveToPoseWithAvoidance(
      //         new Pose2d(Meters.of(5), Meters.of(3), Rotation2d.fromDegrees(0))
      //     )
      // );

      // Button 2: Navigate through defense
      // xboxController.button(2).onTrue(
      //     swerve.navigateThroughDefense(
      //         new Pose2d(Meters.of(8), Meters.of(4), Rotation2d.fromDegrees(45))
      //     )
      // );
    }

    public Command getAutonomousCommand()
    {
      // Example auto: Navigate through opponent defense to score
      Pose2d scoringPose = new Pose2d(Meters.of(10), Meters.of(5), Rotation2d.fromDegrees(0));

      return swerve.driveToPoseWithAvoidance(scoringPose)
          .withTimeout(5.0);
    }
  }

  /**
   * CONFIGURATION EXAMPLES
   */
  public static class ConfigExamples
  {
    // For aggressive offense (rush to goal)
    public static AdvancedObstacleAvoidance.DriveConfig getOffensiveConfig()
    {
      return AdvancedObstacleAvoidance.DriveConfig.ultraAggressive();
    }

    // For careful navigation (precision scoring)
    public static AdvancedObstacleAvoidance.DriveConfig getPrecisionConfig()
    {
      return AdvancedObstacleAvoidance.DriveConfig.forPrecision();
    }

    // For defensive play
    public static AdvancedObstacleAvoidance.DriveConfig getDefensiveConfig()
    {
      return AdvancedObstacleAvoidance.DriveConfig.forDefense();
    }

    // Custom config
    public static AdvancedObstacleAvoidance.DriveConfig getCustomConfig()
    {
      AdvancedObstacleAvoidance.DriveConfig config =
          new AdvancedObstacleAvoidance.DriveConfig();

      // Tune for your robot
      config.maxVelocity = MetersPerSecond.of(5.0);
      config.aggressiveness = 1.8;
      config.smoothness = 0.3;
      config.goalBias = 2.5;

      // Enable all smart features
      config.useCollisionPrediction = true;
      config.useVelocityAwareAvoidance = true;
      config.enablePathSmoothing = true;

      // Boost collision urgency
      config.collisionUrgencyMultiplier = 4.0;

      return config;
    }
  }

  /**
   * OBSTACLE BUILDING EXAMPLES
   */
  public static class ObstacleExamples
  {
    // Build obstacles from vision data
    public static List<AdvancedObstacleAvoidance.Obstacle> buildFromVision()
    {
      List<AdvancedObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

      // Example: Opponent robots
      // PhotonPipelineResult result = camera.getLatestResult();
      // for (PhotonTrackedTarget target : result.getTargets()) {
      //   Pose2d opponentPose = getOpponentPose(target);
      //   Translation2d vel = estimateVelocity(opponentPose);
      //   obstacles.add(AdvancedObstacleAvoidance.createRobot(opponentPose, vel, true));
      // }

      return obstacles;
    }

    // Build field boundaries
    public static List<AdvancedObstacleAvoidance.Obstacle> buildFieldBoundaries()
    {
      List<AdvancedObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

      // Field walls
      obstacles.add(AdvancedObstacleAvoidance.createWall(
          new Translation2d(0, 0),
          new Translation2d(16.54, 0)
      ));
      obstacles.add(AdvancedObstacleAvoidance.createWall(
          new Translation2d(0, 8.02),
          new Translation2d(16.54, 8.02)
      ));

      return obstacles;
    }

    // Build game-specific obstacles
    public static List<AdvancedObstacleAvoidance.Obstacle> buildGameObstacles()
    {
      List<AdvancedObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

      // Example: Stage zone (avoid during teleop)
      obstacles.add(AdvancedObstacleAvoidance.createZone(
          new Translation2d(Meters.of(12), Meters.of(4)),
          Meters.of(2.0)
      ));

      // Example: Game pieces (soft obstacles)
      Translation2d[] notePoses = {
          new Translation2d(Meters.of(3), Meters.of(2)),
          new Translation2d(Meters.of(5), Meters.of(4)),
          new Translation2d(Meters.of(7), Meters.of(6))
      };
      for (Translation2d notePos : notePoses)
      {
        obstacles.add(AdvancedObstacleAvoidance.createSoft(notePos, Meters.of(0.2)));
      }

      return obstacles;
    }
  }
}
