package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.KitBot;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Meters;

public class SwerveSubsystem extends SubsystemBase
{
  private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();
  private final List<ObstacleAvoidance.Obstacle> staticObstacles = new ArrayList<>();

  private final ObstacleAvoidance poseController = new ObstacleAvoidance();
  private final PIDController rotationPID = new PIDController(1.5, 0, 0.1);

  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

  // Maple-sim integration
  private final Arena2025Reefscape arena = new Arena2025Reefscape();
  private final ReefscapeOpponentManager opponentManager = new ReefscapeOpponentManager();
  private final List<SmartOpponent> opponents = new ArrayList<>();

  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity  maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(4);

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,
                                                    DoubleSupplier translationYScalar,
                                                    DoubleSupplier rotationScalar)
  {
    return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
        .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
        .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl()
        .withDeadband(0.01);
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar,
                                                        DoubleSupplier translationYScalar,
                                                        DoubleSupplier rotationScalar)
  {
    return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    MechanismGearing driveGearing         = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing       = new MechanismGearing(GearBox.fromStages("21:1"));
    PIDController    azimuthPIDController = new PIDController(1, 0, 0);
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(50, 0, 4)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);
    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem()
  {
    Pigeon2 gyro = new Pigeon2(14);
    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                          new SparkMax(2, MotorType.kBrushless),
                          new CANcoder(3),
                          "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                          new SparkMax(5, MotorType.kBrushless),
                          new CANcoder(6),
                          "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                          new SparkMax(8, MotorType.kBrushless),
                          new CANcoder(9),
                          "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
                          new SparkMax(11, MotorType.kBrushless),
                          new CANcoder(12),
                          "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    // Configure rotation PID with continuous input
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize Reefscape field obstacles
    createReefscapeObstacles();

    // Create maple-sim opponents (2 KitBots on red alliance)
    // Note: KitBot constructor automatically registers opponents with the manager
    KitBot opponent1 = new KitBot(opponentManager, DriverStation.Alliance.Red, 1);
    KitBot opponent2 = new KitBot(opponentManager, DriverStation.Alliance.Red, 2);
    opponents.add(opponent1);
    opponents.add(opponent2);

    // Set opponents to active state when teleop starts
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
      for (SmartOpponent opp : opponents) {
        opp.setState(SmartOpponent.States.STARTING);
      }
    }));

    SmartDashboard.putData("Field", field);
    SmartDashboard.putString("Obstacle Count", "Static: " + staticObstacles.size() + ", Dynamic: 2");

    // Visualize static obstacles on Field2d
    visualizeObstacles();
  }

  private void createReefscapeObstacles() {
    // Field boundaries (thicker walls for stronger avoidance)
    // Left wall (vertical)
    ObstacleAvoidance.Obstacle leftWall = ObstacleAvoidance.line(
        new Translation2d(0, 0), new Translation2d(0, 8.052), Meters.of(1.0)
    );
    leftWall.likelyDestination = new Translation2d(0, 8.052); // Mark as vertical
    leftWall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
    leftWall.priority = 1.0;
    staticObstacles.add(leftWall);

    // Right wall (vertical)
    ObstacleAvoidance.Obstacle rightWall = ObstacleAvoidance.line(
        new Translation2d(17.548, 0), new Translation2d(17.548, 8.052), Meters.of(1.0)
    );
    rightWall.likelyDestination = new Translation2d(17.548, 8.052); // Mark as vertical
    rightWall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
    rightWall.priority = 1.0;
    staticObstacles.add(rightWall);

    // Bottom wall (horizontal)
    ObstacleAvoidance.Obstacle bottomWall = ObstacleAvoidance.line(
        new Translation2d(0, 0), new Translation2d(17.548, 0), Meters.of(1.0)
    );
    bottomWall.likelyDestination = new Translation2d(17.548, 0); // Mark as horizontal
    bottomWall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
    bottomWall.priority = 1.0;
    staticObstacles.add(bottomWall);

    // Top wall (horizontal)
    ObstacleAvoidance.Obstacle topWall = ObstacleAvoidance.line(
        new Translation2d(0, 8.052), new Translation2d(17.548, 8.052), Meters.of(1.0)
    );
    topWall.likelyDestination = new Translation2d(17.548, 8.052); // Mark as horizontal
    topWall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
    topWall.priority = 1.0;
    staticObstacles.add(topWall);

    // Blue Reef (hexagon obstacle) - larger radius for better avoidance
    ObstacleAvoidance.Obstacle blueReef = ObstacleAvoidance.circle(
        new Translation2d(4.489, 4.026), Meters.of(1.0)
    );
    blueReef.type = ObstacleAvoidance.Obstacle.Type.STATIC;
    blueReef.priority = 0.9;
    staticObstacles.add(blueReef);

    // Red Reef (hexagon obstacle) - larger radius for better avoidance
    ObstacleAvoidance.Obstacle redReef = ObstacleAvoidance.circle(
        new Translation2d(13.059, 4.026), Meters.of(1.0)
    );
    redReef.type = ObstacleAvoidance.Obstacle.Type.STATIC;
    redReef.priority = 0.9;
    staticObstacles.add(redReef);

    // Center pillar - larger radius for better avoidance
    ObstacleAvoidance.Obstacle pillar = ObstacleAvoidance.circle(
        new Translation2d(8.774, 4.026), Meters.of(0.6)
    );
    pillar.type = ObstacleAvoidance.Obstacle.Type.STATIC;
    pillar.priority = 0.9;
    staticObstacles.add(pillar);

    System.out.println("Created " + staticObstacles.size() + " Reefscape static obstacles");
  }

  private void visualizeObstacles() {
    int obstacleIndex = 0;

    for (ObstacleAvoidance.Obstacle obstacle : staticObstacles) {
      String name = "Obstacle" + obstacleIndex;

      if (obstacle.shape == ObstacleAvoidance.Obstacle.Shape.CIRCLE) {
        // Visualize circle as a single pose with rotation indicating radius
        field.getObject(name).setPose(new Pose2d(
            obstacle.position,
            new Rotation2d()
        ));

        // Create a circle visualization using multiple poses
        int numPoints = 16;
        Pose2d[] circlePoses = new Pose2d[numPoints];
        for (int i = 0; i < numPoints; i++) {
          double angle = 2 * Math.PI * i / numPoints;
          double x = obstacle.position.getX() + obstacle.radius.in(Meters) * Math.cos(angle);
          double y = obstacle.position.getY() + obstacle.radius.in(Meters) * Math.sin(angle);
          circlePoses[i] = new Pose2d(x, y, Rotation2d.fromRadians(angle + Math.PI/2));
        }
        field.getObject(name + "_circle").setPoses(circlePoses);

      } else if (obstacle.shape == ObstacleAvoidance.Obstacle.Shape.RECTANGLE) {
        // Visualize rectangular obstacle as a line
        // Use likelyDestination to determine orientation
        Translation2d center = obstacle.position;
        Translation2d end = obstacle.likelyDestination;
        double w = obstacle.width.in(Meters);

        // Determine if vertical or horizontal by checking which coordinate changed
        boolean isVertical = Math.abs(end.getX() - center.getX()) < 0.01;

        int numPoints = Math.max(4, (int)(w / 0.5));
        Pose2d[] linePoses = new Pose2d[numPoints];

        for (int i = 0; i < numPoints; i++) {
          double t = (double)i / (numPoints - 1) - 0.5;
          if (isVertical) {
            // Vertical wall
            linePoses[i] = new Pose2d(
                center.getX(),
                center.getY() + w * t,
                Rotation2d.fromDegrees(0)
            );
          } else {
            // Horizontal wall
            linePoses[i] = new Pose2d(
                center.getX() + w * t,
                center.getY(),
                Rotation2d.fromDegrees(90)
            );
          }
        }

        field.getObject(name + "_line").setPoses(linePoses);
      }

      obstacleIndex++;
    }

    System.out.println("Visualized " + obstacleIndex + " static obstacles on Field2d");
  }

  public Command driveToPose(Pose2d pose)
  {
    return run(() -> {
      // Update dynamic obstacles list with current opponents
      obstacles.clear();
      obstacles.addAll(staticObstacles);

      // Add opponents as dynamic obstacles
      for (SmartOpponent opponent : opponents) {
        Pose2d opponentPose = opponent.getPose();
        // Create aggressive robot obstacle (velocity tracking not available from SmartOpponent)
        // Using zero velocity for now - APF will handle avoidance based on position
        Translation2d velocity = new Translation2d();

        ObstacleAvoidance.Obstacle opponentObstacle = ObstacleAvoidance.robot(
            opponentPose, velocity, true
        ).aggressive().difficulty(0.9);

        obstacles.add(opponentObstacle);
      }

      // Drive with obstacle avoidance using ultra-safe configuration
      // This ensures strong avoidance behavior
      ChassisSpeeds speeds = poseController.drive(
          drive.getPose(),
          pose,
          obstacles,
          rotationPID,
          ObstacleAvoidance.Config.ultraSafe(),
          new Translation2d()
      );

      drive.setRobotRelativeChassisSpeeds(speeds);

      // Visualize goal position
      field.getObject("Goal").setPose(pose);

      // Debug output
      SmartDashboard.putNumber("Obstacles/Total", obstacles.size());
      SmartDashboard.putNumber("Obstacles/Dynamic", opponents.size());
      SmartDashboard.putNumber("Distance to Goal", drive.getPose().getTranslation().getDistance(pose.getTranslation()));
    });
  }

  /**
   * Drive the {@link SwerveDrive} object with robot relative chassis speeds.
   *
   * @param speedsSupplier Robot relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command lock()
  {
    return run(drive::lockPose);
  }

  public Command resetRobotPose() {
        return Commands.runOnce(() ->
        drive.resetOdometry(new Pose2d(3, 3, Rotation2d.kZero)));
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());

    // Update opponent poses on Field2d
    for (int i = 0; i < opponents.size(); i++) {
      field.getObject("Opponent" + i).setPose(opponents.get(i).getPose());
    }
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();

    // Update opponent simulations
    for (SmartOpponent opponent : opponents) {
      opponent.simulationPeriodic();
    }
  }
}
