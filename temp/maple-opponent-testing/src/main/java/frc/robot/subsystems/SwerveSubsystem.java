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
  // Navigator - Choose one option below:

  // OPTION 1: Default balanced navigation
  private final CompactNavigator navigator = new CompactNavigator();

  // OPTION 2: Presets
  // private final CompactNavigator navigator = CompactNavigator.aggressive();  // Fast, push through
  // private final CompactNavigator navigator = CompactNavigator.defensive();   // Slow, safe
  // private final CompactNavigator navigator = CompactNavigator.rally();       // High speed with control
  // private final CompactNavigator navigator = CompactNavigator.bulldozer();   // Ignores opponents

  // OPTION 3: Custom parameters
  // private final CompactNavigator navigator = new CompactNavigator(
  //     5.0,   // maxSpeed (m/s)
  //     8.0,   // goalAttraction
  //     1.5,   // avoidanceStrength
  //     0.6    // opponentWeight (lower = allow bumping)
  // );

  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

  // Maple-sim integration
  private final Arena2025Reefscape arena = new Arena2025Reefscape();
  private final ReefscapeOpponentManager opponentManager = new ReefscapeOpponentManager();
  private final List<SmartKitBot> opponents = new ArrayList<>();

  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity  maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(8);

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

    // Initialize Reefscape field obstacles
    createReefscapeObstacles();

    // Share obstacles with opponent manager
    opponentManager.setSharedObstacles(CompactNavigator.fieldWalls());
    opponentManager.getSharedObstacles().addAll(CompactNavigator.reefscapeObstacles());

    // Create smart opponents with obstacle avoidance (2 KitBots on red alliance)
    // Each can have different navigation styles!

    // Opponent 1 - Use default balanced navigation
    SmartKitBot opponent1 = new SmartKitBot(opponentManager, DriverStation.Alliance.Red, 1);
    opponents.add(opponent1);

    // Opponent 2 - Use default balanced navigation
    SmartKitBot opponent2 = new SmartKitBot(opponentManager, DriverStation.Alliance.Red, 2);
    opponents.add(opponent2);

    // Examples of different opponent styles:
    // Aggressive opponent (fast, push through):
    // SmartKitBot opponent1 = new SmartKitBot(opponentManager, DriverStation.Alliance.Red, 1, CompactNavigator.aggressive());

    // Defensive opponent (slow, cautious):
    // SmartKitBot opponent2 = new SmartKitBot(opponentManager, DriverStation.Alliance.Red, 2, CompactNavigator.defensive());

    // Bulldozer opponent (pushes through other robots):
    // SmartKitBot opponent1 = new SmartKitBot(opponentManager, DriverStation.Alliance.Red, 1, CompactNavigator.bulldozer());

    // Rally opponent (high speed with control):
    // SmartKitBot opponent2 = new SmartKitBot(opponentManager, DriverStation.Alliance.Red, 2, CompactNavigator.rally());

    // Set opponents to active state when teleop starts
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
      for (SmartOpponent opp : opponents) {
        opp.setState(SmartOpponent.States.STARTING);
      }
    }));

    SmartDashboard.putData("Field", field);
    SmartDashboard.putString("Obstacle Count", "Total: " + navigator.getObstacleCount() + ", Opponents: 2");
  }

  private void createReefscapeObstacles() {
    // Add field walls and reefscape obstacles to navigator
    navigator.addObstacles(CompactNavigator.fieldWalls());
    navigator.addObstacles(CompactNavigator.reefscapeObstacles());

    System.out.println("Created " + navigator.getObstacleCount() + " obstacles using CompactNavigator");
  }


  public Command driveToPose(Pose2d pose)
  {
    return run(() -> {
      // Use CompactNavigator like a PIDController
      List<SmartOpponent> opponentList = new ArrayList<>(opponents);
      ChassisSpeeds speeds = navigator.calculate(drive.getPose(), pose, opponentList);

      drive.setRobotRelativeChassisSpeeds(speeds);

      // Visualize goal position
      field.getObject("Goal").setPose(pose);

      // Debug output
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
