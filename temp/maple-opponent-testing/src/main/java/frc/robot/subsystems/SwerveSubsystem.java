package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.opponentsim.SmartOpponent;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.KitBot;
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
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive drive;
  private final Field2d field = new Field2d();
  private final Navigator nav = new Navigator();
  private final List<SmartOpponent> opponents = new ArrayList<>();
  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity maximumChassisSpeedsLinearVelocity = MetersPerSecond.of(8);

  // Compact obstacle-avoiding navigator
  private static class Navigator {
    private final ObstacleAvoidance controller = new ObstacleAvoidance();
    private final PIDController rotPID = new PIDController(1.5, 0, 0.1);
    private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();
    private double speed = 5.0, goalPull = 8.0, avoidStr = 1.5, oppWeight = 0.6;

    Navigator() { rotPID.enableContinuousInput(-Math.PI, Math.PI); }

    void addWall(double x, double y) {
      ObstacleAvoidance.Obstacle w = ObstacleAvoidance.circle(new Translation2d(x, y), Meters.of(0.35));
      w.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
      w.avoidanceWeight = 2.0;
      w.priority = 1.0;
      obstacles.add(w);
    }

    void addCircle(double x, double y, double r) {
      ObstacleAvoidance.Obstacle c = ObstacleAvoidance.circle(new Translation2d(x, y), Meters.of(r));
      c.avoidanceWeight = 2.0;
      obstacles.add(c);
    }

    ChassisSpeeds navigate(Pose2d current, Pose2d target, List<SmartOpponent> opps) {
      List<ObstacleAvoidance.Obstacle> all = new ArrayList<>(obstacles);
      for (SmartOpponent opp : opps) {
        ObstacleAvoidance.Obstacle o = ObstacleAvoidance.robot(opp.getPose(), new Translation2d(), true);
        o.avoidanceWeight *= oppWeight;
        all.add(o);
      }
      ObstacleAvoidance.Config cfg = new ObstacleAvoidance.Config();
      cfg.maxVelocity = Meters.per(Second).of(speed);
      cfg.baseAvoidanceStrength = avoidStr;
      cfg.defaultAvoidanceRadius = Meters.of(1.2);
      cfg.goalBias = goalPull;
      cfg.predictionLookAhead = 1.2;
      cfg.useCollisionPrediction = true;
      cfg.useVelocityAwareAvoidance = true;
      return controller.drive(current, target, all, rotPID, cfg, new Translation2d());
    }
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

    // Setup obstacles - field walls as circles for smooth APF
    for (double y = 0; y <= 8.052; y += 0.8) { nav.addWall(0.8, y); nav.addWall(16.748, y); }
    for (double x = 0; x <= 17.548; x += 0.8) { nav.addWall(x, 0.8); nav.addWall(x, 7.252); }
    nav.addCircle(4.489, 4.026, 0.9);   // Blue reef
    nav.addCircle(13.059, 4.026, 0.9);  // Red reef
    nav.addCircle(8.774, 4.026, 0.54);  // Pillar

    // Create opponents
    Arena2025Reefscape arena = new Arena2025Reefscape();
    opponents.add(new KitBot(arena, DriverStation.Alliance.Red, 1));
    opponents.add(new KitBot(arena, DriverStation.Alliance.Red, 2));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() ->
      opponents.forEach(o -> o.setState(SmartOpponent.States.STARTING))));

    SmartDashboard.putData("Field", field);
  }

  public Command driveToPose(Pose2d target) {
    return run(() -> {
      drive.setRobotRelativeChassisSpeeds(nav.navigate(drive.getPose(), target, opponents));
      field.getObject("Goal").setPose(target);
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
