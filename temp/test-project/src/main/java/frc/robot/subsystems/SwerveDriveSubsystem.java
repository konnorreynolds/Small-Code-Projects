package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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

/**
 * YAMS Swerve Drive Subsystem with Obstacle Avoidance
 *
 * A button: Drive to pose with opponent avoidance using real YAMS swerve drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveDrive drive;
  private final Field2d field = new Field2d();
  private final ObstacleNavigator navigator = new ObstacleNavigator();

  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity maximumChassisSpeedsLinearVelocity = MetersPerSecond.of(4);

  public SwerveModule createModule(SparkMax driveMotor, SparkMax azimuthMotor, CANcoder absoluteEncoder,
                                   String moduleName, Translation2d location) {
    MechanismGearing driveGearing = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromStages("21:1"));

    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(50, 0, 4)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorController driveSMC = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);

    return new SwerveModule(moduleConfig);
  }

  public SwerveDriveSubsystem() {
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
        .withStartingPose(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);
    SmartDashboard.putData("Field", field);
  }

  /**
   * A Button Command: Drive to pose with opponent avoidance
   */
  public void driveToWithOpponentAvoidance() {
    Pose2d target = new Pose2d(8.0, 4.0, new Rotation2d());

    // Create opponent obstacle
    Obstacle opponent = Obstacle.robot(
        new Pose2d(5.0, 3.0, new Rotation2d()),
        new Translation2d(0.2, 0.1),
        true
    ).aggressive().difficulty(1.0);

    System.out.println("\n=== DRIVE TO POSE WITH OPPONENT AVOIDANCE ===");
    System.out.println("Current: " + drive.getPose());
    System.out.println("Target:  " + target);
    System.out.println("Opponent: " + opponent.position + " moving at " + opponent.velocity.getNorm() + " m/s");

    List<Obstacle> obstacles = createObstacles(opponent);
    Config config = Config.forOpponent();

    // Drive with obstacle avoidance
    ChassisSpeeds currentSpeeds = drive.getRobotRelativeSpeed();
    Translation2d currentVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    ChassisSpeeds speeds = navigator.drive(
        drive.getPose(),
        target,
        obstacles,
        config,
        currentVelocity
    );

    drive.setRobotRelativeChassisSpeeds(speeds);
    System.out.println("Obstacle avoidance command sent to YAMS swerve drive");
    System.out.println("===========================================\n");
  }

  private List<Obstacle> createObstacles(Obstacle opponent) {
    List<Obstacle> obstacleList = new ArrayList<>();
    obstacleList.add(opponent);

    // Field boundaries
    obstacleList.add(Obstacle.wall(new Translation2d(0, 0), new Translation2d(16.54, 0)));
    obstacleList.add(Obstacle.wall(new Translation2d(0, 8.23), new Translation2d(16.54, 8.23)));

    return obstacleList;
  }

  public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {
    return drive.drive(speedsSupplier);
  }

  public Command driveToPose(Pose2d pose) {
    return drive.driveToPose(pose);
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }

  // Obstacle avoidance classes
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
      return new Obstacle(new Pose2d(mid, new Rotation2d()), new Translation2d(0, 0), false);
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

  public static class Config {
    public double maxVelocity = 4.0;
    public double avoidanceRadius = 2.0;
    public double avoidanceStrength = 1.5;
    public double goalAttraction = 10.0;

    public static Config forOpponent() {
      Config c = new Config();
      c.maxVelocity = 4.5;
      c.avoidanceRadius = 2.5;
      c.avoidanceStrength = 2.0;
      c.goalAttraction = 12.0;
      return c;
    }
  }

  private static class ObstacleNavigator {
    public ChassisSpeeds drive(Pose2d current, Pose2d target, List<Obstacle> obstacles,
                               Config config, Translation2d currentVel) {
      Translation2d currentPos = current.getTranslation();
      Translation2d targetPos = target.getTranslation();
      double dist = currentPos.getDistance(targetPos);

      if (dist < 0.01) return new ChassisSpeeds(0, 0, 0);

      Translation2d toGoal = targetPos.minus(currentPos);
      Translation2d desiredDir = toGoal.div(toGoal.getNorm());
      Translation2d avoidance = new Translation2d(0, 0);

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

      Translation2d combined = desiredDir.times(config.goalAttraction).plus(avoidance);
      double mag = combined.getNorm();
      if (mag > 0.01) {
        combined = combined.div(mag);
      }

      double speed = Math.min(config.maxVelocity, dist * 2.0);
      Translation2d velocity = combined.times(speed);
      return new ChassisSpeeds(velocity.getX(), velocity.getY(), 0);
    }
  }
}
