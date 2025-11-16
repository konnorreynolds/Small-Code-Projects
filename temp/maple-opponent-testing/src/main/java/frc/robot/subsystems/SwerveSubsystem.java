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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
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
  private final List<SmartOpponent> opponents = new ArrayList<>(2);
  private AngularVelocity maxAngularVel = DegreesPerSecond.of(720);
  private LinearVelocity maxLinearVel = MetersPerSecond.of(8);

  // Compact obstacle-avoiding navigator with cached config for performance
  private static final class Navigator {
    private static final Translation2d ZERO = new Translation2d();
    private final ObstacleAvoidance controller = new ObstacleAvoidance();
    private final PIDController rotPID = new PIDController(1.5, 0, 0.1);
    private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>(60);
    private final ObstacleAvoidance.Config cfg = new ObstacleAvoidance.Config();
    private final double speed = 5.0, goalPull = 8.0, avoidStr = 1.5, oppWeight = 0.6;

    Navigator() {
      rotPID.enableContinuousInput(-Math.PI, Math.PI);
      cfg.maxVelocity = Meters.per(Second).of(speed);
      cfg.baseAvoidanceStrength = avoidStr;
      cfg.defaultAvoidanceRadius = Meters.of(1.2);
      cfg.goalBias = goalPull;
      cfg.predictionLookAhead = 1.2;
      cfg.useCollisionPrediction = true;
      cfg.useVelocityAwareAvoidance = true;
    }

    void addWall(double x, double y) {
      var w = ObstacleAvoidance.circle(new Translation2d(x, y), Meters.of(0.35));
      w.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
      w.avoidanceWeight = 2.0;
      w.priority = 1.0;
      obstacles.add(w);
    }

    void addCircle(double x, double y, double r) {
      var c = ObstacleAvoidance.circle(new Translation2d(x, y), Meters.of(r));
      c.avoidanceWeight = 2.0;
      obstacles.add(c);
    }

    ChassisSpeeds navigate(Pose2d current, Pose2d target, List<SmartOpponent> opps) {
      var all = new ArrayList<ObstacleAvoidance.Obstacle>(obstacles.size() + opps.size());
      all.addAll(obstacles);
      for (var opp : opps) {
        var o = ObstacleAvoidance.robot(opp.getPose(), ZERO, true);
        o.avoidanceWeight *= oppWeight;
        all.add(o);
      }
      return controller.drive(current, target, all, rotPID, cfg, ZERO);
    }
  }

  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot)
        .withMaximumAngularVelocity(maxAngularVel)
        .withMaximumLinearVelocity(maxLinearVel)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl()
        .withDeadband(0.01);
  }

  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return () -> new ChassisSpeeds(
        maxLinearVel.times(x.getAsDouble()).in(MetersPerSecond),
        maxLinearVel.times(y.getAsDouble()).in(MetersPerSecond),
        maxAngularVel.times(rot.getAsDouble()).in(RadiansPerSecond));
  }

  private SwerveModule createModule(SparkMax driveMotor, SparkMax steerMotor, CANcoder encoder, String name, Translation2d loc) {
    var driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4)).withClosedLoopController(50, 0, 4)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromStages("12:1", "2:1")))
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    var azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(new MechanismGearing(GearBox.fromStages("21:1")))
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    return new SwerveModule(new SwerveModuleConfig(
        new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg),
        new SparkWrapper(steerMotor, DCMotor.getNEO(1), azimuthCfg))
        .withAbsoluteEncoder(encoder.getAbsolutePosition().asSupplier())
        .withTelemetry(name, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(loc).withOptimization(true));
  }

  public SwerveSubsystem() {
    var d = Inches.of(24);
    drive = new SwerveDrive(new SwerveDriveConfig(this,
        createModule(new SparkMax(1, MotorType.kBrushless), new SparkMax(2, MotorType.kBrushless), new CANcoder(3), "fl", new Translation2d(d, d)),
        createModule(new SparkMax(4, MotorType.kBrushless), new SparkMax(5, MotorType.kBrushless), new CANcoder(6), "fr", new Translation2d(d, d.negate())),
        createModule(new SparkMax(7, MotorType.kBrushless), new SparkMax(8, MotorType.kBrushless), new CANcoder(9), "bl", new Translation2d(d.negate(), d)),
        createModule(new SparkMax(10, MotorType.kBrushless), new SparkMax(11, MotorType.kBrushless), new CANcoder(12), "br", new Translation2d(d.negate(), d.negate())))
        .withGyro(new Pigeon2(14).getYaw().asSupplier())
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0)));

    // Use MapleSim's Reefscape obstacles (from Arena2025Reefscape.ReefscapeFieldObstacleMap)
    addReefscapeObstacles();

    // Opponents
    var arena = new Arena2025Reefscape();
    opponents.add(new KitBot(arena, DriverStation.Alliance.Red, 1));
    opponents.add(new KitBot(arena, DriverStation.Alliance.Red, 2));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() ->
      opponents.forEach(o -> o.setState(SmartOpponent.States.STARTING))));

    SmartDashboard.putData("Field", field);
  }

  private void addReefscapeObstacles() {
    // Convert MapleSim field borders to navigation obstacles (as circles for smooth APF)
    addLine(0, 1.27, 0, 6.782);           // Blue wall
    addLine(0, 1.27, 1.672, 0);           // Blue coral station top
    addLine(0, 6.782, 1.672, 8.052);      // Blue coral station bottom
    addLine(17.548, 1.27, 17.548, 6.782); // Red wall
    addLine(17.548, 1.27, 15.876, 0);     // Red coral station top
    addLine(17.548, 6.782, 15.876, 8.052);// Red coral station bottom
    addLine(1.672, 8.052, 11, 8.052);     // Upper wall left
    addLine(12, 8.052, 15.876, 8.052);    // Upper wall right
    addLine(1.672, 0, 5.8, 0);            // Lower wall left
    addLine(6.3, 0, 15.876, 0);           // Lower wall right

    // Reefs (hexagonal - use circles to approximate)
    nav.addCircle(4.489, 4.026, 0.9);     // Blue reef
    nav.addCircle(13.059, 4.026, 0.9);    // Red reef
    nav.addCircle(8.774, 4.026, 0.2);     // Center pillar
  }

  private void addLine(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1, dy = y2 - y1, len = Math.sqrt(dx*dx + dy*dy);
    if (len < 0.01) return; // Skip zero-length lines
    for (double t = 0; t <= 1; t += 0.8 / len) nav.addWall(x1 + dx*t, y1 + dy*t);
  }

  public Command driveToPose(Pose2d target) {
    return run(() -> {
      drive.setRobotRelativeChassisSpeeds(nav.navigate(drive.getPose(), target, opponents));
      field.getObject("Goal").setPose(target);
    });
  }

  public Command driveWithSpeeds(Supplier<ChassisSpeeds> speeds) { return drive.drive(speeds); }
  public Command setChassisSpeeds(ChassisSpeeds s) { return run(() -> drive.setRobotRelativeChassisSpeeds(s)); }
  public Command lock() { return run(drive::lockPose); }
  public Command resetPose() { return Commands.runOnce(() -> drive.resetOdometry(new Pose2d(3, 3, Rotation2d.kZero))); }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
    for (int i = 0; i < opponents.size(); i++)
      field.getObject("Opponent" + i).setPose(opponents.get(i).getPose());
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
    opponents.forEach(SmartOpponent::simulationPeriodic);
  }
}
