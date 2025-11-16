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
  private final List<SmartOpponent> opponents = new ArrayList<>(2);
  private AngularVelocity maxAngularVel = DegreesPerSecond.of(720);
  private LinearVelocity maxLinearVel = MetersPerSecond.of(8);

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

    // Opponents - they have built-in obstacle avoidance now
    var arena = new Arena2025Reefscape();
    opponents.add(new KitBot(arena, DriverStation.Alliance.Red, 1));
    opponents.add(new KitBot(arena, DriverStation.Alliance.Red, 2));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() ->
      opponents.forEach(o -> o.setState(SmartOpponent.States.STARTING))));

    SmartDashboard.putData("Field", field);
  }

  public Command driveToPose(Pose2d target) {
    return run(() -> {
      var current = drive.getPose();
      var delta = target.getTranslation().minus(current.getTranslation());
      var dist = delta.getNorm();
      if (dist > 0.1) {
        var speed = Math.min(4.0, dist * 2.0);
        var vx = (delta.getX() / dist) * speed;
        var vy = (delta.getY() / dist) * speed;
        var omega = target.getRotation().minus(current.getRotation()).getRadians() * 3.0;
        drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(vx, vy, omega));
      } else {
        drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds());
      }
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
