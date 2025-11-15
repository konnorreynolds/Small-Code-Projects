package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ReefscapeSwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final ReefscapeSwerveSubsystem m_swerve = new ReefscapeSwerveSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();

    // Set default commands
    // Example: m_swerve.setDefaultCommand(...);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // ============================================================================
    // 2025 REEFSCAPE BUTTON MAPPINGS
    // ============================================================================

    // A Button: Drive to Reef Center with ultra-decisive avoidance
    m_driverController.a().onTrue(
        m_swerve.driveToWithAvoidance(
            new Pose2d(8.23, 4.115, Rotation2d.kZero)  // Reef center
        )
    );

    // B Button: Precision approach to Blue Coral Station
    m_driverController.b().onTrue(
        m_swerve.driveToCoralStation(new Translation2d(1.5, 1.0))
    );

    // X Button: Aggressive push through opponents
    m_driverController.x().onTrue(
        m_swerve.aggressiveDriveTo(
            new Pose2d(14, 4, Rotation2d.kZero)  // Across field
        )
    );

    // Y Button: Reset robot pose to starting position
    m_driverController.y().onTrue(
        m_swerve.resetPose(new Pose2d(2, 2, Rotation2d.kZero))
    );

    // Back Button: Clear all tracked opponents
    m_driverController.back().onTrue(
        Commands.runOnce(() -> m_swerve.clearOpponents())
    );

    // Start Button: Run example reef navigation auto
    m_driverController.start().onTrue(
        m_swerve.exampleReefNavigationAuto()
    );

    // Left Bumper: Add test opponent at (5, 3)
    m_driverController.leftBumper().onTrue(
        Commands.runOnce(() -> m_swerve.addOpponent(new Pose2d(5, 3, Rotation2d.kZero)))
    );

    // Right Bumper: Add test opponent at (7, 5)
    m_driverController.rightBumper().onTrue(
        Commands.runOnce(() -> m_swerve.addOpponent(new Pose2d(7, 5, Rotation2d.kZero)))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the example reef navigation autonomous command
    return m_swerve.exampleReefNavigationAuto();
  }
}
