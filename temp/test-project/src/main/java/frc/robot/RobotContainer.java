package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
  private final SwerveDriveSubsystem m_swerve = new SwerveDriveSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // A Button: Drive to pose with opponent avoidance
    m_driverController.a().onTrue(
      Commands.runOnce(() -> m_swerve.driveToWithOpponentAvoidance(), m_swerve)
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
