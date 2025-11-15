package frc.robot;

import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Robot container with controller bindings
 */
public class RobotContainer {
    // Subsystems
    private final SwerveDriveSubsystem swerve;

    // Controller (mocked for standalone testing)
    private final MockController controller;

    public RobotContainer() {
        swerve = new SwerveDriveSubsystem();
        controller = new MockController();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // A Button: Drive to pose with opponent avoidance
        // Config and obstacles are created in the subsystem
        controller.onAButton(() -> {
            System.out.println("\n[A BUTTON PRESSED]");
            swerve.driveToWithOpponentAvoidance();
            System.out.println("Drive command completed\n");
        });

        // Simulate A button press after init
        System.out.println("Button bindings configured:");
        System.out.println("  [A] Drive to pose with opponent avoidance");
    }

    public void simulateAButtonPress() {
        controller.pressA();
    }

    /**
     * Mock controller for standalone testing
     */
    static class MockController {
        private Runnable aButtonAction;

        public void onAButton(Runnable action) {
            this.aButtonAction = action;
        }

        public void pressA() {
            if (aButtonAction != null) {
                aButtonAction.run();
            }
        }
    }
}
