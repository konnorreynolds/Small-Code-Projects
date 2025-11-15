package frc.robot;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.*;

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
        // A Button: Drive to target pose with opponent avoidance
        controller.onAButton(() -> {
            System.out.println("\n[A BUTTON PRESSED]");
            System.out.println("Driving to target pose (8.0, 4.0) with opponent avoidance...");

            // Create target pose
            MockPose2d target = new MockPose2d(8.0, 4.0, 0);

            // Add opponent obstacle
            MockObstacle opponent = MockObstacle.robot(
                new MockPose2d(5.0, 3.0, 0),
                new MockTranslation2d(0.2, 0.1),
                true
            ).aggressive().difficulty(1.0);  // Hard difficulty opponent

            // Execute drive command
            swerve.driveToWithOpponentAvoidance(target, opponent);

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
