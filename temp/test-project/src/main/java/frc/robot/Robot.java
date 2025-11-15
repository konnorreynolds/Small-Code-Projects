package frc.robot;

/**
 * Main robot class - TimedRobot entry point
 */
public class Robot {
    private RobotContainer robotContainer;

    public void robotInit() {
        robotContainer = new RobotContainer();
        System.out.println("Robot initialized");
    }

    public void robotPeriodic() {
        // CommandScheduler would run here in real WPILib
    }

    public void autonomousInit() {
        System.out.println("Autonomous mode started");
    }

    public void autonomousPeriodic() {
        // Auto commands would execute here
    }

    public void teleopInit() {
        System.out.println("Teleop mode started");
    }

    public void teleopPeriodic() {
        // Driver control happens here
    }

    public void disabledInit() {
        System.out.println("Robot disabled");
    }

    public void disabledPeriodic() {
        // Nothing runs while disabled
    }

    public RobotContainer getRobotContainer() {
        return robotContainer;
    }

    public static void main(String[] args) {
        Robot robot = new Robot();
        robot.robotInit();
        robot.teleopInit();

        System.out.println("\n=== YAMS Swerve Robot Ready ===");
        System.out.println("Simulating A button press...");
        System.out.println("===============================\n");

        // Simulate A button press
        robot.getRobotContainer().simulateAButtonPress();

        System.out.println("\nRobot demo complete!");
    }
}
