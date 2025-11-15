package frc.robot.subsystems;

import frc.robot.utils.*;
import java.util.ArrayList;
import java.util.List;

/**
 * YAMS-style Swerve Drive Subsystem with Obstacle Avoidance
 *
 * Integrates obstacle avoidance into swerve drive control.
 * Maps controller A button to drive to pose with opponent avoidance.
 */
public class SwerveDriveSubsystem {
    // Mock swerve drive components
    private MockSwerveDrive drive;
    private MockPose2d currentPose;
    private MockChassisSpeeds currentSpeeds;
    private ObstacleNavigator navigator;

    public SwerveDriveSubsystem() {
        drive = new MockSwerveDrive();
        currentPose = new MockPose2d(2.0, 2.0, 0);
        currentSpeeds = new MockChassisSpeeds(0, 0, 0);
        navigator = new ObstacleNavigator();

        System.out.println("SwerveDriveSubsystem initialized");
        System.out.println("  Starting pose: " + currentPose);
    }

    /**
     * Drive to pose with opponent avoidance (A button command)
     * Creates opponent obstacle and uses opponent config internally.
     */
    public void driveToWithOpponentAvoidance() {
        // Target pose
        MockPose2d target = new MockPose2d(8.0, 4.0, 0);

        // Create opponent obstacle
        MockObstacle opponent = MockObstacle.robot(
            new MockPose2d(5.0, 3.0, 0),
            new MockTranslation2d(0.2, 0.1),
            true
        ).aggressive().difficulty(1.0);  // Hard difficulty opponent

        System.out.println("\n=== DRIVE TO POSE WITH OPPONENT AVOIDANCE ===");
        System.out.println("Current: " + currentPose);
        System.out.println("Target:  " + target);
        System.out.println("Opponent: " + opponent.position + " moving at " + opponent.velocity.getNorm() + " m/s");
        System.out.println("Config: Opponent (aggressive, difficulty=" + opponent.difficultyLevel + ")");

        // Create obstacle list
        List<MockObstacle> obstacles = createObstacles(opponent);

        // Use opponent config (aggressive, high avoidance)
        MockConfig config = MockConfig.forOpponent();

        // Simulate navigation
        int steps = 0;
        double startTime = System.currentTimeMillis() / 1000.0;
        MockPose2d current = new MockPose2d(currentPose.x, currentPose.y, currentPose.theta);
        MockTranslation2d velocity = new MockTranslation2d(0, 0);

        System.out.println("\nNavigation:");

        while (distanceTo(current, target) > 0.3 && steps < 20) {
            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double dist = Math.sqrt(dx * dx + dy * dy);

            // Get navigation command
            MockChassisSpeeds speeds = navigator.drive(
                current, target, obstacles, config, velocity
            );

            // Update position
            double dt = 0.2;
            current.x += speeds.vx * dt;
            current.y += speeds.vy * dt;
            velocity.x = speeds.vx;
            velocity.y = speeds.vy;

            double elapsed = steps * dt;
            double speed = Math.sqrt(speeds.vx * speeds.vx + speeds.vy * speeds.vy);

            if (steps % 2 == 0) {
                System.out.printf("  t=%.2fs: Pos(%.2f, %.2f) Speed(%.2f m/s) Dist(%.2fm)%n",
                    elapsed, current.x, current.y, speed, dist);
            }

            steps++;
        }

        double totalTime = steps * 0.2;
        double finalDist = distanceTo(current, target);

        System.out.println("\nResults:");
        System.out.println("  Final position: " + current);
        System.out.println("  Distance to goal: " + String.format("%.2fm", finalDist));
        System.out.println("  Time: " + String.format("%.2fs", totalTime));
        System.out.println("  Success: " + (finalDist < 0.3 ? "YES ✓" : "NO"));
        System.out.println("===========================================\n");

        // Update robot pose
        currentPose = current;
    }

    /**
     * Create obstacles including opponent and field boundaries
     */
    private List<MockObstacle> createObstacles(MockObstacle opponent) {
        List<MockObstacle> obstacles = new ArrayList<>();

        // Add opponent
        obstacles.add(opponent);

        // Add field boundaries (FRC field is 54' x 27' = 16.54m x 8.23m)
        obstacles.add(MockObstacle.wall(new MockTranslation2d(0, 0), new MockTranslation2d(16.54, 0)));
        obstacles.add(MockObstacle.wall(new MockTranslation2d(0, 8.23), new MockTranslation2d(16.54, 8.23)));

        return obstacles;
    }

    private double distanceTo(MockPose2d from, MockPose2d to) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public MockPose2d getPose() {
        return currentPose;
    }

    /**
     * Mock YAMS-style swerve drive
     */
    private static class MockSwerveDrive {
        public MockSwerveDrive() {
            System.out.println("  Mock YAMS swerve drive created (4 modules)");
        }

        public void setRobotRelativeSpeeds(MockChassisSpeeds speeds) {
            // Would command actual swerve modules
        }
    }

    /**
     * Obstacle navigation using APF algorithm
     */
    private static class ObstacleNavigator {
        public MockChassisSpeeds drive(MockPose2d current, MockPose2d target,
                                       List<MockObstacle> obstacles, MockConfig config,
                                       MockTranslation2d currentVel) {
            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double dist = Math.sqrt(dx * dx + dy * dy);

            if (dist < 0.01) return new MockChassisSpeeds(0, 0, 0);

            // Goal attraction
            MockTranslation2d desiredDir = new MockTranslation2d(dx / dist, dy / dist);
            MockTranslation2d avoidance = new MockTranslation2d(0, 0);

            // Obstacle repulsion
            for (MockObstacle obs : obstacles) {
                double obsDx = current.x - obs.position.x;
                double obsDy = current.y - obs.position.y;
                double obsDist = Math.sqrt(obsDx * obsDx + obsDy * obsDy);

                if (obsDist < config.avoidanceRadius && obsDist > 0.01) {
                    double repulsion = config.avoidanceStrength * obs.avoidanceWeight *
                        Math.pow(1.0 - obsDist / config.avoidanceRadius, 2.0);
                    avoidance.x += (obsDx / obsDist) * repulsion;
                    avoidance.y += (obsDy / obsDist) * repulsion;
                }
            }

            // Combine goal attraction and obstacle repulsion
            double combinedX = desiredDir.x * config.goalAttraction + avoidance.x;
            double combinedY = desiredDir.y * config.goalAttraction + avoidance.y;

            double mag = Math.sqrt(combinedX * combinedX + combinedY * combinedY);
            if (mag > 0.01) {
                combinedX /= mag;
                combinedY /= mag;
            }

            // Scale by max velocity
            double speed = Math.min(config.maxVelocity, dist * 2.0);
            return new MockChassisSpeeds(combinedX * speed, combinedY * speed, 0);
        }
    }
}
