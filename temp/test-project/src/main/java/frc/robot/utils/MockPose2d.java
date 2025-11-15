package frc.robot.utils;

/**
 * Mock Pose2d
 */
public class MockPose2d {
    public double x, y, theta;

    public MockPose2d(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public String toString() {
        return String.format("Pose2d(%.2f, %.2f, %.2f°)", x, y, Math.toDegrees(theta));
    }
}
