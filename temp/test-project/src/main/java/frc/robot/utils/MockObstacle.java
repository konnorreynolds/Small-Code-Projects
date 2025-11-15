package frc.robot.utils;

/**
 * Mock Obstacle
 */
public class MockObstacle {
    public MockPose2d position;
    public MockTranslation2d velocity;
    public boolean isDynamic;
    public double avoidanceWeight = 1.0;
    public double difficultyLevel = 0.5;

    public MockObstacle(MockPose2d pos, MockTranslation2d vel, boolean dynamic) {
        this.position = pos;
        this.velocity = vel;
        this.isDynamic = dynamic;
    }

    public static MockObstacle robot(MockPose2d pose, MockTranslation2d velocity, boolean dynamic) {
        return new MockObstacle(pose, velocity, dynamic);
    }

    public static MockObstacle wall(MockTranslation2d start, MockTranslation2d end) {
        double midX = (start.x + end.x) / 2;
        double midY = (start.y + end.y) / 2;
        return new MockObstacle(
            new MockPose2d(midX, midY, 0),
            new MockTranslation2d(0, 0),
            false
        );
    }

    public MockObstacle aggressive() {
        this.avoidanceWeight = 2.5;
        return this;
    }

    public MockObstacle difficulty(double level) {
        this.difficultyLevel = level;
        return this;
    }
}
