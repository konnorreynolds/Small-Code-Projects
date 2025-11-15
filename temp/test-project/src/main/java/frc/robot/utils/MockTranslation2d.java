package frc.robot.utils;

/**
 * Mock Translation2d
 */
public class MockTranslation2d {
    public double x, y;

    public MockTranslation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getNorm() {
        return Math.sqrt(x * x + y * y);
    }
}
