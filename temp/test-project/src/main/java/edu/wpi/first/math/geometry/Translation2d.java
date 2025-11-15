package edu.wpi.first.math.geometry;

/**
 * WPILib Translation2d - minimal implementation
 */
public class Translation2d {
    private final double x;
    private final double y;

    public Translation2d() {
        this(0, 0);
    }

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getNorm() {
        return Math.hypot(x, y);
    }

    public double getDistance(Translation2d other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    public Translation2d plus(Translation2d other) {
        return new Translation2d(x + other.x, y + other.y);
    }

    public Translation2d minus(Translation2d other) {
        return new Translation2d(x - other.x, y - other.y);
    }

    public Translation2d times(double scalar) {
        return new Translation2d(x * scalar, y * scalar);
    }

    public Translation2d div(double scalar) {
        return new Translation2d(x / scalar, y / scalar);
    }

    @Override
    public String toString() {
        return String.format("Translation2d(%.2f, %.2f)", x, y);
    }
}
