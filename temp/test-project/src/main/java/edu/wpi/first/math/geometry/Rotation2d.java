package edu.wpi.first.math.geometry;

/**
 * WPILib Rotation2d - minimal implementation
 */
public class Rotation2d {
    private final double radians;

    public Rotation2d() {
        this.radians = 0;
    }

    public Rotation2d(double radians) {
        this.radians = radians;
    }

    public double getRadians() {
        return radians;
    }

    public double getDegrees() {
        return Math.toDegrees(radians);
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(%.2f°)", getDegrees());
    }
}
