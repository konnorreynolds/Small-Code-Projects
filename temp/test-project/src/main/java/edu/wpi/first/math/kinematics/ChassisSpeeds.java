package edu.wpi.first.math.kinematics;

/**
 * WPILib ChassisSpeeds - minimal implementation
 */
public class ChassisSpeeds {
    public double vxMetersPerSecond;
    public double vyMetersPerSecond;
    public double omegaRadiansPerSecond;

    public ChassisSpeeds() {
        this(0, 0, 0);
    }

    public ChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(vx=%.2f, vy=%.2f, omega=%.2f)",
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
}
