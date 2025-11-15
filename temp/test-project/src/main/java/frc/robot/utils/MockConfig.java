package frc.robot.utils;

/**
 * Mock Config for obstacle avoidance
 */
public class MockConfig {
    public double maxVelocity = 4.0;
    public double avoidanceRadius = 2.0;
    public double avoidanceStrength = 1.5;
    public double goalAttraction = 10.0;

    /**
     * Opponent config - aggressive avoidance for dynamic opponents
     */
    public static MockConfig forOpponent() {
        MockConfig c = new MockConfig();
        c.maxVelocity = 4.5;
        c.avoidanceRadius = 2.5;      // Larger safety bubble
        c.avoidanceStrength = 2.0;     // Strong repulsion
        c.goalAttraction = 12.0;       // High goal bias
        return c;
    }
}
