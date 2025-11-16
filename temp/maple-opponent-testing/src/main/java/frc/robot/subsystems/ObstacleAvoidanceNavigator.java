package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.opponentsim.SmartOpponent;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.*;

/**
 * Compact obstacle avoidance navigator using Artificial Potential Fields (APF).
 * Returns ChassisSpeeds for navigating to a goal while avoiding obstacles.
 *
 * Includes preset configs for different difficulty levels and drive styles.
 */
public class ObstacleAvoidanceNavigator {
    // Tunable Parameters
    private final double navSpeedMps;
    private final double avoidanceRadius;
    private final double avoidanceStrength;
    private final double goalAttraction;
    private final double predictionTime;
    private final double opponentAvoidanceMultiplier;

    private final ObstacleAvoidance controller = new ObstacleAvoidance();
    private final PIDController rotationPID = new PIDController(1.5, 0, 0.1);
    private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

    /**
     * Create navigator with default balanced settings
     */
    public ObstacleAvoidanceNavigator() {
        this(5.0, 1.2, 1.5, 8.0, 1.2, 0.6);
    }

    /**
     * Create navigator with custom settings
     * @param navSpeedMps Navigation speed (m/s)
     * @param avoidanceRadius Safety margin around obstacles (m)
     * @param avoidanceStrength Base repulsion force
     * @param goalAttraction Goal pull strength
     * @param predictionTime Collision prediction look-ahead (seconds)
     * @param opponentAvoidanceMultiplier Opponent avoidance strength (lower = allow more contact)
     */
    public ObstacleAvoidanceNavigator(double navSpeedMps, double avoidanceRadius,
                                     double avoidanceStrength, double goalAttraction,
                                     double predictionTime, double opponentAvoidanceMultiplier) {
        this.navSpeedMps = navSpeedMps;
        this.avoidanceRadius = avoidanceRadius;
        this.avoidanceStrength = avoidanceStrength;
        this.goalAttraction = goalAttraction;
        this.predictionTime = predictionTime;
        this.opponentAvoidanceMultiplier = opponentAvoidanceMultiplier;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // ========================================================================
    // DIFFICULTY PRESETS - Easy to Hard opponent navigation
    // ========================================================================

    /**
     * EASY: Slow, cautious navigation. Avoids everything.
     * Good for beginners or testing.
     */
    public static ObstacleAvoidanceNavigator easy() {
        return new ObstacleAvoidanceNavigator(
            3.0,  // Slow speed
            2.0,  // Large safety margins
            2.5,  // Strong avoidance
            3.0,  // Moderate goal pull
            2.0,  // Long prediction
            1.2   // Strong opponent avoidance
        );
    }

    /**
     * MEDIUM: Balanced navigation. Good overall performance.
     * Default competitive setting.
     */
    public static ObstacleAvoidanceNavigator medium() {
        return new ObstacleAvoidanceNavigator(
            5.0,  // Moderate speed
            1.2,  // Moderate safety margins
            1.5,  // Balanced avoidance
            8.0,  // Strong goal pull
            1.2,  // Standard prediction
            0.6   // Light opponent avoidance (allows some bumping)
        );
    }

    /**
     * HARD: Fast, aggressive navigation. Cuts it close.
     * For experienced drivers or risky plays.
     */
    public static ObstacleAvoidanceNavigator hard() {
        return new ObstacleAvoidanceNavigator(
            7.0,  // Fast speed
            0.8,  // Tight safety margins
            1.2,  // Weaker avoidance
            12.0, // Very strong goal pull (dominant force)
            0.8,  // Short prediction (reactive)
            0.3   // Minimal opponent avoidance (bumps are fine)
        );
    }

    // ========================================================================
    // DRIVE STYLE PRESETS - Different navigation behaviors
    // ========================================================================

    /**
     * AGGRESSIVE: Maximum speed, minimal avoidance. Push through obstacles.
     * High risk, high reward.
     */
    public static ObstacleAvoidanceNavigator aggressive() {
        return new ObstacleAvoidanceNavigator(
            8.0,  // Very fast
            0.6,  // Minimal safety margin
            1.0,  // Weak avoidance
            15.0, // Extremely strong goal pull
            0.6,  // Very short prediction
            0.2   // Almost no opponent avoidance
        );
    }

    /**
     * DEFENSIVE: Prioritizes safety. Gives obstacles wide berth.
     * Slow but reliable.
     */
    public static ObstacleAvoidanceNavigator defensive() {
        return new ObstacleAvoidanceNavigator(
            3.5,  // Slow speed
            2.5,  // Very large safety margins
            3.0,  // Very strong avoidance
            2.0,  // Weak goal pull (safety first)
            2.5,  // Very long prediction
            1.5   // Very strong opponent avoidance
        );
    }

    /**
     * PRECISE: Moderate speed, careful navigation. Smooth paths.
     * Good for narrow spaces or delicate maneuvers.
     */
    public static ObstacleAvoidanceNavigator precise() {
        return new ObstacleAvoidanceNavigator(
            4.0,  // Moderate speed
            1.5,  // Good safety margins
            2.0,  // Strong avoidance
            5.0,  // Balanced goal pull
            1.5,  // Good prediction
            0.8   // Moderate opponent avoidance
        );
    }

    /**
     * RALLY: High speed with good control. Drifts around obstacles.
     * Fast but maintains safety.
     */
    public static ObstacleAvoidanceNavigator rally() {
        return new ObstacleAvoidanceNavigator(
            6.5,  // High speed
            1.0,  // Tight margins
            1.8,  // Good avoidance
            10.0, // Strong goal pull
            1.0,  // Moderate prediction
            0.5   // Light opponent avoidance
        );
    }

    /**
     * BULLDOZER: Moderate speed, ignores opponents. Walls matter.
     * Will push through other robots to reach goal.
     */
    public static ObstacleAvoidanceNavigator bulldozer() {
        return new ObstacleAvoidanceNavigator(
            5.5,  // Moderate speed
            1.2,  // Standard margins for walls
            1.5,  // Standard wall avoidance
            10.0, // Strong goal pull
            1.0,  // Standard prediction
            0.1   // Minimal opponent avoidance (push through!)
        );
    }

    /**
     * Calculate ChassisSpeeds to navigate to goal while avoiding obstacles
     * @param currentPose Current robot pose
     * @param goalPose Target pose
     * @param staticObstacles List of static obstacles (walls, reefs, etc)
     * @param opponents List of opponent robots (dynamic obstacles)
     * @return ChassisSpeeds for robot to follow
     */
    public ChassisSpeeds navigate(Pose2d currentPose, Pose2d goalPose,
                                 List<ObstacleAvoidance.Obstacle> staticObstacles,
                                 List<SmartOpponent> opponents) {
        // Combine static and dynamic obstacles
        obstacles.clear();
        obstacles.addAll(staticObstacles);

        // Add opponents as dynamic obstacles
        for (SmartOpponent opponent : opponents) {
            ObstacleAvoidance.Obstacle opponentObstacle = ObstacleAvoidance.robot(
                opponent.getPose(), new Translation2d(), true
            );
            opponentObstacle.avoidanceWeight *= opponentAvoidanceMultiplier;  // Apply tunable opponent avoidance
            obstacles.add(opponentObstacle);
        }

        // Configure APF
        ObstacleAvoidance.Config config = new ObstacleAvoidance.Config();
        config.maxVelocity = MetersPerSecond.of(navSpeedMps);
        config.baseAvoidanceStrength = avoidanceStrength;
        config.defaultAvoidanceRadius = Meters.of(avoidanceRadius);
        config.goalBias = goalAttraction;
        config.predictionLookAhead = predictionTime;
        config.useCollisionPrediction = true;
        config.useVelocityAwareAvoidance = true;

        // Calculate navigation speeds
        return controller.drive(
            currentPose,
            goalPose,
            obstacles,
            rotationPID,
            config,
            new Translation2d()
        );
    }

    /**
     * Create field boundary obstacles as circles
     * @param wallRadius Radius of wall obstacle circles (m)
     * @param wallSpacing Spacing between circles (m)
     * @param wallWeight Wall avoidance strength
     * @return List of wall obstacles
     */
    public static List<ObstacleAvoidance.Obstacle> createFieldBoundaries(
            double wallRadius, double wallSpacing, double wallWeight) {
        List<ObstacleAvoidance.Obstacle> walls = new ArrayList<>();

        // Left wall (vertical at x=0.8)
        for (double y = 0; y <= 8.052; y += wallSpacing) {
            ObstacleAvoidance.Obstacle wall = ObstacleAvoidance.circle(
                new Translation2d(0.8, y), Meters.of(wallRadius)
            );
            wall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
            wall.avoidanceWeight = wallWeight;
            wall.priority = 1.0;
            walls.add(wall);
        }

        // Right wall (vertical at x=16.748)
        for (double y = 0; y <= 8.052; y += wallSpacing) {
            ObstacleAvoidance.Obstacle wall = ObstacleAvoidance.circle(
                new Translation2d(16.748, y), Meters.of(wallRadius)
            );
            wall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
            wall.avoidanceWeight = wallWeight;
            wall.priority = 1.0;
            walls.add(wall);
        }

        // Bottom wall (horizontal at y=0.8)
        for (double x = 0; x <= 17.548; x += wallSpacing) {
            ObstacleAvoidance.Obstacle wall = ObstacleAvoidance.circle(
                new Translation2d(x, 0.8), Meters.of(wallRadius)
            );
            wall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
            wall.avoidanceWeight = wallWeight;
            wall.priority = 1.0;
            walls.add(wall);
        }

        // Top wall (horizontal at y=7.252)
        for (double x = 0; x <= 17.548; x += wallSpacing) {
            ObstacleAvoidance.Obstacle wall = ObstacleAvoidance.circle(
                new Translation2d(x, 7.252), Meters.of(wallRadius)
            );
            wall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
            wall.avoidanceWeight = wallWeight;
            wall.priority = 1.0;
            walls.add(wall);
        }

        return walls;
    }

    /**
     * Create Reefscape 2025 field obstacles
     * @param reefRadius Reef obstacle radius (m)
     * @param reefWeight Reef avoidance strength
     * @return List of reef and pillar obstacles
     */
    public static List<ObstacleAvoidance.Obstacle> createReefscapeObstacles(
            double reefRadius, double reefWeight) {
        List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

        // Blue Reef
        ObstacleAvoidance.Obstacle blueReef = ObstacleAvoidance.circle(
            new Translation2d(4.489, 4.026), Meters.of(reefRadius)
        );
        blueReef.avoidanceWeight = reefWeight;
        obstacles.add(blueReef);

        // Red Reef
        ObstacleAvoidance.Obstacle redReef = ObstacleAvoidance.circle(
            new Translation2d(13.059, 4.026), Meters.of(reefRadius)
        );
        redReef.avoidanceWeight = reefWeight;
        obstacles.add(redReef);

        // Center pillar
        ObstacleAvoidance.Obstacle pillar = ObstacleAvoidance.circle(
            new Translation2d(8.774, 4.026), Meters.of(reefRadius * 0.6)
        );
        pillar.avoidanceWeight = reefWeight;
        obstacles.add(pillar);

        return obstacles;
    }
}
