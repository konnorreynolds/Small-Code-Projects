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
 */
public class ObstacleAvoidanceNavigator {
    // Tunable Parameters
    private final double navSpeedMps;
    private final double avoidanceRadius;
    private final double avoidanceStrength;
    private final double goalAttraction;
    private final double predictionTime;

    private final ObstacleAvoidance controller = new ObstacleAvoidance();
    private final PIDController rotationPID = new PIDController(1.5, 0, 0.1);
    private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

    /**
     * Create navigator with default balanced settings
     */
    public ObstacleAvoidanceNavigator() {
        this(5.0, 1.2, 1.5, 8.0, 1.2);
    }

    /**
     * Create navigator with custom settings
     * @param navSpeedMps Navigation speed (m/s)
     * @param avoidanceRadius Safety margin around obstacles (m)
     * @param avoidanceStrength Base repulsion force
     * @param goalAttraction Goal pull strength
     * @param predictionTime Collision prediction look-ahead (seconds)
     */
    public ObstacleAvoidanceNavigator(double navSpeedMps, double avoidanceRadius,
                                     double avoidanceStrength, double goalAttraction,
                                     double predictionTime) {
        this.navSpeedMps = navSpeedMps;
        this.avoidanceRadius = avoidanceRadius;
        this.avoidanceStrength = avoidanceStrength;
        this.goalAttraction = goalAttraction;
        this.predictionTime = predictionTime;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
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
            opponentObstacle.avoidanceWeight *= 1.0;  // Moderate opponent avoidance
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
