package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.opponentsim.SmartOpponent;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Meters;

/**
 * Simple obstacle-avoiding navigation controller.
 * Acts like a PIDController but returns ChassisSpeeds that avoid obstacles.
 *
 * <pre>
 * // Create navigator
 * var nav = new CompactNavigator(5.0, 8.0, 1.5);
 *
 * // Add obstacles
 * nav.addObstacles(CompactNavigator.fieldWalls());
 * nav.addObstacles(CompactNavigator.circles(new Translation2d(4, 4), 1.0, 3));
 *
 * // Navigate (like PIDController.calculate)
 * ChassisSpeeds speeds = nav.calculate(currentPose, targetPose, opponents);
 * drive.setRobotRelativeChassisSpeeds(speeds);
 * </pre>
 */
public class CompactNavigator {
    // Navigation parameters
    private final double maxSpeed;           // m/s
    private final double goalAttraction;     // Goal pull strength
    private final double avoidanceStrength;  // Obstacle repulsion strength
    private final double opponentWeight;     // Opponent avoidance (0.0 = ignore, 1.0 = avoid)

    // Internal state
    private final ObstacleAvoidance controller = new ObstacleAvoidance();
    private final PIDController rotationPID = new PIDController(1.5, 0, 0.1);
    private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

    /**
     * Create navigator with custom parameters
     * @param maxSpeed Maximum navigation speed (m/s)
     * @param goalAttraction How strongly to pull toward goal (higher = more aggressive)
     * @param avoidanceStrength How strongly to avoid obstacles (higher = wider berth)
     * @param opponentWeight How much to avoid opponents (0.0 = ignore, 1.0 = avoid strongly)
     */
    public CompactNavigator(double maxSpeed, double goalAttraction,
                           double avoidanceStrength, double opponentWeight) {
        this.maxSpeed = maxSpeed;
        this.goalAttraction = goalAttraction;
        this.avoidanceStrength = avoidanceStrength;
        this.opponentWeight = opponentWeight;
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Create navigator with standard settings (5 m/s, balanced)
     */
    public CompactNavigator() {
        this(5.0, 8.0, 1.5, 0.6);
    }

    // ========================================================================
    // PRESETS - Quick configurations
    // ========================================================================

    /** Fast aggressive navigation (7 m/s, minimal avoidance) */
    public static CompactNavigator aggressive() { return new CompactNavigator(7.0, 12.0, 1.2, 0.3); }

    /** Safe defensive navigation (3.5 m/s, strong avoidance) */
    public static CompactNavigator defensive() { return new CompactNavigator(3.5, 2.0, 3.0, 1.5); }

    /** High-speed rally (6.5 m/s, balanced) */
    public static CompactNavigator rally() { return new CompactNavigator(6.5, 10.0, 1.8, 0.5); }

    /** Bulldozer - ignores opponents (5.5 m/s) */
    public static CompactNavigator bulldozer() { return new CompactNavigator(5.5, 10.0, 1.5, 0.1); }

    // ========================================================================
    // MAIN API - Like PIDController
    // ========================================================================

    /**
     * Calculate ChassisSpeeds to navigate to target while avoiding obstacles.
     * Call this repeatedly like PIDController.calculate()
     *
     * @param currentPose Current robot pose
     * @param targetPose Target pose to reach
     * @param dynamicObstacles Live opponents/robots to avoid (can be empty)
     * @return ChassisSpeeds to follow
     */
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose,
                                   List<SmartOpponent> dynamicObstacles) {
        // Build combined obstacle list
        List<ObstacleAvoidance.Obstacle> allObstacles = new ArrayList<>(obstacles);

        // Add dynamic obstacles (opponents)
        for (SmartOpponent opponent : dynamicObstacles) {
            ObstacleAvoidance.Obstacle opp = ObstacleAvoidance.robot(
                opponent.getPose(), new Translation2d(), true
            );
            opp.avoidanceWeight *= opponentWeight;
            allObstacles.add(opp);
        }

        // Configure APF
        ObstacleAvoidance.Config config = new ObstacleAvoidance.Config();
        config.maxVelocity = Meters.per(edu.wpi.first.units.Units.Second).of(maxSpeed);
        config.baseAvoidanceStrength = avoidanceStrength;
        config.defaultAvoidanceRadius = Meters.of(1.2);
        config.goalBias = goalAttraction;
        config.predictionLookAhead = 1.2;
        config.useCollisionPrediction = true;
        config.useVelocityAwareAvoidance = true;

        // Calculate navigation
        return controller.drive(currentPose, targetPose, allObstacles, rotationPID, config, new Translation2d());
    }

    /**
     * Calculate without dynamic obstacles
     */
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
        return calculate(currentPose, targetPose, new ArrayList<>());
    }

    // ========================================================================
    // OBSTACLE MANAGEMENT
    // ========================================================================

    /** Add obstacles to the navigator */
    public void addObstacles(List<ObstacleAvoidance.Obstacle> newObstacles) {
        obstacles.addAll(newObstacles);
    }

    /** Clear all obstacles */
    public void clearObstacles() {
        obstacles.clear();
    }

    /** Get current obstacle count */
    public int getObstacleCount() {
        return obstacles.size();
    }

    // ========================================================================
    // OBSTACLE FACTORIES - Easy obstacle creation
    // ========================================================================

    /**
     * Create standard FRC field boundary walls (17.548m x 8.052m)
     * @param thickness Wall thickness (recommended: 0.35-0.5)
     * @param spacing Gap between wall circles (recommended: 0.6-1.0)
     * @return List of wall obstacles
     */
    public static List<ObstacleAvoidance.Obstacle> fieldWalls(double thickness, double spacing) {
        List<ObstacleAvoidance.Obstacle> walls = new ArrayList<>();
        double weight = 2.0;

        // Create walls as series of circles
        for (double y = 0; y <= 8.052; y += spacing) {
            walls.add(makeWall(0.8, y, thickness, weight));           // Left wall
            walls.add(makeWall(16.748, y, thickness, weight));        // Right wall
        }
        for (double x = 0; x <= 17.548; x += spacing) {
            walls.add(makeWall(x, 0.8, thickness, weight));           // Bottom wall
            walls.add(makeWall(x, 7.252, thickness, weight));         // Top wall
        }
        return walls;
    }

    /** Field walls with default settings (0.35m thick, 0.8m spacing) */
    public static List<ObstacleAvoidance.Obstacle> fieldWalls() {
        return fieldWalls(0.35, 0.8);
    }

    /**
     * Create circular obstacles (reefs, pillars, etc)
     * @param center Center position
     * @param radius Obstacle radius
     * @param count Number of circles to create (spread evenly if > 1)
     * @return List of circular obstacles
     */
    public static List<ObstacleAvoidance.Obstacle> circles(Translation2d center, double radius, int count) {
        List<ObstacleAvoidance.Obstacle> circles = new ArrayList<>();
        if (count == 1) {
            circles.add(makeCircle(center, radius, 2.0));
        } else {
            // Spread multiple circles
            for (int i = 0; i < count; i++) {
                double angle = 2 * Math.PI * i / count;
                double x = center.getX() + 0.5 * Math.cos(angle);
                double y = center.getY() + 0.5 * Math.sin(angle);
                circles.add(makeCircle(new Translation2d(x, y), radius, 2.0));
            }
        }
        return circles;
    }

    /** Single circular obstacle */
    public static List<ObstacleAvoidance.Obstacle> circle(Translation2d center, double radius) {
        return circles(center, radius, 1);
    }

    /**
     * Create Reefscape 2025 field obstacles (2 reefs + pillar)
     */
    public static List<ObstacleAvoidance.Obstacle> reefscapeObstacles() {
        List<ObstacleAvoidance.Obstacle> obs = new ArrayList<>();
        obs.add(makeCircle(new Translation2d(4.489, 4.026), 0.9, 2.0));   // Blue reef
        obs.add(makeCircle(new Translation2d(13.059, 4.026), 0.9, 2.0));  // Red reef
        obs.add(makeCircle(new Translation2d(8.774, 4.026), 0.54, 2.0));  // Center pillar
        return obs;
    }

    // Helper methods
    private static ObstacleAvoidance.Obstacle makeWall(double x, double y, double radius, double weight) {
        ObstacleAvoidance.Obstacle wall = ObstacleAvoidance.circle(new Translation2d(x, y), Meters.of(radius));
        wall.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
        wall.avoidanceWeight = weight;
        wall.priority = 1.0;
        return wall;
    }

    private static ObstacleAvoidance.Obstacle makeCircle(Translation2d pos, double radius, double weight) {
        ObstacleAvoidance.Obstacle circle = ObstacleAvoidance.circle(pos, Meters.of(radius));
        circle.avoidanceWeight = weight;
        return circle;
    }

    // ========================================================================
    // UTILITY
    // ========================================================================

    /** Check if close to target (within tolerance) */
    public boolean atTarget(Pose2d current, Pose2d target, double toleranceMeters) {
        return current.getTranslation().getDistance(target.getTranslation()) < toleranceMeters;
    }

    /** Get current max speed */
    public double getMaxSpeed() { return maxSpeed; }

    /** Get goal attraction strength */
    public double getGoalAttraction() { return goalAttraction; }

    /** Get avoidance strength */
    public double getAvoidanceStrength() { return avoidanceStrength; }
}
