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
 * Compact obstacle-avoiding navigator. Use like PIDController.
 * Example: nav.addObstacles(CompactNavigator.fieldWalls());
 *          speeds = nav.calculate(currentPose, targetPose, opponents);
 */
public class CompactNavigator {
    private final double maxSpeed, goalAttraction, avoidanceStrength, opponentWeight;
    private final ObstacleAvoidance controller = new ObstacleAvoidance();
    private final PIDController rotationPID = new PIDController(1.5, 0, 0.1);
    private final List<ObstacleAvoidance.Obstacle> obstacles = new ArrayList<>();

    public CompactNavigator(double maxSpeed, double goalAttraction,
                           double avoidanceStrength, double opponentWeight) {
        this.maxSpeed = maxSpeed;
        this.goalAttraction = goalAttraction;
        this.avoidanceStrength = avoidanceStrength;
        this.opponentWeight = opponentWeight;
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public CompactNavigator() { this(5.0, 8.0, 1.5, 0.6); }

    // Presets
    public static CompactNavigator aggressive() { return new CompactNavigator(7.0, 12.0, 1.2, 0.3); }
    public static CompactNavigator defensive() { return new CompactNavigator(3.5, 2.0, 3.0, 1.5); }
    public static CompactNavigator rally() { return new CompactNavigator(6.5, 10.0, 1.8, 0.5); }
    public static CompactNavigator bulldozer() { return new CompactNavigator(5.5, 10.0, 1.5, 0.1); }

    // Main API - like PIDController.calculate()
    public ChassisSpeeds calculate(Pose2d current, Pose2d target, List<SmartOpponent> opponents) {
        List<ObstacleAvoidance.Obstacle> all = new ArrayList<>(obstacles);
        for (SmartOpponent opp : opponents) {
            ObstacleAvoidance.Obstacle o = ObstacleAvoidance.robot(opp.getPose(), new Translation2d(), true);
            o.avoidanceWeight *= opponentWeight;
            all.add(o);
        }
        ObstacleAvoidance.Config cfg = new ObstacleAvoidance.Config();
        cfg.maxVelocity = Meters.per(edu.wpi.first.units.Units.Second).of(maxSpeed);
        cfg.baseAvoidanceStrength = avoidanceStrength;
        cfg.defaultAvoidanceRadius = Meters.of(1.2);
        cfg.goalBias = goalAttraction;
        cfg.predictionLookAhead = 1.2;
        cfg.useCollisionPrediction = true;
        cfg.useVelocityAwareAvoidance = true;
        return controller.drive(current, target, all, rotationPID, cfg, new Translation2d());
    }

    public ChassisSpeeds calculate(Pose2d current, Pose2d target) {
        return calculate(current, target, new ArrayList<>());
    }

    // Obstacle management
    public void addObstacles(List<ObstacleAvoidance.Obstacle> obs) { obstacles.addAll(obs); }
    public void clearObstacles() { obstacles.clear(); }
    public int getObstacleCount() { return obstacles.size(); }

    // Obstacle factories
    public static List<ObstacleAvoidance.Obstacle> fieldWalls() {
        List<ObstacleAvoidance.Obstacle> w = new ArrayList<>();
        for (double y = 0; y <= 8.052; y += 0.8) {
            w.add(wall(0.8, y, 0.35)); w.add(wall(16.748, y, 0.35));
        }
        for (double x = 0; x <= 17.548; x += 0.8) {
            w.add(wall(x, 0.8, 0.35)); w.add(wall(x, 7.252, 0.35));
        }
        return w;
    }

    public static List<ObstacleAvoidance.Obstacle> circle(Translation2d center, double radius) {
        List<ObstacleAvoidance.Obstacle> c = new ArrayList<>();
        ObstacleAvoidance.Obstacle o = ObstacleAvoidance.circle(center, Meters.of(radius));
        o.avoidanceWeight = 2.0;
        c.add(o);
        return c;
    }

    public static List<ObstacleAvoidance.Obstacle> reefscapeObstacles() {
        List<ObstacleAvoidance.Obstacle> obs = new ArrayList<>();
        obs.addAll(circle(new Translation2d(4.489, 4.026), 0.9));   // Blue reef
        obs.addAll(circle(new Translation2d(13.059, 4.026), 0.9));  // Red reef
        obs.addAll(circle(new Translation2d(8.774, 4.026), 0.54));  // Pillar
        return obs;
    }

    private static ObstacleAvoidance.Obstacle wall(double x, double y, double r) {
        ObstacleAvoidance.Obstacle w = ObstacleAvoidance.circle(new Translation2d(x, y), Meters.of(r));
        w.type = ObstacleAvoidance.Obstacle.Type.DANGEROUS;
        w.avoidanceWeight = 2.0;
        w.priority = 1.0;
        return w;
    }

    public boolean atTarget(Pose2d current, Pose2d target, double tol) {
        return current.getTranslation().getDistance(target.getTranslation()) < tol;
    }
}
