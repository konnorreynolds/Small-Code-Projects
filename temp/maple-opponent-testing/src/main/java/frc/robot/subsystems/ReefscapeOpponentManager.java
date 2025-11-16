package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.opponentsim.OpponentManager;
import org.ironmaple.simulation.opponentsim.SmartOpponent;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

/**
 * Custom OpponentManager for Reefscape that properly initializes opponent lists
 * and provides obstacle-aware navigation for opponents.
 */
public class ReefscapeOpponentManager extends OpponentManager {

    // Shared obstacles for all opponents
    private static List<ObstacleAvoidance.Obstacle> sharedStaticObstacles = new ArrayList<>();

    // Default navigator for opponents (can be customized per opponent)
    private final CompactNavigator defaultOpponentNavigator = new CompactNavigator();

    public ReefscapeOpponentManager() {
        super();
        // Initialize the Optional lists that the base class doesn't initialize
        this.redOpponents = Optional.of(new ArrayList<>());
        this.blueOpponents = Optional.of(new ArrayList<>());
    }

    /**
     * Set shared obstacles that all opponents will use for navigation
     */
    public void setSharedObstacles(List<ObstacleAvoidance.Obstacle> obstacles) {
        sharedStaticObstacles = new ArrayList<>(obstacles);
    }

    /**
     * Get shared obstacles list (for adding more obstacles)
     */
    public List<ObstacleAvoidance.Obstacle> getSharedObstacles() {
        return sharedStaticObstacles;
    }

    /**
     * Navigate opponent to target using obstacle avoidance
     * @param opponent The opponent to navigate
     * @param targetPose Target destination
     * @param navigator Custom navigator (null to use default)
     * @return ChassisSpeeds for the opponent
     */
    public ChassisSpeeds navigateOpponent(SmartOpponent opponent, Pose2d targetPose,
                                          CompactNavigator navigator) {
        // Use default navigator if none provided
        CompactNavigator nav = (navigator != null) ? navigator : defaultOpponentNavigator;

        // Get other opponents for dynamic avoidance
        List<SmartOpponent> otherOpponents = new ArrayList<>();
        for (SmartOpponent opp : getOpponents()) {
            if (opp != opponent) {
                otherOpponents.add(opp);
            }
        }

        // Navigate with obstacle avoidance using PIDController-like API
        // Note: We need to add obstacles to the navigator first
        nav.clearObstacles();
        nav.addObstacles(sharedStaticObstacles);

        return nav.calculate(opponent.getPose(), targetPose, otherOpponents);
    }

    @Override
    protected Pose2d getStartingPose(DriverStation.Alliance alliance, int id) {
        // Red alliance starting poses
        if (alliance == DriverStation.Alliance.Red) {
            return switch (id) {
                case 1 -> new Pose2d(new Translation2d(14, 2), Rotation2d.fromDegrees(180));
                case 2 -> new Pose2d(new Translation2d(14, 6), Rotation2d.fromDegrees(180));
                default -> new Pose2d(new Translation2d(14, 4), Rotation2d.fromDegrees(180));
            };
        }
        // Blue alliance starting poses
        return switch (id) {
            case 1 -> new Pose2d(new Translation2d(3, 2), Rotation2d.kZero);
            case 2 -> new Pose2d(new Translation2d(3, 6), Rotation2d.kZero);
            default -> new Pose2d(new Translation2d(3, 4), Rotation2d.kZero);
        };
    }

    @Override
    protected Pose2d getQueeningPose(DriverStation.Alliance alliance, int id) {
        // Queening pose (off-field position when disabled)
        if (alliance == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(16, 1 + id * 2), Rotation2d.fromDegrees(180));
        }
        return new Pose2d(new Translation2d(1.5, 1 + id * 2), Rotation2d.kZero);
    }

    @Override
    protected Pose2d getNextCollectTarget(DriverStation.Alliance alliance) {
        // Simple collect target (center-ish of field)
        if (alliance == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(12, 4), Rotation2d.fromDegrees(180));
        }
        return new Pose2d(new Translation2d(5.5, 4), Rotation2d.kZero);
    }

    @Override
    protected Pose2d getNextScoreTarget(DriverStation.Alliance alliance) {
        // Simple score target (near reef)
        if (alliance == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(13, 4), Rotation2d.fromDegrees(180));
        }
        return new Pose2d(new Translation2d(4.5, 4), Rotation2d.kZero);
    }

    /**
     * Create a pathfinding command for an opponent using obstacle avoidance
     * @param opponent The opponent robot
     * @param targetPose Target position
     * @param navigator Custom navigator (null for default)
     * @return Command that navigates the opponent
     */
    public Command createNavigationCommand(SmartOpponent opponent, Pose2d targetPose,
                                          CompactNavigator navigator) {
        return Commands.run(() -> {
            ChassisSpeeds speeds = navigateOpponent(opponent, targetPose, navigator);

            // Apply speeds to opponent simulation if available
            if (opponent instanceof org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.KitBot kitBot) {
                // KitBot has access to simulation - apply speeds directly
                // This would need the simulation field to be accessible
                // For now, opponents will use their default pathfinding
            }
        }, opponent);
    }
}
