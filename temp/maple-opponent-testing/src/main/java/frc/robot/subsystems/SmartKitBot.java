package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim.KitBot;

import static edu.wpi.first.units.Units.Meters;

/**
 * Enhanced KitBot that uses ObstacleAvoidanceNavigator for intelligent pathfinding.
 * Extends the base KitBot with obstacle-aware navigation.
 */
public class SmartKitBot extends KitBot {

    private final ReefscapeOpponentManager smartManager;
    private final ObstacleAvoidanceNavigator navigator;

    /**
     * Create a SmartKitBot with default medium difficulty navigation
     */
    public SmartKitBot(ReefscapeOpponentManager manager, DriverStation.Alliance alliance, int id) {
        this(manager, alliance, id, ObstacleAvoidanceNavigator.medium());
    }

    /**
     * Create a SmartKitBot with custom navigation behavior
     * @param manager Opponent manager
     * @param alliance Robot alliance
     * @param id Robot ID
     * @param navigator Custom navigator (easy, medium, hard, or custom style)
     */
    public SmartKitBot(ReefscapeOpponentManager manager, DriverStation.Alliance alliance, int id,
                       ObstacleAvoidanceNavigator navigator) {
        super(manager, alliance, id);
        this.smartManager = manager;
        this.navigator = navigator;
    }

    /**
     * Override pathfinding to use obstacle avoidance navigator
     */
    @Override
    public Command pathfindCommand(Pose2d targetPose, Distance tolerance) {
        if (!simulation.isPresent()) {
            return Commands.runOnce(() -> DriverStation.reportWarning("No simulation found", false), this);
        }

        return Commands.run(() -> {
            Pose2d robotPose = simulation.get().getActualPoseInSimulationWorld();

            // Use ObstacleAvoidanceNavigator for intelligent pathfinding
            ChassisSpeeds speeds = smartManager.navigateOpponent(this, targetPose, navigator);

            // Apply the speeds to the simulation
            simulation.get().runChassisSpeeds(speeds, new Translation2d(), false, false);

            // Check if we've reached the target
            double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            if (distance <= tolerance.in(Meters)) {
                // Stop when reached
                simulation.get().runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false);
            }

        }, this).until(() -> {
            if (simulation.isEmpty()) return true;
            return simulation.get().getActualPoseInSimulationWorld().getTranslation()
                    .getDistance(targetPose.getTranslation()) <= tolerance.in(Meters);
        });
    }

    /**
     * Get the navigator being used by this opponent
     */
    public ObstacleAvoidanceNavigator getNavigator() {
        return navigator;
    }
}
