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
 * Enhanced KitBot that uses CompactNavigator for intelligent pathfinding.
 * Extends the base KitBot with obstacle-aware navigation.
 */
public class SmartKitBot extends KitBot {

    private final ReefscapeOpponentManager smartManager;
    private final CompactNavigator navigator;

    /**
     * Create a SmartKitBot with default balanced navigation
     */
    public SmartKitBot(ReefscapeOpponentManager manager, DriverStation.Alliance alliance, int id) {
        this(manager, alliance, id, new CompactNavigator());
    }

    /**
     * Create a SmartKitBot with custom navigation behavior
     * @param manager Opponent manager
     * @param alliance Robot alliance
     * @param id Robot ID
     * @param navigator Custom navigator (use presets like CompactNavigator.aggressive() or custom)
     */
    public SmartKitBot(ReefscapeOpponentManager manager, DriverStation.Alliance alliance, int id,
                       CompactNavigator navigator) {
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
    public CompactNavigator getNavigator() {
        return navigator;
    }
}
