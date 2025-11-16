package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.opponentsim.OpponentManager;
import org.ironmaple.simulation.opponentsim.SmartOpponent;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Custom OpponentManager for Reefscape that properly initializes opponent lists.
 */
public class ReefscapeOpponentManager extends OpponentManager {

    public ReefscapeOpponentManager() {
        super();
        // Initialize the Optional lists that the base class doesn't initialize
        this.redOpponents = Optional.of(new ArrayList<>());
        this.blueOpponents = Optional.of(new ArrayList<>());
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
}
