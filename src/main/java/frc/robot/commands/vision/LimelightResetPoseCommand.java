package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;

import static frc.robot.util.Util.log;

/**
 * Resets the current pose of the swerve drive using the latest vision
 * pose estimate. This is insanely useful in testing on a practice field,
 * either at school or at a competition.
 */
public class LimelightResetPoseCommand extends Command{

    final SwerveDriveSubsystem drive;
    final LimelightSubsystem limelight;

    public LimelightResetPoseCommand(SwerveDriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;
        addRequirements(drive, limelight);
    }

    @Override
    public void execute() {

        Pose2d pose = limelight.getCurrentPose();
        if (pose != null) {
            log("[ll-pose] resetting pose to %s", pose);
            drive.resetPose(pose);
        } else {
            log("[ll-pose] can't reset pose - no tag in view");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
