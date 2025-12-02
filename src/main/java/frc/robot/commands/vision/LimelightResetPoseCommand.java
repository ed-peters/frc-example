package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;

import java.util.function.Consumer;

import static frc.robot.util.Util.log;

/**
 * Resets the current pose of the swerve drive using the latest vision
 * pose estimate. This is insanely useful in testing on a practice field,
 * either at school or at a competition.</p>
 *
 * This command is implemented so it doesn't depend on a specific swerve
 * drive implementation.</p>
 */
public class LimelightResetPoseCommand extends Command{

    final Consumer<Pose2d> poseConsumer;
    final LimelightSubsystem limelight;

    public LimelightResetPoseCommand(LimelightSubsystem limelight,
                                     Subsystem driveSubsystem,
                                     Consumer<Pose2d> poseConsumer) {
        this.poseConsumer = poseConsumer;
        this.limelight = limelight;
        addRequirements(driveSubsystem, limelight);
    }

    @Override
    public void execute() {

        Pose2d pose = limelight.getCurrentPose();
        if (pose != null) {
            log("[ll-pose] resetting pose to %s", pose);
            poseConsumer.accept(pose);
        } else {
            log("[ll-pose] can't reset pose - no tag in view");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
