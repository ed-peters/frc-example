package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.swerve.SwervePoseCalculator;
import frc.robot.util.swerve.SwervePoseCalculator.PoseType;

import java.util.function.Consumer;

import static frc.robot.util.Util.log;

/**
 * Resets the current pose of the swerve drive using the latest vision pose
 * estimate from a {@link SwervePoseCalculator}. This is insanely useful in
 * testing on a practice field, either at school or at a competition.</p>
 *
 * This command is implemented so it doesn't depend on a specific swerve
 * drive implementation or pose estimator.</p>
 */
public class SwerveResetVisionPoseCommand extends Command{

    final SwervePoseCalculator poseCalculator;
    final Consumer<Pose2d> poseConsumer;

    public SwerveResetVisionPoseCommand(SwervePoseCalculator poseCalculator,
                                        Consumer<Pose2d> poseConsumer) {

        this.poseCalculator = poseCalculator;
        this.poseConsumer = poseConsumer;

        // unlike most commands, we won't actually depend on a subsystem;
        // we don't need to consume a full scheduler cycle to do this
    }

    @Override
    public void execute() {
        Pose2d pose = poseCalculator.getLatestPoseEstimate(PoseType.VISION);
        if (pose != null) {
            log("[swerve] resetting pose to vision pose %s", pose);
            poseConsumer.accept(pose);
        } else {
            log("[swerve] can't reset pose from vision - no estimate available");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
