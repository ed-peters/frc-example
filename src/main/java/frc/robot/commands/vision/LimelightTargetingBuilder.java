package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.swerve.SwerveAutoPoseCommand;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightTarget;
import frc.robot.util.Util;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * This acts as a factory for various commands that implement different ways
 * of using the Limelight for driving the robot to a specific target position
 * on the field.</p>
 *
 * It is largely inspired by our 2025 Reefscape targeting process, with some
 * improvements.
 */
public class LimelightTargetingBuilder {

    // these are used to construct the various commands
    final LimelightSubsystem limelight;
    final Subsystem driveSubsystem;
    final Supplier<Pose2d> poseSupplier;
    final Consumer<ChassisSpeeds> speedConsumer;

    public LimelightTargetingBuilder(LimelightSubsystem limelight,
                                     Subsystem driveSubsystem,
                                     Supplier<Pose2d> poseSupplier,
                                     Consumer<ChassisSpeeds> speedConsumer) {
        this.driveSubsystem = driveSubsystem;
        this.limelight = limelight;
        this.poseSupplier = poseSupplier;
        this.speedConsumer = speedConsumer;
    }

    /**
     * Creates a command that will use visual servo logic to center the robot
     * in front of the currently in-view AprilTag, and optionally drive to an
     * offset from that position. This is basically our 2025 Reefscape
     * targeting logic.
     */
    public Command twoStageTargetingCommand(Translation2d offset) {

        Command print = Commands.print("[ll-target] executing two-stage targeting");

        // use the visual servo approach to align as accurately as possible to
        // where the current tag is in the real world
        Command servo = new LimelightVisualServoCommand(
                limelight,
                driveSubsystem,
                speedConsumer);

        // if there is no offset, this is all we'll do
        if (offset == null) {
            return print.andThen(servo);
        }

        // this will drive us to the offset position. we defer the creation of
        // the command in case the servo command fails and we finish with the
        // tag out-of-view
        Command driveToOffset = driveSubsystem.defer(() -> {

            if (limelight.getCurrentTarget() == LimelightTarget.NO_TARGET) {
                Util.log("[ll-target] !!! NO TAG IN VIEW !!!");
                return Commands.none();
            }

            return relativePoseCommand(currentPose -> new Pose2d(
                    currentPose.getTranslation().plus(offset),
                    currentPose.getRotation()));
        });

        return print
                .andThen(servo)
                .andThen(driveToOffset);
    }

    /**
     * Creates a command that will drive the robot in front of the specified
     * tag based purely on odometry, and then execute the two-stage targeting
     * logic above. This is superior because you can start with no tag in
     * view.
     */
    public Command threeStageTargetingCommand(AprilTag tag,
                                              double feetInFrontOfTag,
                                              Translation2d offset) {

        Pose2d tagPose = tag.pose.toPose2d();

        // this transformation will produce a starting pose which is in front
        // of the tag by the specified number of feet, and pointing back to
        // look at the tag
        Transform2d transform = new Transform2d(
                new Translation2d(Units.feetToMeters(feetInFrontOfTag), 0.0),
                Rotation2d.k180deg);

        Command print = Commands.print("[ll-target] driving to tag "+tag.ID);

        // drive to the starting pose and then do two-stage targeting
        return print
                .andThen(absolutePoseCommand(tagPose.transformBy(transform)))
                .andThen(twoStageTargetingCommand(offset));
    }

    /**
     * Creates a command to drive to an absolute position on the field
     */
    private Command absolutePoseCommand(Pose2d pose) {
        return new SwerveAutoPoseCommand(
                driveSubsystem,
                poseSupplier,
                speedConsumer,
                pose);
    }

    /**
     * Creates a command that will drive to a new pose which is based on
     * transforming the current pose of the robot when the command starts
     */
    private Command relativePoseCommand(Function<Pose2d,Pose2d> poseFunction) {
        return driveSubsystem.defer(() -> {
            Pose2d oldPose = poseSupplier.get();
            Pose2d newPose = poseFunction.apply(oldPose);
            return absolutePoseCommand(newPose);
        });
    }
}
