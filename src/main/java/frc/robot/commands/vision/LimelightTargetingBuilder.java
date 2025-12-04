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

import java.util.function.BooleanSupplier;
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
     * Implements our 2025 multi-stage targeting logic. This will:
     * <ul>
     *
     *     <li>Rotate the robot to the appropriate heading to face the
     *     supplied tag. If the tag slips out of view during this step,
     *     we will stop and not proceed to the next step.</li>
     *
     *     <li>Use "visual servoing" to align the robot to a repeatable
     *     position in front of the tag. Again, if the tag slips out of view
     *     while we are doing this, we will stop and not proceed. Then,</li>
     *
     *     <li>Drive to an optional offset from that point. We used this to
     *     implement a "score left / right" distinction based on the shape of
     *     the game field.</li>
     * </ul>
     */
    public Command reefscapeTargetingCommand(AprilTag tag,
                                             Translation2d offset) {

        Command print = Commands.print("[ll-target] reefscape targeting on tag "+tag.ID);

        // this is the heading that is directly facing the tag (we flip the
        // tag's heading by 180 degrees because the tag is facing "out" from
        // where it's mounted)
        Rotation2d facingHeading = tag.pose.toPose2d()
                .getRotation()
                .plus(Rotation2d.k180deg);

        // this rotates us to that heading, stopping if the tag slips out
        // of view while we're doing it
        Command rotate = relativePoseCommand(currentPose -> new Pose2d(
                currentPose.getTranslation(),
                facingHeading));

        // use the visual servo approach to align as accurately as possible to
        // where the current tag is in the real world
        Command servo = new LimelightVisualServoCommand(
                limelight,
                driveSubsystem,
                speedConsumer);

        // if there is no offset, this is all we'll do
        if (offset == null) {
            return print
                    .andThen(rotate)
                    .andThen(servo)
                    .onlyWhile(tagInView(tag));
        }

        // otherwise, we will drive to the offset position
        Command driveToOffset = relativePoseCommand(currentPose -> new Pose2d(
                currentPose.getTranslation().plus(offset),
                currentPose.getRotation()));

        return print
                .andThen(rotate)
                .andThen(servo)
                .andThen(driveToOffset)
                .onlyWhile(tagInView(tag));
    }

    /**
     * This is slightly better than the reefscape targeting approach. Instead
     * of just rotating to face the tag, we will drive to a specific position
     * in front of the tag. This would allow us to target a tag without even
     * having it in view.
     */
    public Command swankyTargetingCommand(AprilTag tag,
                                          double feetInFrontOfTag,
                                          Translation2d offset) {

        Command print = Commands.print("[ll-target] swanky targeting on tag "+tag.ID);

        // this transformation will produce a starting pose which is in front
        // of the tag by the specified number of feet, and pointing back to
        // look at the tag
        Transform2d transform = new Transform2d(
                new Translation2d(Units.feetToMeters(feetInFrontOfTag), 0.0),
                Rotation2d.k180deg);

        Command driveToStart = absolutePoseCommand(tag.pose.toPose2d()
                .transformBy(transform));

        // use the visual servo approach to align as accurately as possible to
        // where the current tag is in the real world
        Command servo = new LimelightVisualServoCommand(
                limelight,
                driveSubsystem,
                speedConsumer);

        // if there is no offset, this is all we'll do
        if (offset == null) {
            return print
                    .andThen(driveToStart)
                    .andThen(servo)
                    .onlyWhile(tagInView(tag));
        }

        // this will drive us to the offset position. we defer the creation of
        // the command in case the servo command fails and we finish with the
        // tag out-of-view
        Command driveToOffset = driveSubsystem.defer(() -> {

            LimelightTarget target = limelight.getCurrentTarget();
            if (target == null || target.id() != tag.ID) {
                Util.log("[ll-target] !!! LOST VIEW OF TAG !!!");
                return Commands.none();
            }

            return relativePoseCommand(currentPose -> new Pose2d(
                    currentPose.getTranslation().plus(offset),
                    currentPose.getRotation()));
        });

        // drive to the starting pose and then do two-stage targeting
        return print
                .andThen(driveToStart)
                .andThen(servo)
                .andThen(driveToOffset)
                .onlyWhile(tagInView(tag));
    }

    /**
     * @return a command to drive to an absolute position on the field
     */
    private Command absolutePoseCommand(Pose2d pose) {
        return new SwerveAutoPoseCommand(
                driveSubsystem,
                poseSupplier,
                speedConsumer,
                pose);
    }

    /**
     * @return a command that will drive to a new pose which is based on
     * transforming the current pose of the robot when the command starts
     */
    private Command relativePoseCommand(Function<Pose2d,Pose2d> poseFunction) {
        return driveSubsystem.defer(() -> {
            Pose2d oldPose = poseSupplier.get();
            Pose2d newPose = poseFunction.apply(oldPose);
            return absolutePoseCommand(newPose);
        });
    }

    /**
     * @return a {@link BooleanSupplier} that returns true if the specified tag
     * is in view (it's used to help us cancel sub-commands if a tag slips out
     * of view)
     */
    private BooleanSupplier tagInView(AprilTag tag) {
        return () -> {
            if (limelight.isTagInView(tag)) {
                return true;
            }
            Util.log("[ll-target] !!! TAG %d is not in view", tag.ID);
            return false;
        };
    }
}
