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
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static frc.robot.commands.vision.LimelightTargetingConfig.tagStartingOffset;

/**
 * This acts as a factory for various commands that implement different ways
 * of using the Limelight for driving the robot to a specific target position
 * on the field.</p>
 *
 * It's implemented so it doesn't depend on a specific swerve drive
 * implementation.</p>
 */
public class TargetingCommandBuilder {

    final LimelightSubsystem limelight;
    final Subsystem driveSubsystem;
    final Supplier<Pose2d> poseSupplier;
    final Consumer<ChassisSpeeds> speedConsumer;

    public TargetingCommandBuilder(LimelightSubsystem limelight,
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
        Command rotate = orientOnTagCommand(tag, false);
        Command servo = visualServoCommand();

        // if there is no offset, this is all we'll do
        if (offset == null) {
            return print
                    .andThen(rotate)
                    .andThen(servo)
                    .onlyWhile(tagInView(tag));
        }

        // otherwise, we will finish by translating to the offset position
        Command driveToOffset = SwerveAutoPoseCommand.translate(
                driveSubsystem,
                poseSupplier,
                speedConsumer,
                offset);
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
                                          Translation2d offset) {

        Command print = Commands.print("[ll-target] swanky targeting on tag "+tag.ID);
        Command driveToStart = orientOnTagCommand(tag, true);
        Command servo = visualServoCommand();

        // if there is no offset, this is all we'll do
        if (offset == null) {
            return print
                    .andThen(driveToStart)
                    .andThen(servo)
                    .onlyWhile(tagInView(tag));
        }

        // otherwise, we will finish by driving to the offset position
        Command driveToOffset = SwerveAutoPoseCommand.translate(
                driveSubsystem,
                poseSupplier,
                speedConsumer,
                offset);
        return print
                .andThen(driveToStart)
                .andThen(servo)
                .andThen(driveToOffset)
                .onlyWhile(tagInView(tag));
    }

    /**
     * @return a command to use the visual servo approach to align as
     * accurately as possible to where the current tag is in the real world
     */
    private Command visualServoCommand() {
        return new LimelightVisualServoCommand(
                limelight,
                driveSubsystem,
                speedConsumer);
    }

    /**
     * @return a command that orients the robot in front of the specified tag;
     * this might just be rotating, or it might include translating
     */
    private Command orientOnTagCommand(AprilTag tag, boolean driveToTag) {

        // if we want to drive to a point in front of the tag, we add in a
        // translation in the X direction when we transform the tag pose;
        // otherwise we just leave the robot where it is
        Translation2d translation = driveToTag
                ? new Translation2d(Units.feetToMeters(tagStartingOffset.getAsDouble()), 0.0)
                : Translation2d.kZero;

        // this calculates our target pose from the tag's pose; we always
        // rotate the tag's pose by 180 degrees because the tag is pointing
        // "out" from where it's mounted, and we want the robot facing it.
        Pose2d targetPose = tag.pose.toPose2d().transformBy(new Transform2d(
                translation,
                Rotation2d.k180deg));

        return SwerveAutoPoseCommand.absolute(
                driveSubsystem,
                poseSupplier,
                speedConsumer,
                targetPose);
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
