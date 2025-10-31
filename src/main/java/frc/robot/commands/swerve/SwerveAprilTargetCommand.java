package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.util.AprilTarget;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.Set;
import java.util.function.Supplier;

/**
 * Comprehensive target command for AprilTags. Includes three different
 * operations:
 * <ul>
 *
 *     <li>Align our heading to face the AprilTag that's currently
 *     in view; and,</li>
 *
 *     <li>Align the robot in the X and Y directions to be directly
 *     "in front" of the tag; and,</li>
 *
 *     <li>Optionally, drive to an offset of the "right in front of
 *     the tag" position</li>
 *
 * </ul>
 */
public class SwerveAprilTargetCommand extends DeferredCommand {

    public SwerveAprilTargetCommand(SwerveDriveSubsystem drive,
                                    Supplier<AprilTarget> targetSupplier,
                                    Supplier<Translation2d> translationSupplier) {
        super(() -> createCommands(drive, targetSupplier, translationSupplier), Set.of(drive));
    }

    public static Command createCommands(SwerveDriveSubsystem drive,
                                               Supplier<AprilTarget> targetSupplier,
                                               Supplier<Translation2d> offsetSupplier) {

        // get the currently in-view target; if there isn't one,
        // we can't do anything
        AprilTarget target = targetSupplier.get();
        if (target == null || target.id() < 1) {
            Util.log("[april-target] NO TAG IN VIEW!!!");
            return Commands.none();
        }

        // we are going to rotate to face the tag; this means our heading
        // will be 180 degrees off from the tag
        Command rotate = new SwerveAlignToHeadingCommand(
                drive,
                target.pose().getRotation().plus(Rotation2d.k180deg));

        // next we are going to align ourselves to be directly in front
        // of the target
        Command align = new SwerveAlignToTagCommand(
                drive,
                targetSupplier);

        Translation2d offset = offsetSupplier != null
                ? offsetSupplier.get()
                : null;

        // that might be it
        if (offset == null) {
            Util.log("[april-target] preparing two-stage targeting");
            return rotate.andThen(align);
        }

        // if not, we're going to translate as well
        Command translate = new SwerveAlignTrajectoryCommand(drive, offset);
        Util.log("[april-target] preparing three-stage targeting");
        return rotate
                .andThen(align)
                .andThen(translate);
    }
}
