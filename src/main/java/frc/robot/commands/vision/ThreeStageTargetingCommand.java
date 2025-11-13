package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.swerve.SwerveRotateCommand;
import frc.robot.commands.swerve.SwerveTranslateCommand;
import frc.robot.subsystems.vision.LimelightEstimator;
import frc.robot.subsystems.vision.LimelightTarget;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.Set;
import java.util.function.Supplier;

/**
 * Comprehensive "three-stage" targeting command for AprilTags based on
 * the Limelight. Includes three different operations:
 * <ul>
 *
 *     <li>Align our heading to face the AprilTag that's currently
 *     in view; and,</li>
 *
 *     <li>Align the robot in the X and Y directions to be directly
 *     "in front" of the tag; and,</li>
 *
 *     <li>Drive to an offset of the "directly in front of
 *     the tag" position</li>
 *
 * </ul>
 *
 * This is modeled on what we did for automated scoring in the 2025
 * Reefscape game
 */
public class ThreeStageTargetingCommand extends DeferredCommand {

    public ThreeStageTargetingCommand(SwerveDriveSubsystem drive,
                                      LimelightEstimator limelight,
                                      Supplier<Translation2d> translationSupplier) {

        // this is always a deferred command, since we won't know whether we
        // have a tag in view until we actually run
        super(() -> {

            // get the currently in-view target; if there isn't one,
            // we can't do anything
            LimelightTarget target = limelight.getCurrentTarget();
            if (target == null || target.tag() < 1) {
                Util.log("[ll-target] NO TAG IN VIEW!!!");
                return Commands.none();
            }

            // we are going to rotate to face the tag; this means our heading
            // will be 180 degrees off from the tag
            Command rotate = new SwerveRotateCommand(
                    drive,
                    target.pose().getRotation().plus(Rotation2d.k180deg));

            // next we are going to align ourselves to be directly in front
            // of the target
            Command align = new LimelightAprilTagCommand(
                    drive,
                    limelight);

            // translate to the target position
            Command translate = new SwerveTranslateCommand(drive, translationSupplier.get());
            Util.log("[ll-target] preparing three-stage targeting");
            return rotate
                    .andThen(align)
                    .andThen(translate);

        }, Set.of(drive));
    }
}
