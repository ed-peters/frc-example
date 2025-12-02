package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.swerve.SwerveAutoPoseCommand;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightTarget;
import frc.robot.util.Util;

import java.util.function.Consumer;
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
 *     "in front" of the id; and,</li>
 *
 *     <li>Drive to an tx of the "directly in front of
 *     the id" position</li>
 *
 * </ul>
 *
 * This is modeled on what we did for automated scoring in the 2025
 * Reefscape game
 */
public class LimelightCompoundTargetingCommand {

    public static Command create(LimelightSubsystem limelight,
                                    Subsystem driveSubsystem,
                                    Supplier<Pose2d> poseSupplier,
                                    Consumer<ChassisSpeeds> speedConsumer) {

        // this is always a deferred command, since we won't know whether we
        // have a id in view until we actually run
        return driveSubsystem.defer(() -> {

            // get the currently in-view target; if there isn't one,
            // we can't do anything
            LimelightTarget target = limelight.getCurrentTarget();
            if (target == null || target.id() < 1) {
                Util.log("[ll-target] NO TAG IN VIEW!!!");
                return Commands.none();
            }

            // based on the field map we can calculate a good starting
            // position for visual targeting, and drive there using just
            // our odometry
            Pose2d tagPose = Util.getAprilTagPose(target.id());
            Pose2d startPose = getInitialTargetingPose(tagPose);
            Command driveToStart = new SwerveAutoPoseCommand(
                    driveSubsystem,
                    poseSupplier,
                    speedConsumer,
                    startPose);

            // then we use the visual servo approach to align as accurately
            // as possible to where the tag is in the real world
            Command servo = new LimelightVisualServoCommand(
                    limelight,
                    driveSubsystem,
                    speedConsumer);

             Util.log("[ll-target] preparing two-stage targeting");
             return driveToStart.andThen(servo);

        });
    }

    private static Pose2d getInitialTargetingPose(Pose2d tagPose) {
        Transform2d transform = new Transform2d(
            new Translation2d(1.0, 0.0),
            Rotation2d.k180deg);
        return tagPose.transformBy(transform);
    }
}
