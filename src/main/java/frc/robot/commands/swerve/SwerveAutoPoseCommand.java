package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.SwerveAutoRotateCalculator.AutoRotation;
import frc.robot.commands.swerve.SwerveAutoTranslateCalculator.AutoTranslation;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateTolerance;

/**
 * This command will drive the robot to a fixed pose on the field. It is
 * capable of handling both translation (moving to a specific X/Y point)
 * and rotating (orienting to a specific heading). It can be configured
 * with only translation, only rotation or both.
 * </p>
 *
 * Calculating the trajectory for translation and rotation is done
 * independently by the {@link SwerveAutoTranslateCalculator} and
 * {@link SwerveAutoRotateCalculator}, respectively.
 * </p>
 */
public class SwerveAutoPoseCommand extends Command {

    final SwerveDriveSubsystem drive;
    final SwerveAutoTranslateCalculator translate;
    final SwerveAutoRotateCalculator rotate;
    final Pose2d finalPose;
    final Timer timer;
    Pose2d startPose;

    public SwerveAutoPoseCommand(SwerveDriveSubsystem drive,
                                 Pose2d finalPose) {
        this.drive = drive;
        this.translate = new SwerveAutoTranslateCalculator();
        this.rotate = new SwerveAutoRotateCalculator();
        this.finalPose = finalPose;
        this.timer = new Timer();
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        // capture the starting pose in meters
        startPose = drive.getPose();

        // log what we're about to do
        Util.log("[swerve-pose] heading for %s", finalPose);

        // calculate trajectories
        translate.initialize(startPose, finalPose);
        rotate.initialize(startPose, finalPose);

        // start timing
        timer.restart();

    }

    @Override
    public void execute() {

        Pose2d currentPose = drive.getPose();
        double time = timer.get();

        // these tell us what the speed and position of the chassis should be
        // at this point in the movement (translation will be in meters)
        AutoTranslation tx = translate.calculate(currentPose, time);
        AutoRotation rx = rotate.calculate(currentPose, time);

        // we combine the speed portion into ChassisSpeeds for driving (this
        // is like a "feedforward" term - if we could follow this perfectly
        // we would be exactly on track)
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
            tx.speedX,
            tx.speedY,
            0.0 // rx.speed.getRadians()
        ), currentPose.getRotation());

        // we combine the position portions into a pose for visualization (the
        // individual calculators handle feedback)
        Pose2d nextPose = new Pose2d(
            tx.positionX,
            tx.positionY,
            rx.heading);

        // log stuff to dashboard (for that we will use feet/degrees)
        SmartDashboard.putNumber("SwerveAutoTranslateCommand/SpeedX", Units.metersToFeet(speeds.vxMetersPerSecond));
        SmartDashboard.putNumber("SwerveAutoTranslateCommand/SpeedY", Units.metersToFeet(speeds.vyMetersPerSecond));
        SmartDashboard.putNumber("SwerveAutoTranslateCommand/SpeedOmega", Math.toDegrees(speeds.omegaRadiansPerSecond));
        SmartDashboard.putBoolean("SwerveAutoTranslateCommand/Running?", true);

        // publish the "next" and final poses for debugging
        Util.publishPose("AutoPoseNext", nextPose);
        Util.publishPose("AutoPoseFinal", finalPose);

        drive.drive("swerve-pose", speeds);
    }

    @Override
    public boolean isFinished() {

        // we're done when the total time of the longest trajectory has
        // elapsed (we may not have made it to our goal; see end() for
        // an error check
        double totalTime = Math.max(translate.totalTime(), rotate.totalTime());
        return timer.hasElapsed(totalTime);
    }

    @Override
    public void end(boolean interrupted) {

        Pose2d currentPose = drive.getPose();

        // if we ran out of time before reaching our goal, we might have hit
        // an obstacle or something, or it might be a sign that our tuning is
        // off and needs to be adjusted; we'll log failure here

        // translation tolerance is specified in feet
        double distance = Util.feetBetween(currentPose, finalPose);
        if (distance > translateTolerance.getAsDouble()) {
            Util.log("[swerve-pose] !!! FAILED TO TRANSLATE - distance is %.2f", distance);
        }

        // rotational tolerance is specified in degrees
        double degreeDelta = Math.abs(currentPose
                .getRotation()
                .minus(finalPose.getRotation())
                .getDegrees());
        if (degreeDelta > rotateTolerance.getAsDouble()) {
            Util.log("[swerve-pose] !!! FAILED TO ROTATE - delta is %.2f", degreeDelta);
        }

        SmartDashboard.putBoolean("SwerveAutoTranslateCommand/Running?", false);
    }

    /**
     * @return a command that, when run, will calculate a target position
     * by transforming the drive's current position, and then drive there
     */
    public static Command fromTransform(SwerveDriveSubsystem drive,
                                        Transform2d transform) {
        return drive.defer(() -> new SwerveAutoPoseCommand(
                drive,
                drive.getPose().transformBy(transform)));
    }
}
