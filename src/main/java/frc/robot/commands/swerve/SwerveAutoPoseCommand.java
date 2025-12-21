package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.Util;
import frc.robot.util.motion.PoseProfile;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.commands.swerve.SwerveAutoConfig.rotateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateRampTime;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateRampTime;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateTolerance;

/**
 * This command drives the robot to a fixed pose on the field using a
 * {@link PoseProfile}. It's implemented so it doesn't depend on a specific
 * swerve drive implementation.
 */
public class SwerveAutoPoseCommand extends Command {

    final Supplier<Pose2d> poseSupplier;
    final Consumer<ChassisSpeeds> speedConsumer;
    final Pose2d finalPose;
    final PoseProfile profile;
    final PIDController pidX;
    final PIDController pidY;
    final PIDController pidOmega;
    final Timer timer;
    Pose2d startPose;
    double cos;
    double sin;

    public SwerveAutoPoseCommand(Subsystem subsystem,
                                 Supplier<Pose2d> poseSupplier,
                                 Consumer<ChassisSpeeds> speedConsumer,
                                 Pose2d finalPose) {
        this.poseSupplier = poseSupplier;
        this.speedConsumer = speedConsumer;
        this.finalPose = finalPose;
        this.profile = new PoseProfile(
                        rotateMaxVelocity,
                        rotateMaxAcceleration,
                        rotateRampTime,
                        translateMaxVelocity,
                        translateMaxAcceleration,
                        translateRampTime);
        this.pidX = new PIDController(translateP.getAsDouble(), 0, translateD.getAsDouble());
        this.pidY = new PIDController(translateP.getAsDouble(), 0, translateD.getAsDouble());
        this.pidOmega = new PIDController(rotateP.getAsDouble(), 0, rotateD.getAsDouble());
        this.timer = new Timer();

        pidOmega.enableContinuousInput(-180.0, 180.0);

        addRequirements(subsystem);
    }

    // ===============================================================
    // INITIALIZATION
    // ===============================================================

    @Override
    public void initialize() {

        // calculate starting pose and prepare for the rotation and translation
        // components of our movement
        startPose = poseSupplier.get();
        profile.reset(startPose, finalPose);

        // start timing
        timer.restart();
    }

    // ===============================================================
    // EXECUTION
    // ===============================================================

    @Override
    public void execute() {

        PoseProfile.State desiredState = profile.sample(timer.get());

        // publish speeds to the dashboard
        ChassisSpeeds desiredSpeeds = desiredState.speed();
        SmartDashboard.putNumber("SpeedX", Units.metersToFeet(desiredSpeeds.vxMetersPerSecond));
        SmartDashboard.putNumber("SpeedY", Units.metersToFeet(desiredSpeeds.vyMetersPerSecond));
        SmartDashboard.putNumber("SpeedOmega", Math.toDegrees(desiredSpeeds.omegaRadiansPerSecond));

        // publish pose to the dashboard
        Pose2d desiredPose = desiredState.pose();
        Util.publishPose("AutoPoseNext", desiredPose);
        Util.publishPose("AutoPoseFinal", finalPose);

        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", true);

        // make it so!
        speedConsumer.accept(desiredSpeeds);
    }

    // ===============================================================
    // FINISHING UP
    // ===============================================================

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(profile.totalTime());
    }

    @Override
    public void end(boolean interrupted) {

        Pose2d currentPose = poseSupplier.get();
        Rotation2d currentHeading = currentPose.getRotation();

        // since we determine completion based on the timing of the motion
        // profiles, we may not actually hit the target pose (e.g. if there
        // is an obstruction on the field, or our tuning is wrong). we will
        // log a warning about that - if this appears a lot, there may be a
        // problem with tuning

        double deltaDegrees = finalPose.getRotation()
                .minus(currentHeading)
                .getDegrees();
        if (Math.abs(deltaDegrees) > rotateTolerance.getAsDouble()) {
            Util.log("[auto-pose] !!! FAILED ROTATE; delta is %.2f", deltaDegrees);
        } else {
            Util.log("[auto-pose] done rotating");
        }

        double deltaFeet = Util.feetBetween(finalPose, currentPose);
        if (Math.abs(deltaFeet) > translateTolerance.getAsDouble()) {
            Util.log("[auto-pose] !!! FAILED TRANSLATE; delta is %.2f", deltaFeet);
        } else {
            Util.log("[auto-pose] done translating");
        }

        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", false);
    }

    // ===============================================================
    // HELPERS
    // ===============================================================

    /**
     * Creates a version of this command that will calculate its final pose
     * dynamically based on the pose of the robot when the command is first
     * scheduled
     */
    public static Command relative(Subsystem subsystem,
                                   Supplier<Pose2d> poseSupplier,
                                   Consumer<ChassisSpeeds> speedConsumer,
                                   Function<Pose2d,Pose2d> poseFunction) {
        return Commands.defer(
                () -> {
                    Pose2d finalPose = poseFunction.apply(poseSupplier.get());
                    return new SwerveAutoPoseCommand(subsystem,
                            poseSupplier,
                            speedConsumer,
                            finalPose);
                },
                Set.of(subsystem));
    }
}
