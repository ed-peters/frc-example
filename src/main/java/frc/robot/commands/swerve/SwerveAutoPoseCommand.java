package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.Util;
import frc.robot.util.motion.PoseProfile;
import frc.robot.util.motion.PoseProfile.State;

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
    final PoseProfile profile;
    final PIDController pidX;
    final PIDController pidY;
    final PIDController pidOmega;
    final Timer timer;
    Function<Pose2d,Pose2d> poseFunction;
    Pose2d startPose;
    Pose2d finalPose;

    /**
     * Creates a {@link SwerveAutoPoseCommand} that calculates its target
     * pose based on the robot's current pose when it starts executing
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param poseFunction calculates target pose based on current pose
     */
    public SwerveAutoPoseCommand(Subsystem subsystem,
                                 Supplier<Pose2d> poseSupplier,
                                 Consumer<ChassisSpeeds> speedConsumer,
                                 Function<Pose2d,Pose2d> poseFunction) {
        this.poseSupplier = poseSupplier;
        this.speedConsumer = speedConsumer;
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
        this.poseFunction = poseFunction;
        this.startPose = Util.NAN_POSE;
        this.finalPose = Util.NAN_POSE;

        pidOmega.enableContinuousInput(-180.0, 180.0);

        addRequirements(subsystem);
    }

    /**
     * Captures the start pose, reset the motion profile and start timing
     */
    @Override
    public void initialize() {
        startPose = poseSupplier.get();
        finalPose = poseFunction.apply(startPose);
        profile.reset(startPose, finalPose);
        timer.restart();
    }

    // ===============================================================
    // EXECUTION
    // ===============================================================

    /**
     * Calculates target pose and speeds, logs stuff to the dashboard and
     * applies target speed
     */
    @Override
    public void execute() {

        State desiredState = profile.sample(timer.get());
        ChassisSpeeds desiredSpeeds = desiredState.speed();
        Pose2d desiredPose = desiredState.pose();

        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", true);
        SmartDashboard.putNumber("SpeedX", Units.metersToFeet(desiredSpeeds.vxMetersPerSecond));
        SmartDashboard.putNumber("SpeedY", Units.metersToFeet(desiredSpeeds.vyMetersPerSecond));
        SmartDashboard.putNumber("SpeedOmega", Math.toDegrees(desiredSpeeds.omegaRadiansPerSecond));
        Util.publishPose("AutoPoseNext", desiredPose);
        Util.publishPose("AutoPoseFinal", finalPose);

        speedConsumer.accept(desiredSpeeds);
    }

    /**
     * @return are we done with the calculated motion?
     */
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(profile.totalTime());
    }

    /**
     * Cleans up when we're done. Since we determine completion based on the
     * timing of the motion profile, we may not actually hit the target pose
     * (e.g. if there is an obstruction on the field, or our tuning is wrong).
     * We will log a warning about that - if this appears a lot, there may be
     * a problem with tuning.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

        Pose2d currentPose = poseSupplier.get();
        Rotation2d currentHeading = currentPose.getRotation();

        // error in rotation
        double deltaDegrees = finalPose.getRotation()
                .minus(currentHeading)
                .getDegrees();
        if (Math.abs(deltaDegrees) > rotateTolerance.getAsDouble()) {
            Util.log("[auto-pose] !!! FAILED ROTATE; delta is %.2f", deltaDegrees);
        } else {
            Util.log("[auto-pose] done rotating");
        }

        // error in translation
        double deltaFeet = Util.feetBetween(finalPose, currentPose);
        if (Math.abs(deltaFeet) > translateTolerance.getAsDouble()) {
            Util.log("[auto-pose] !!! FAILED TRANSLATE; delta is %.2f", deltaFeet);
        } else {
            Util.log("[auto-pose] done translating");
        }

        startPose = Util.NAN_POSE;
        finalPose = Util.NAN_POSE;

        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", false);
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that drives to the same
     * absolute position on the field each time it's run
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param finalPose final pose on the field
     * @return the command
     */
    public static Command absolute(Subsystem subsystem,
                                   Supplier<Pose2d> poseSupplier,
                                   Consumer<ChassisSpeeds> speedConsumer,
                                   Pose2d finalPose) {
        return new SwerveAutoPoseCommand(subsystem,
                poseSupplier,
                speedConsumer,
                currentPose -> finalPose);
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that calculates its final pose
     * by applying a {@link Transform2d} to the robot's starting pose each
     * time it's run
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param transform transformation of the start pose
     * @return the command
     */
    public static Command transform(Subsystem subsystem,
                                    Supplier<Pose2d> poseSupplier,
                                    Consumer<ChassisSpeeds> speedConsumer,
                                    Transform2d transform) {
        return new SwerveAutoPoseCommand(subsystem,
                poseSupplier,
                speedConsumer,
                currentPose -> currentPose.transformBy(transform));
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that translates the robot in a
     * specified direction
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param translation translation to apply
     * @return the command
     */
    public static Command translate(Subsystem subsystem,
                                    Supplier<Pose2d> poseSupplier,
                                    Consumer<ChassisSpeeds> speedConsumer,
                                    Translation2d translation) {
        return transform(subsystem,
                poseSupplier,
                speedConsumer,
                new Transform2d(translation, Rotation2d.kZero));
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that rotates the robot to a
     * specific heading
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param heading heading to face
     * @return the command
     */
    public static Command rotateTo(Subsystem subsystem,
                                   Supplier<Pose2d> poseSupplier,
                                   Consumer<ChassisSpeeds> speedConsumer,
                                   Rotation2d heading) {
        Function<Pose2d,Pose2d> func = currentHeading -> new Pose2d(
                currentHeading.getTranslation(),
                heading);
        return new SwerveAutoPoseCommand(subsystem,
                poseSupplier,
                speedConsumer,
                func);
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that rotates the robot by
     * a relative angle
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param rotation rotation to apply
     * @return the command
     */
    public static Command rotateBy(Subsystem subsystem,
                                   Supplier<Pose2d> poseSupplier,
                                   Consumer<ChassisSpeeds> speedConsumer,
                                   Rotation2d rotation) {
        return transform(subsystem,
                poseSupplier,
                speedConsumer,
                new Transform2d(Translation2d.kZero, rotation));
    }
}
