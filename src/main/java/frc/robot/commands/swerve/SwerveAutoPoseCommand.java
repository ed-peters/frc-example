package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.Util;
import frc.robot.util.motion.Motion;
import frc.robot.util.motion.Motions;
import frc.robot.util.motion.PDController;
import frc.robot.util.swerve.SwerveState;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.commands.swerve.SwerveAutoConfig.rotateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateTolerance;

/**
 * This command drives the robot to a fixed pose on the field using a
 * {@link frc.robot.util.motion.SwerveStraightLineMotion}. It's implemented
 * so it doesn't depend on a specific swerve drive implementation.
 */
public class SwerveAutoPoseCommand extends Command {

    final Supplier<Pose2d> poseSupplier;
    final Consumer<ChassisSpeeds> speedConsumer;
    final PDController pidX;
    final PDController pidY;
    final PDController pidOmega;
    final Timer timer;
    Motion<SwerveState> motion;
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
        this.pidX = new PDController(translateP, translateD, translateTolerance, translateMaxFeedback);
        this.pidY = new PDController(translateP, translateD, translateTolerance, translateMaxFeedback);
        this.pidOmega = new PDController(rotateP, rotateD, rotateTolerance, rotateMaxFeedback);
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
        motion = Motions.straightLineSwerve(
                rotateMaxVelocity,
                rotateMaxAcceleration,
                translateMaxVelocity,
                translateMaxAcceleration,
                startPose,
                finalPose);

        pidX.reset();
        pidY.reset();
        pidOmega.reset();

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

        SwerveState desiredState = motion.sample(timer.get());
        Pose2d desiredPose = desiredState.pose();

        // since the PoseProfile gives us field-relative speeds, we need to
        // translate them into robot-relative speeds so we can tell the
        // robot to drive the intended path
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredState.speeds(),
                poseSupplier.get().getRotation());

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
        return motion == null || timer.hasElapsed(motion.totalTime());
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

        // what if we failed to meet our rotation target?
        if (!pidOmega.atSetpoint()) {
            Util.log("[auto-pose] !!! FAILED ROTATE; delta is %.2f",
                    Units.radiansToDegrees(pidOmega.getError()));
        }

        // what if we failed to meet either our X or Y targets?
        if (!pidX.atSetpoint() || !pidY.atSetpoint()) {
            Util.log("[auto-pose] !!! FAILED TRANSLATE; delta is %.2f",
                    Util.feetBetween(finalPose, currentPose));
        }

        startPose = Util.NAN_POSE;
        finalPose = Util.NAN_POSE;
        motion = null;

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
     * Creates a {@link SwerveAutoPoseCommand} that drives to a relative
     * offset from the current position every time it's run
     *
     * @param subsystem the subsystem that controls the drive
     * @param poseSupplier supplier for the current robot pose
     * @param speedConsumer setter for drive speed
     * @param relativePose target pose, relative to the robot's pose when
     *                     the command starts (for instance, an X value of +1
     *                     would slide the robot 1m to the left when the
     *                     command runs)
     * @return the command
     */
    public static Command relative(Subsystem subsystem,
                                   Supplier<Pose2d> poseSupplier,
                                   Consumer<ChassisSpeeds> speedConsumer,
                                   Pose2d relativePose) {

        return new SwerveAutoPoseCommand(subsystem,
                poseSupplier,
                speedConsumer,
                currentPose -> Util.addRelativePose(currentPose, relativePose));
    }
}
