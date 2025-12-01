package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.MotionProfile;
import frc.robot.util.Util;

import java.util.function.Consumer;
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
 * This command drives the robot to a fixed pose on the field. It handles
 * two different aspects of movement:
 * <ul>
 *
 *     <li>Rotation - if the new pose has a different heading than the
 *     starting pose when the command is run, we use a {@link MotionProfile}
 *     to smoothly rotate to the target heading.</li>
 *
 *     <li>Translation - if the new pose has a different position on the field
 *     than the starting pose when the command is run, we use a second
 *     {@link MotionProfile} to smoothly drive along a straight line to the
 *     target position.</li>
 *
 * </ul>
 *
 * These are independent actions and are calculated separately, then combined
 * during execution. Neither is required - you can rotate without translating
 * (e.g. to align to a target heading), or translate without rotating (e.g. to
 * "scoot" to a fixed offset).</p>
 *
 * This command is implemented so it doesn't depend on a specific swerve
 * drive implementation.</p>
 */
public class SwerveAutoPoseCommand extends Command {

    final Supplier<Pose2d> poseSupplier;
    final Consumer<ChassisSpeeds> speedConsumer;
    final Pose2d finalPose;
    final MotionProfile rotationProfile;
    final MotionProfile translationProfile;
    final PIDController pidX;
    final PIDController pidY;
    final PIDController pidOmega;
    final Timer timer;
    boolean skipTranslation;
    boolean skipRotation;
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
        this.rotationProfile = new MotionProfile();
        this.translationProfile = new MotionProfile();
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
        initializeRotation(startPose);
        initializeTranslation(startPose);

        // start timing
        timer.restart();
    }

    private void initializeRotation(Pose2d startPose) {

        // if we're not rotating, there's nothing to do
        skipRotation = startPose.getRotation().equals(finalPose.getRotation());
        if (skipRotation) {
            return;
        }

        // we calculate a "position" around the circle based on its offset
        // from the starting position - starting at 0 degrees and ending
        // at the final heading. the sign of this offset may be positive or
        // negative depending on whether we're going left or right.
        double degrees = finalPose.getRotation()
                        .minus(startPose.getRotation())
                        .getDegrees();
        rotationProfile.resetMotion(0.0, degrees);

        // we use a maximum acceleration and velocity to ensure a smooth
        // movement throughout the turn
        rotationProfile.resetConstraints(
                rotateMaxVelocity.getAsDouble(),
                rotateMaxAcceleration.getAsDouble());

        Util.log("[auto-pose] rotating from %s to %s",
                startPose.getRotation(),
                finalPose.getRotation());

        // reset PID
        Util.resetPid(pidOmega, rotateP, rotateD, rotateTolerance);

    }

    private void initializeTranslation(Pose2d startPose) {

        // if we're not translating, there's nothing to do
        skipTranslation = startPose.getTranslation()
                .equals(finalPose.getTranslation());
        if (skipTranslation) {
            return;
        }

        // this is how far (in feet) we are moving from start to final
        // (this will always be a positive number)
        double distance = Util.feetBetween(startPose, finalPose);

        // this is the angle of line between the start and final poses; cos
        // and sin will help us decompose straight-line movement along that
        // line into separate X and Y components
        Rotation2d angle = finalPose.getTranslation()
                .minus(startPose.getTranslation())
                .getAngle();
        cos = angle.getCos();
        sin = angle.getSin();

        // we will plan motion along the straight line between start and final
        translationProfile.resetMotion(
                0.0,
                distance);

        // we use a maximum acceleration and velocity to ensure a smooth
        // movement throughout the turn
        translationProfile.resetConstraints(
                translateMaxVelocity.getAsDouble(),
                translateMaxAcceleration.getAsDouble());

        Util.log("[auto-pose] translating from %s to %s",
                startPose.getTranslation(),
                finalPose.getTranslation());

        // reset PIDs
        Util.resetPid(pidX, translateP, translateD, translateTolerance);
        Util.resetPid(pidY, translateP, translateD, translateTolerance);

    }

    // ===============================================================
    // EXECUTION
    // ===============================================================

    @Override
    public void execute() {

        Pose2d currentPose = poseSupplier.get();

        // calculate the desired rotation and translation
        DesiredRotation desiredRotation = calculateRotation(
                currentPose,
                timer.get());
        DesiredTranslation desiredTranslation = calculateTranslation(
                currentPose,
                timer.get());

        // all these calculations assume field-relative movement, so when we
        // calculate speed we have to convert it
        ChassisSpeeds speeds = new ChassisSpeeds(
                desiredTranslation.desiredSpeed.getX(),
                desiredTranslation.desiredSpeed.getY(),
                desiredRotation.desiredSpeed.getRadians());
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                currentPose.getRotation());

        // publish some stuff to the dashboard
        SmartDashboard.putBoolean("SwerveAutoPoseCommand/IsTranslating?", !skipTranslation);
        SmartDashboard.putBoolean("SwerveAutoPoseCommand/IsRotating?", !skipRotation);
        SmartDashboard.putNumber("SpeedX", Units.metersToFeet(speeds.vxMetersPerSecond));
        SmartDashboard.putNumber("SpeedY", Units.metersToFeet(speeds.vyMetersPerSecond));
        SmartDashboard.putNumber("SpeedOmega", Math.toDegrees(speeds.omegaRadiansPerSecond));
        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", true);

        // let's also  calculate and display where we "should" be; this is
        // incredibly helpful for debugging
        Pose2d desiredPose = new Pose2d(
                desiredTranslation.desiredPosition,
                desiredRotation.desiredHeading);
        Util.publishPose("AutoPoseNext", desiredPose);
        Util.publishPose("AutoPoseFinal", finalPose);

        // make it so!
        speedConsumer.accept(speeds);
    }

    private DesiredRotation calculateRotation(Pose2d currentPose, double time) {

        // if we're not rotating, there's nothing to do
        if (skipRotation) {
            return new DesiredRotation(
                    Rotation2d.kZero,
                    startPose.getRotation());
        }

        // this tells us both where we are supposed to be facing right now
        // (in degrees), as well as how fast we should be turning (in degrees
        // per second) and in what direction
        State desiredState = rotationProfile.sample(time);

        // the velocity is the base component of our turning speed - it's like
        // a "feedforward" component
        double desiredSpeed = desiredState.velocity;

        // this is the actual position where we should be at this time
        Rotation2d desiredHeading = startPose.getRotation()
                .plus(Rotation2d.fromDegrees(desiredState.position));

        // we compare actual vs desired position to add feedback to speed and
        // correct for discrepancies
        desiredSpeed += Util.applyClamp(
                pidOmega.calculate(currentPose.getRotation().getDegrees(), desiredHeading.getDegrees()),
                rotateMaxFeedback);

        return new DesiredRotation(
                Rotation2d.fromDegrees(desiredSpeed),
                desiredHeading);
    }

    private DesiredTranslation calculateTranslation(Pose2d currentPose, double time) {

        // if we're not translating, there's nothing to do
        if (skipTranslation) {
            return new DesiredTranslation(
                    Translation2d.kZero,
                    currentPose.getTranslation());
        }

        // this is tells us how far we should be along the straight line
        // between (in feet) between the start and final pose, and how fast we
        // should be going along that line (in feet per second)
        State desiredState = translationProfile.sample(time);

        // this decomposes the speed (in feet per second) into X and Y
        // components using the cos and sin we already calculated; as with
        // rotation this is "feedforward"
        double speedX = desiredState.velocity * cos;
        double speedY = desiredState.velocity * sin;

        // this does the same for position (but calculates it in meters, since
        // that's what Pose2d uses, and then updates desired speed based on
        // feedback
        double positionX = startPose.getX() + Units.feetToMeters(desiredState.position * cos);
        double positionY = startPose.getY() + Units.feetToMeters(desiredState.position * sin);

        // here's where we calculate feedback in the X and Y directions based
        // on how far off we are from the desired X/Y
         speedX += Util.applyClamp(
                 pidX.calculate(currentPose.getX(), positionX),
                 translateMaxFeedback);
         speedY += Util.applyClamp(
                 pidY.calculate(currentPose.getY(), positionY),
                 translateMaxFeedback);

        return new DesiredTranslation(
                new Translation2d(Units.feetToMeters(speedX), Units.feetToMeters(speedY)),
                new Translation2d(positionX, positionY));
    }

    // ===============================================================
    // FINISHING UP
    // ===============================================================

    @Override
    public boolean isFinished() {

        double time = timer.get();

        // rotation and translation are separate motions; one may complete
        // before the other, but we aren't done until both are complete
        boolean rotationDone = skipRotation || rotationProfile.isFinished(time);
        boolean translationDone = skipTranslation || translationProfile.isFinished(time);

        return rotationDone && translationDone;
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

    record DesiredRotation(Rotation2d desiredSpeed, Rotation2d desiredHeading) {

    }

    record DesiredTranslation(Translation2d desiredSpeed, Translation2d desiredPosition) {

    }
}
