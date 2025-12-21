package frc.robot.util.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Calculates a straight-line profile between two separate {@link Pose2d}
 * instances. This can be used to achieve a target position, or heading, or
 * both.
 */
public class PoseProfile {

    // we're going to hardcode these because it seems unlikely to change
    static final double MIN_ROTATION = Units.degreesToRadians(3.0);
    static final double MIN_TRANSLATION = 3.0 / 12.0;

    /**
     * Captures the desired position and speed at a moment in time.
     *
     * @param pose the desired pose at that moment
     * @param speed the desired speed at that moment
     */
    public record State(Pose2d pose, ChassisSpeeds speed) {

    }

    final SCurveProfile translateProfile;
    final SCurveProfile rotateProfile;
    Pose2d startPose;
    Pose2d finalPose;
    boolean isRotating;
    boolean isTranslating;
    double cos;
    double sin;
    double totalTime;

    /**
     * Creates a {@link PoseProfile}
     * @param rotateMaxVelocity maximum rotational velocity (required)
     * @param rotateMaxAcceleration maximum rotational velocity (required)
     * @param rotateRampTime maximum rotational velocity (required)
     * @param translateMaxVelocity maximum rotational velocity (required)
     * @param translateMaxAcceleration maximum rotational velocity (required)
     * @param translateRampTime maximum rotational velocity (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public PoseProfile(DoubleSupplier rotateMaxVelocity,
                       DoubleSupplier rotateMaxAcceleration,
                       DoubleSupplier rotateRampTime,
                       DoubleSupplier translateMaxVelocity,
                       DoubleSupplier translateMaxAcceleration,
                       DoubleSupplier translateRampTime) {
        translateProfile = new SCurveProfile(rotateMaxVelocity,
                rotateMaxAcceleration,
                rotateRampTime);
        rotateProfile = new SCurveProfile(translateMaxVelocity,
                translateMaxAcceleration,
                translateRampTime);
    }

    /**
     * Resets the motion described by this profile.
     *
     * @param startPose start pose (required)
     * @param finalPose final pose (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public void reset(Pose2d startPose, Pose2d finalPose) {

        Objects.requireNonNull(startPose);
        Objects.requireNonNull(finalPose);
        this.startPose = startPose;
        this.finalPose = finalPose;

        totalTime = 0.0;

        // if we're rotating, we calculate an "offset" around the circle based
        // on its delta from the starting position - starting at 0 degrees and
        // ending at the final heading. the sign of this tx may be positive or
        // negative depending on whether we're going left or right. we will use
        // our SCurveProfile to move smoothly around. if the angular distance
        // is tiny, we won't bother trying to do the rotation.
        double radians = finalPose.getRotation()
                .minus(startPose.getRotation())
                .getRadians();
        isRotating = radians > MIN_ROTATION;
        if (isRotating) {
            rotateProfile.reset(0.0, 0.0, radians);
            totalTime = Math.max(totalTime, rotateProfile.totalTime());
        }

        // if we're translating, we will calculate the distance and angle
        // between the start and end pose, and we will move along a straight
        // line using an SCurve profile.
        //
        // if the distance is less than a few inches, we won't bother trying
        // to do the translation.
        double meters = Units.feetToMeters(Util.feetBetween(
                startPose,
                finalPose));
        isTranslating = meters > MIN_TRANSLATION;
        if (isTranslating) {

            // this is the angle of line between the start and final poses; cos
            // and sin will help us decompose straight-line movement along that
            // line into separate X and Y components
            Rotation2d angle = finalPose.getTranslation()
                    .minus(startPose.getTranslation())
                    .getAngle();
            cos = angle.getCos();
            sin = angle.getSin();

            translateProfile.reset(0.0, 0.0, meters);
            totalTime = Math.max(totalTime, translateProfile.totalTime());
        }
    }

    /**
     * @param t the time in seconds
     * @return desired pose and speed at the specified moment in time
     */
    public State sample(double t) {

        double speedX = 0.0;
        double speedY = 0.0;
        double speedOmega = 0.0;
        double poseX = startPose.getX();
        double poseY = startPose.getY();
        double poseOmega = startPose.getRotation().getRadians();

        // if we're rotating, we get the state of the rotation profile. the
        // position will be the offset from the start heading at this moment
        // in time.
        if (isRotating) {
            SCurveProfile.State state = rotateProfile.sample(t);
            poseOmega += state.position();
            speedOmega = state.velocity();
        }

        // if we're translation, we get the state of the translation profile.
        // the position and velocity will along the line from start to finish;
        // we decompose them X/Y using cos/sin.
        if (isTranslating) {
            SCurveProfile.State state = translateProfile.sample(t);
            speedX = state.velocity() * cos;
            speedY = state.velocity() * sin;
            poseX += state.position() * cos;
            poseY += state.position() * sin;
        }

        return new State(
                new Pose2d(poseX, poseY, Rotation2d.fromRadians(poseOmega)),
                new ChassisSpeeds(speedX, speedY, speedOmega));
    }

    /**
     * @return how long in seconds this profile should run for (this is the
     * maximum of the translation time and the rotation time)
     */
    public double totalTime() {
        return totalTime;
    }
}
