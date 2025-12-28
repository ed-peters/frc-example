package frc.robot.util.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util;
import frc.robot.util.swerve.SwerveState;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Implementation of {@link Motion} that describes a straight-line movement
 * between two points in 2d space. Movement can include translation and/or
 * rotation components. This can be used to achieve a target position, or
 * heading, or both.
 */
public class SwerveStraightLineMotion implements Motion<SwerveState> {

    // we're going to hardcode these because they seem unlikely to need tuning
    static final double MIN_ROTATION = Units.degreesToRadians(3.0);
    static final double MIN_TRANSLATION = Units.inchesToMeters(3.0);

    final Pose2d startPose;
    final Pose2d finalPose;
    final Motion<State> translation;
    final Motion<State> rotation;
    final double cos;
    final double sin;
    double totalTime;

    /**
     * Creates a {@link SwerveStraightLineMotion}
     * @param rotateMaxVelocity supplies max rotational velocity in deg/sec (required)
     * @param rotateMaxAcceleration supplies max rotational velocity in deg/sec squared (required)
     * @param translateMaxVelocity supplies max translation velocity in ft/sec (required)
     * @param translateMaxAcceleration supplies max translation acceleration in ft/sec squared (required)
     * @param startPose the starting pose (required)
     * @param finalPose the final pose (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveStraightLineMotion(DoubleSupplier rotateMaxVelocity,
                                    DoubleSupplier rotateMaxAcceleration,
                                    DoubleSupplier translateMaxVelocity,
                                    DoubleSupplier translateMaxAcceleration,
                                    Pose2d startPose,
                                    Pose2d finalPose) {

        Objects.requireNonNull(rotateMaxVelocity);
        Objects.requireNonNull(rotateMaxAcceleration);
        Objects.requireNonNull(translateMaxVelocity);
        Objects.requireNonNull(translateMaxAcceleration);
        Objects.requireNonNull(startPose);
        Objects.requireNonNull(finalPose);

        this.startPose = startPose;
        this.finalPose = finalPose;
        this.totalTime = 0.0;

        // if we're rotating, we calculate an "offset" around the circle based
        // on its delta from the starting position - starting at 0 degrees and
        // ending at the final heading. the sign of this tx may be positive or
        // negative depending on whether we're going left or right. we will use
        // our SCurveProfile to move smoothly around. if the angular distance
        // is tiny, we won't bother trying to do the rotation.
        double radians = finalPose.getRotation()
                .minus(startPose.getRotation())
                .getRadians();
        if (Math.abs(radians) > MIN_ROTATION) {
            rotation = Motions.trapezoid(
                    rotateMaxVelocity,
                    rotateMaxAcceleration,
                    0.0, radians);
            totalTime = Math.max(totalTime, rotation.totalTime());
        } else {
            if (radians != 0.0) {
                Util.log("[swerve-motion] ignoring rotation of %.2f degrees",
                        Math.toDegrees(radians));
            }
            rotation = null;
        }

        // if we're translating, we will calculate the distance and angle
        // between the start and end pose, and we will move along a straight
        // line using a regular trapezoid profile.
        double meters = Util.metersBetween(startPose, finalPose);
        if (meters > MIN_TRANSLATION) {
            translation = Motions.trapezoid(
                    translateMaxVelocity,
                    translateMaxAcceleration,
                    0.0, meters);
            totalTime = Math.max(totalTime, translation.totalTime());
        } else {
            if (meters != 0.0) {
                Util.log("[swerve-motion] ignoring translation of %.2f inches",
                        Units.metersToInches(meters));
            }
            translation = null;
        }

        // if we're translating, we need to know the angle of the line between
        // the start and final poses; the cos and sin of this angle will let us
        // decompose straight-line movement into separate X and Y components
        if (translation != null) {
            Rotation2d angle = finalPose.getTranslation()
                    .minus(startPose.getTranslation())
                    .getAngle();
            cos = angle.getCos();
            sin = angle.getSin();
        } else {
            cos = 0.0;
            sin = 0.0;
        }
    }

    /**
     * @return how long in seconds this motion will run for (this is the
     * maximum of the translation time and the rotation time)
     */
    public double totalTime() {
        return totalTime;
    }

    /**
     * @param t the time in seconds
     * @return desired pose and speed at the specified moment in time
     */
    public SwerveState sample(double t) {

        // if we're before the beginning or after the end, we will use either
        // the start or final pose and assume 0 speed
        if (t < 0) {
            return new SwerveState(startPose, Util.ZERO_SPEED);
        } else if (t > totalTime) {
            return new SwerveState(finalPose, Util.ZERO_SPEED);
        }

        double speedX = 0.0;
        double speedY = 0.0;
        double speedOmega = 0.0;
        double poseX = startPose.getX();
        double poseY = startPose.getY();
        double poseOmega = startPose.getRotation().getRadians();

        // if we're rotating, we get the state of the rotation profile. the
        // position will be the offset from the start heading at this moment
        // in time.
        if (rotation != null) {
            State state = rotation.sample(t);
            poseOmega += state.position;
            speedOmega = state.velocity;
        }

        // if we're translating, we get the state of the translation profile.
        // the position and velocity will along the line from start to finish;
        // we decompose them along X/Y using cos/sin.
        if (translation != null) {
            State state = translation.sample(t);
            speedX = state.velocity * cos;
            speedY = state.velocity * sin;
            poseX += state.position * cos;
            poseY += state.position * sin;
        }

        return new SwerveState(
                new Pose2d(poseX, poseY, Rotation2d.fromRadians(poseOmega)),
                new ChassisSpeeds(speedX, speedY, speedOmega));
    }
}
