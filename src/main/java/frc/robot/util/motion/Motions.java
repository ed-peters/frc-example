package frc.robot.util.motion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.util.swerve.SwerveState;

import java.util.function.DoubleSupplier;

/**
 * Helper class for creating various types of {@link Motion} implementations.
 */
public class Motions {

    /**
     * Creates a {@link Motion} through a one-dimensional space using a
     * {@link TrapezoidProfile} to control velocity and acceleration.
     * Starting and ending velocity are assumed to be 0.
     *
     * @param maxVelocity supplier for max velocity (required)
     * @param maxAcceleration supplier for max acceleration (required)
     * @param startPosition start position
     * @param finalPosition final position
     * @return a {@link Motion} that describes that movement
     */
    public static Motion<State> trapezoid(
            DoubleSupplier maxVelocity,
            DoubleSupplier maxAcceleration,
            double startPosition,
            double finalPosition) {
        return trapezoid(
                maxVelocity,
                maxAcceleration,
                startPosition,
                0.0,
                finalPosition);
    }

    /**
     * Creates a {@link Motion} through a one-dimensional space using a
     * {@link TrapezoidProfile} to control velocity and acceleration.
     * Ending velocity is assumed to be 0.
     *
     * @param maxVelocity supplier for max velocity (required)
     * @param maxAcceleration supplier for max acceleration (required)
     * @param startPosition start position
     * @param startVelocity start velocity
     * @param finalPosition final position
     * @return a {@link Motion} that describes that movement
     */
    public static Motion<State> trapezoid(
            DoubleSupplier maxVelocity,
            DoubleSupplier maxAcceleration,
            double startPosition,
            double startVelocity,
            double finalPosition) {

        State startState = new State(startPosition, startVelocity);
        State finalState = new State(finalPosition, 0.0);
        TrapezoidProfile profile = new TrapezoidProfile(new Constraints(
                maxVelocity.getAsDouble(),
                maxAcceleration.getAsDouble()));

        profile.calculate(0.0, startState, finalState);

        return new Motion<>() {

            @Override
            public State sample(double t) {
                if (t < 0) {
                    return startState;
                }
                if (t > totalTime()) {
                    return finalState;
                }
                return profile.calculate(t, startState, finalState);
            }

            @Override
            public double totalTime() {
                return profile.totalTime();
            }
        };
    }

    /**
     * Creates a {@link Motion} that describes moving a swerve drive along
     * a straight line between two points.
     *
     * @param rotateMaxVelocity supplies max rotational velocity in deg/sec (required)
     * @param rotateMaxAcceleration supplies max rotational velocity in deg/sec squared (required)
     * @param translateMaxVelocity supplies max translation velocity in ft/sec (required)
     * @param translateMaxAcceleration supplies max translation acceleration in ft/sec squared (required)
     * @param startPose the starting pose (required)
     * @param finalPose the final pose (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public static Motion<SwerveState> straightLineSwerve(
            DoubleSupplier rotateMaxVelocity,
            DoubleSupplier rotateMaxAcceleration,
            DoubleSupplier translateMaxVelocity,
            DoubleSupplier translateMaxAcceleration,
            Pose2d startPose,
            Pose2d finalPose) {
        return new SwerveStraightLineMotion(
                rotateMaxVelocity,
                rotateMaxAcceleration,
                translateMaxVelocity,
                translateMaxAcceleration,
                startPose,
                finalPose);
    }
}
