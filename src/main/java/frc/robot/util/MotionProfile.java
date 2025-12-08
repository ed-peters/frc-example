package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * Encapsulates some of the jiggery-pokery involved in calculating and using
 * a {@link TrapezoidProfile}, especially when you want to keep it up-to-date
 * with potentially changing configuration
 */
public class MotionProfile {

    TrapezoidProfile profile;
    State startState;
    State finalState;

    /**
     * Reset constrains with the most recent values (this should always be
     * called prior to samples)
     */
    public void resetConstraints(double maxVelocity, double maxAcceleration) {
        this.profile = new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
    }

    /**
     * Reset the motion profile with the most recent values (this should
     * always be called prior to samples)
     */
    public void resetMotion(double startPosition, double finalPosition) {
        this.startState = new State(startPosition, 0.0);
        this.finalState = new State(finalPosition, 0.0);
    }

    /**
     * @return where in the motion should we be at the specified time?
     */
    public State sample(double time) {
        return profile.calculate(time, startState, finalState);
    }

    /**
     * @return is the motion complete at the specified time?
     */
    public boolean isFinished(double time) {
        return profile.isFinished(time);
    }
}
