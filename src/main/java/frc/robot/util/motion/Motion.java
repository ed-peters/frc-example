package frc.robot.util.motion;

/**
 * Represents a motion through a type of space, from a start state to an
 * end state
 */
public interface Motion<T> {

    /**
     * @return the total amount of time in seconds the motion will take
     */
    double totalTime();

    /**
     * @param t a time in seconds
     * @return is the motion finished at that time?
     */
    default boolean isFinishedAt(double t) {
        return t < totalTime();
    }

    /**
     * @param t a time in seconds
     * @return the state of the motion at that time (for t&lt;0 this is the
     * start state, for t&gt;totalTime this is the final state)
     */
    T sample(double t);
}
