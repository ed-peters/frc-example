package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

public class MotionProfile {

    final Timer timer;
    TrapezoidProfile profile;
    State startState;
    State finalState;

    public MotionProfile() {
        this.timer = new Timer();
    }

    public void resetConstraints(double maxVelocity, double maxAcceleration) {
        this.profile = new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
    }

    public void resetMotion(double startPosition, double finalPosition) {
        this.startState = new State(startPosition, 0.0);
        this.finalState = new State(finalPosition, 0.0);
    }

    public void start() {
        timer.restart();
    }

    public State sample() {
        return profile.calculate(timer.get(), startState, finalState);
    }

    public boolean isFinished() {
        return timer.hasElapsed(profile.totalTime());
    }
}
