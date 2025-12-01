package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.rotateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;

/**
 * This calculates rotation to a target heading by using a trapezoid motion
 * profile to ensure smooth acceleration and braking.
 */
public class SwerveAutoRotateCalculator {

    /**
     * This represents the calculation of one "step" in the rotation
     * movement. It holds the heading and rotational speed of the chassis
     * at a particular moment in time.
     */
    public static class AutoRotation {

        public final Rotation2d speed;
        public final Rotation2d heading;

        public AutoRotation(Rotation2d speed, Rotation2d heading) {
            this.heading = heading;
            this.speed = speed;
        }
    }

    final PIDController pid;
    TrapezoidProfile profile;
    Rotation2d startHeading;
    Rotation2d finalHeading;
    State startState;
    State finalState;

    public SwerveAutoRotateCalculator() {
        this.pid = new PIDController(rotateP.getAsDouble(), 0.0, rotateD.getAsDouble());
    }

    public void initialize(Pose2d startPose, Pose2d finalPose) {

        this.startHeading = startPose.getRotation();
        this.finalHeading = finalPose.getRotation();

        // this calculates the distance in degrees; using "minus" here should
        // automatically send us the shortest way around
        double deltaDegrees = Math.abs(finalHeading.minus(startHeading).getDegrees());
        if (deltaDegrees < rotateTolerance.getAsDouble()) {
            startState = Util.NAN_STATE;
            finalState = Util.NAN_STATE;
            profile = null;
            Util.log("[swerve-pose] no rotation required");
            return;
        }

        Util.log("[swerve-pose] rotation required from %s to %s",
                startPose.getRotation(),
                finalPose.getRotation());

        // calculate current and final state; note that we don't wrap the
        // final state at 180 degrees - the motion profile calculates
        // a straight line and doesn't do "wraparound" like a PID controller
        startState = new State(startHeading.getDegrees(), 0.0);
        finalState = new State(startHeading.getDegrees() + deltaDegrees, 0.0);

        // this will calculate a motion profile around the circle with smooth
        // acceleration and deceleration (units are degrees)
        profile = new TrapezoidProfile(new Constraints(
                rotateMaxVelocity.getAsDouble(),
                rotateMaxAcceleration.getAsDouble()));
        
        // reset PID calculations
        Util.resetPid(pid, rotateP, rotateMaxFeedback, rotateTolerance);
    }

    public AutoRotation calculate(Pose2d currentPose, double time) {

        // if we're not rotating, there is nothing to do
        if (profile == null) {
            return new AutoRotation(Rotation2d.kZero, startHeading);
        }

        // the main component of our velocity is going to be provided by
        // the calculated motion profile; this is calculated in degrees
        State nextState = profile.calculate(time, startState, finalState);

        // this is the ideal speed of rotation (this is like "feedforward"
        // and will be in degrees)
        double speed = nextState.velocity;

        // we are also going to calculate "feedback" based on where we are
        // now compared to where we're supposed to be
        speed += Util.applyClamp(
                pid.calculate(currentPose.getRotation().getDegrees(), nextState.position),
                rotateMaxFeedback);

        return new AutoRotation(
                Rotation2d.fromDegrees(speed),
                Rotation2d.fromDegrees(nextState.position));

    }

    /**
     * Our motion profile determines how fast the movement will be
     */
    public double totalTime() {
        return profile == null ? 0.0 : profile.totalTime();
    }
    
}
