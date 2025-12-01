package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.rotateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;

public class SwerveAutoRotateCalculator {
    
    public record AutoRotation(Rotation2d speed, Rotation2d heading) {

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
        double deltaDegrees = finalHeading.minus(startHeading).getDegrees();
        if (deltaDegrees < rotateTolerance.getAsDouble()) {
            startState = Util.NAN_STATE;
            finalState = Util.NAN_STATE;
            profile = null;
            Util.log("[swerve-pose] no rotation required");
            return;
        }

        // calculate current and final state; note that we don't wrap the
        // final state at 180 degrees - the motion profile calculates
        // TODO these will be in the same units as above
        startState = new State(startHeading.getDegrees(), 0.0);
        finalState = new State(startHeading.getDegrees() + deltaDegrees, 0.0);

        // this will calculate a motion profile around the circle with smooth
        // acceleration and deceleration
        // TODO these will be in the same units as above
        profile = new TrapezoidProfile(new Constraints(
                Units.feetToMeters(rotateMaxVelocity.getAsDouble()),
                Units.feetToMeters(rotateMaxAcceleration.getAsDouble())));

    }

    public AutoRotation calculate(Pose2d currentPose, double time) {

        if (profile == null) {
            return new AutoRotation(Rotation2d.kZero, startHeading);
        }

        // the main component of our velocity is going to be provided by
        // the calculated motion profile
        // TODO this will be in the same units as above; note unit conversion
        State nextState = profile.calculate(time, startState, finalState);
        double speed = nextState.velocity;

        // we are also going to calculate feedback based on where we are
        // now compared to where we're supposed to be
        // TODO should this be in degrees?
        speed += Util.applyClamp(
                pid.calculate(currentPose.getRotation().getRadians(), nextState.position),
                rotateMaxFeedback);

        // TODO note use of plus and unit transform
        return new AutoRotation(
                Rotation2d.fromDegrees(speed),
                Rotation2d.fromDegrees(nextState.position));

    }

    public double totalTime() {
        return profile == null ? 0.0 : profile.totalTime();
    }
    
}
