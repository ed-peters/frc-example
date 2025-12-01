package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.translateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateTolerance;

public class SwerveAutoTranslateCalculator {
    
    public record AutoTranslation(Translation2d speeds, Translation2d pose) {

    }

    final PIDController pidX;
    final PIDController pidY;
    double distance;
    double cos;
    double sin;
    TrapezoidProfile profile;
    Pose2d startPose;
    Pose2d finalPose;
    State startState;
    State finalState;

    public SwerveAutoTranslateCalculator() {
        this.pidX = new PIDController(translateP.getAsDouble(), 0.0, translateD.getAsDouble());
        this.pidY = new PIDController(translateP.getAsDouble(), 0.0, translateD.getAsDouble());
    }

    public void initialize(Pose2d startPose, Pose2d finalPose) {

        this.startPose = startPose;
        this.finalPose = finalPose;

        // if the total distance between the start and the end pose is
        // too small, we won't bother with any of this
        // TODO - what units should this be in?
        distance = Util.distanceBetween(startPose, finalPose);
        if (distance < translateTolerance.getAsDouble()) {
            startState = Util.NAN_STATE;
            finalState = Util.NAN_STATE;
            profile = null;
            cos = Double.NaN;
            sin = Double.NaN;
            Util.log("[swerve-pose] no translation required");
            return;
        }

        // these will represent the start and end state of a movement along
        // the straight line between the start and end pose
        // TODO these will be in the same units as above
        startState = new State(0.0, 0.0);
        finalState = new State(distance, 0.0);

        // this will calculate a motion profile along that straight line,
        // with smooth acceleration and deceleration
        // TODO these will be in the same units as above
        profile = new TrapezoidProfile(new Constraints(
                Units.feetToMeters(translateMaxVelocity.getAsDouble()),
                Units.feetToMeters(translateMaxAcceleration.getAsDouble())));

        // once we know the angle of movement relative to the robot's
        // current heading, the cos and sin of the angle will help us
        // calculate the X and Y components of the movement
        Rotation2d angle = finalPose.minus(startPose).getTranslation().getAngle();
        cos = angle.getCos();
        sin = angle.getSin();

    }

    public AutoTranslation calculate(Pose2d currentPose, double time) {

        // if we're not translating, there is nothing to do
        if (profile == null) {
            return new AutoTranslation(Translation2d.kZero, startPose.getTranslation());
        }

        // this will give us the current distance and speed along our line
        // TODO these will be in the same units as above
        State nextState = profile.calculate(
                time,
                startState,
                finalState);

        // this calculates the corresponding field position (we will ignore
        // rotation for this calculation)
        // TODO this will be in meters
        Pose2d nextPose = startPose.transformBy(new Transform2d(
                nextState.position * cos,
                nextState.position * sin,
                Util.ZERO_ROTATION));

        // this is how fast we should be moving along the straight line from
        // start position to final position, based only on our motion profile
        // (this is basically our "feedforward")
        // TODO probably need a unit conversion here
        double speedX = nextState.velocity * cos;
        double speedY = nextState.velocity * sin;

        // this calculates where we should be on the field, and an error
        // from the current position; that will allow us to add in an
        // adjustment to speed (this is our "feedback")
        // TODO should this be in meters?
        Transform2d poseError = nextPose.minus(currentPose);
        speedX += Util.applyClamp(
                pidX.calculate(0.0, poseError.getX()),
                translateMaxFeedback);
        speedY += Util.applyClamp(
                pidY.calculate(0.0, poseError.getY()),
                translateMaxFeedback);

        return new AutoTranslation(
                new Translation2d(speedX, speedY),
                nextPose.getTranslation());
    }

    public double totalTime() {
        return profile == null ? 0.0 : profile.totalTime();
    }

}
