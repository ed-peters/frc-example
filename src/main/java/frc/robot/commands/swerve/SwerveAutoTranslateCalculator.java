package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

/**
 * Calculates the X/Y portion of moving the chassis to a specific point.
 * We start by projecting a straight line between the start and final
 * positions. Then we calculate a trajectory along that line with a
 * trapezoid motion profile, for smooth acceleration and braking.
 * </p>
 *
 * Pay particular attention to units here - our preferences are stored in
 * feet, so we do our calculation in feet and translate to meters only when
 * necessary.
 * </p>
 */
public class SwerveAutoTranslateCalculator {

    /**
     * This represents the calculation of one "step" in the translation
     * movement. It holds the position (in meters) and speed (in meters
     * per second) of the chassis at a particular moment in time.
     */
    public static class AutoTranslation {

        final double positionX;
        final double positionY;
        final double speedX;
        final double speedY;

        public AutoTranslation(double positionX,
                               double positionY,
                               double speedX,
                               double speedY) {
            this.positionX = positionX;
            this.positionY = positionY;
            this.speedX = speedX;
            this.speedY = speedY;
        }
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
        // too small, we won't bother with any of this; we'll calculate
        // distance as feet here and continue in those units
        distance = Util.feetBetween(startPose, finalPose);
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
        // the straight line between the start and end pose; we're still in
        // feet here
        startState = new State(0.0, 0.0);
        finalState = new State(distance, 0.0);

        // this will calculate a motion profile along that straight line,
        // with smooth acceleration and deceleration; also still in feet
        profile = new TrapezoidProfile(new Constraints(
                translateMaxVelocity.getAsDouble(),
                translateMaxAcceleration.getAsDouble()));

        // once we know the angle of movement relative to the robot's
        // current heading, the cos and sin of the angle will help us
        // calculate the X and Y components of the movement (these are
        // just ratios; they have no associated dimension)
        Rotation2d angle = finalPose.minus(startPose).getTranslation().getAngle();
        cos = angle.getCos();
        sin = angle.getSin();

    }

    public AutoTranslation calculate(Pose2d currentPose, double time) {

        // if we're not translating, there is nothing to do
        if (profile == null) {
            return new AutoTranslation(
                    startPose.getX(),
                    startPose.getY(),
                    0.0, 0.0);
        }

        // otherwise, this will give us the "ideal" distance and speed (along
        // the line between the start and final points) at this moment in time;
        // both are in feet
        State nextState = profile.calculate(
                time,
                startState,
                finalState);

        // this calculates where that is on the field; Pose2d expects
        // distance in meters so we convert here
        Pose2d nextPose = startPose.transformBy(new Transform2d(
                Units.feetToMeters(nextState.position * cos),
                Units.feetToMeters(nextState.position * sin),
                Util.ZERO_ROTATION));

        // our starting estimate for velocity in X/Y directions is the ideal
        // velocity we just calculated (this is basically the "feedforward"
        // calculation for this movement); this will be in feet for now
        double speedX = nextState.velocity * cos;
        double speedY = nextState.velocity * sin;
        System.err.println(speedX);

        // this calculates how far away we are from the "ideal" position
        // we just identified; since we're using Pose2d calculations, this
        // is in meters
        Transform2d poseError = nextPose.minus(currentPose);

        // this converts our pose error to feet and calculates positional
        // "feedback" in the X/Y directions
        speedX += Util.applyClamp(
                pidX.calculate(0.0, Units.metersToFeet(poseError.getX())),
                translateMaxFeedback);
        speedY += Util.applyClamp(
                pidY.calculate(0.0, Units.metersToFeet(poseError.getY())),
                translateMaxFeedback);

        // we return both position and speed in meters per second
        return new AutoTranslation(
                nextPose.getX(),
                nextPose.getY(),
                Units.feetToMeters(speedX),
                Units.feetToMeters(speedY));
    }

    /**
     * Our motion profile determines how fast the movement will be
     */
    public double totalTime() {
        return profile == null ? 0.0 : profile.totalTime();
    }

}
