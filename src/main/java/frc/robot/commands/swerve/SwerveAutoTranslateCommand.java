package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.enableLogging;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.translateTolerance;

/**
 * This shows how to use a trapezoid motion profile to translate the robot
 * smoothly to a target offset from its current position. It will drive
 * in a straight line and not change the heading of the robot at all.</p>
 *
 * This is meant for moving a short distance, so the offset is supplied in
 * inches and the configured velocity is in inches per second. But internal
 * calculations are done in meters because that's what WPILib uses. So pay
 * attention to where the unit conversions are done.</p>
 */
public class SwerveAutoTranslateCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Translation2d offset;
    final Timer timer;
    final double distance;
    final Rotation2d angle;
    final double cos;
    final double sin;
    final State startState;
    final State finalState;
    final PIDController pidX;
    final PIDController pidY;
    TrapezoidProfile profile;
    Pose2d startPose;
    Pose2d finalPose;
    double nextSpeedX;
    double nextSpeedY;
    double nextX;
    double nextY;

    public SwerveAutoTranslateCommand(SwerveDriveSubsystem drive, Translation2d offset) {

        this.drive = drive;
        this.offset = offset;
        this.timer = new Timer();
        this.pidX = new PIDController(translateP.getAsDouble(), 0.0, translateD.getAsDouble());
        this.pidY = new PIDController(translateP.getAsDouble(), 0.0, translateD.getAsDouble());

        // this is how far we're going to travel in a straight line to the
        // target pose; we expect the offset to be supplied in inches but we
        // will do all our calculations in meters, so we convert here
        this.distance = Units.inchesToMeters(offset.getNorm());

        // this is the direction we're moving from the start position; the cos
        // and sin will help us convert distance and speed along that line into
        // X and Y values
        this.angle = offset.getAngle();
        this.cos = offset.getAngle().getCos();
        this.sin = offset.getAngle().getSin();

        // these are the start and end state along that line (in meters)
        this.startState = new State(0.0, 0.0);
        this.finalState = new State(distance, 0.0);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        // let's calculate our starting end ending pose for this movement
        startPose = drive.getFusedPose();
        finalPose = startPose.plus(new Transform2d(cos * distance, sin * distance, angle));

        // let's also re-read configuration to create an up-to-date motion
        // profile note that configuration is in feet per second so we
        // convert to meters
        profile = new TrapezoidProfile(new Constraints(
                Units.feetToMeters(translateMaxVelocity.getAsDouble()),
                Units.feetToMeters(translateMaxAcceleration.getAsDouble())));

        // always reset the PIDs when you're doing closed loop
        Util.resetPid(pidX, translateP, translateD, translateTolerance);
        Util.resetPid(pidY, translateP, translateD, translateTolerance);

        Util.log("[swerve-pose] headed to %s", finalPose);

        timer.restart();
    }

    @Override
    public void execute() {

        // this will give us the current distance and speed along our line
        State nextState = profile.calculate(timer.get(), startState, finalState);

        // we calculate the "next" position of the robot using the position
        // along the line to the target
        nextX = startPose.getX() + nextState.position * cos;
        nextY = startPose.getY() + nextState.position * sin;

        // we use the calculated velocity as the basis for our speed
        nextSpeedX = nextState.velocity * cos;
        nextSpeedY = nextState.velocity * sin;

        // we also use the target pose for feedback, to stay on target
        Pose2d currentPose = drive.getFusedPose();
        nextSpeedX += Util.applyClamp(
                pidX.calculate(currentPose.getX(), nextX),
                translateMaxFeedback);
        nextSpeedY += Util.applyClamp(
                pidY.calculate(currentPose.getY(), nextY),
                translateMaxFeedback);

        // let's drive!
        drive.drive("offset", new ChassisSpeeds(nextSpeedX, nextSpeedY, 0.0));

        // publish the "next" and final poses for debugging
        Util.publishPose("AlignOffsetNext", new Pose2d(nextX, nextY, startPose.getRotation()));
        Util.publishPose("AlignOffsetFinal", finalPose);

        // in normal operation, we're probably going to wind up with
        // many instances of this command. instead of trying to register
        // them all under different names, we'll just have whichever one
        // is running publish the "latest" information for debugging
        if (enableLogging) {
            SmartDashboard.putNumber("SwerveAutoTranslateCommand/Distance", distance);
            SmartDashboard.putNumber("SwerveAutoTranslateCommand/Angle", angle.getDegrees());
            SmartDashboard.putNumber("SwerveAutoTranslateCommand/SpeedX", nextSpeedX);
            SmartDashboard.putNumber("SwerveAutoTranslateCommand/SpeedY", nextSpeedY);
            SmartDashboard.putNumber("SwerveAutoTranslateCommand/ErrorX", pidX.getError());
            SmartDashboard.putNumber("SwerveAutoTranslateCommand/ErrorY", pidY.getError());
            SmartDashboard.putString("SwerveAutoTranslateCommand/Status", "running");
        }
    }

    @Override
    public boolean isFinished() {

        // with trapezoid profiles, we decide when we're done based on time,
        // rather than position. getting this right is a matter of proper
        // tuning (see below).
        return timer.hasElapsed(profile.totalTime());
    }

    @Override
    public void end(boolean interrupted) {

        double distanceToTarget = drive
                .getFusedPose()
                .getTranslation()
                .getDistance(finalPose.getTranslation());

        // this command runs until the calculated profile time has elapsed;
        // if tuning isn't correct, or an obstacle gets in our way, we may
        // not reach the target position

        // wpilib calculates distance in meters so we convert to inches
        // for comparing with the threshold
        boolean success = Units.inchesToMeters(distanceToTarget) < translateTolerance.getAsDouble();

        // always log failure; if you see this in logs a log it might
        // suggest the need to re-tune
        if (!success) {
            Util.log("[swerve-pose] !!! MISSED goal %s by %.2f !!!", finalPose, distanceToTarget);
        }

        nextSpeedX = Double.NaN;
        nextSpeedY = Double.NaN;
        nextX = Double.NaN;
        nextY = Double.NaN;
        timer.stop();

        if (enableLogging) {
            SmartDashboard.putString("SwerveAutoTranslateCommand/Running?", success
                    ? "succeeded"
                    : "failed");
        }
    }
}
