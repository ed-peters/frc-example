package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.enableLogging;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;

/**
 * This shows how to automatically rotate the swerve drive to a specific
 * heading. This can be useful for e.g. facing an arena wall or an AprilTag.
 */
public class SwerveAutoRotateCommand extends Command {

    final SwerveDriveSubsystem drive;
    final PIDController pid;
    final Timer timer;
    final Rotation2d targetHeading;
    TrapezoidProfile profile;
    State startState;
    State finalState;

    public SwerveAutoRotateCommand(SwerveDriveSubsystem drive, Rotation2d targetHeading) {

        this.drive = drive;
        this.pid = new PIDController(rotateP.getAsDouble(), 0.0, rotateD.getAsDouble());
        this.timer = new Timer();
        this.targetHeading = targetHeading;
        this.startState = Util.NAN_STATE;
        this.finalState = Util.NAN_STATE;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        Rotation2d currentHeading = drive.getHeading();

        // calculate current and final state
        startState = new State(currentHeading.getDegrees(), drive.getYawRate());
        finalState = new State(calculateFinalDegrees(currentHeading), 0.0);

        if (enableLogging) {
            Util.log("[swerve-rotate] rotating from %.2f to %.2f",
                    startState.position,
                    finalState.position);
        }

        // always reset the PID when you're doing closed loop
        Util.resetPid(pid, rotateP, rotateD, rotateTolerance);

        // calculate a new profile based on up-to-date config
        profile = new TrapezoidProfile(new Constraints(
                rotateMaxVelocity.getAsDouble(),
                rotateMaxAcceleration.getAsDouble()));

        // start the timer going
        timer.restart();

    }

    /*
     * Since angles wrap around, there are always two ways to get from
     * the current heading to the target (left or right). We want to take
     * the shortest direction.
     */
    private double calculateFinalDegrees(Rotation2d currentHeading) {

        // calculate the shortest distance between the target heading
        // and our current heading
        double deltaDegrees = targetHeading.minus(currentHeading).getDegrees();

        // we add that back to the current heading to get the "real"
        // final heading
        return currentHeading.getDegrees() + deltaDegrees;
    }

    @Override
    public void execute() {

        Rotation2d currentHeading = drive.getHeading();

        // the main component of our velocity is going to be provided by
        // the calculated motion profile
        State nextState = profile.calculate(timer.get(), startState, finalState);

        // we are also going to calculate feedback based on where we are
        // now compared to where we're supposed to be
        double feedback = Util.applyClamp(
                    pid.calculate(currentHeading.getDegrees(), nextState.position),
                    rotateMaxFeedback);

        // we are going to drive at the speed calculated by the profile,
        // PLUS a feedback correction based on the expected current position
        drive.drive("heading", new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(nextState.velocity) + feedback));

        // in normal operation, we're probably going to wind up with
        // many instances of this command. instead of trying to register
        // them all under different names, we'll just have whichever one
        // is running publish the "latest" information for debugging
        if (enableLogging) {
            SmartDashboard.putNumber("SwerveAutoRotateCommand/LastError", pid.getError());
            SmartDashboard.putNumber("SwerveAutoRotateCommand/LastFeedback", feedback);
            SmartDashboard.putNumber("SwerveAutoRotateCommand/FinalHeading", finalState.position);
            SmartDashboard.putNumber("SwerveAutoRotateCommand/CurrentHeading", currentHeading.getDegrees());
            SmartDashboard.putString("SwerveAutoRotateCommand/Status", "running");
        }
    }

    @Override
    public boolean isFinished() {

        // we're done when we've completed the motion profile
        return timer.hasElapsed(profile.totalTime());
    }

    @Override
    public void end(boolean interrupted) {

        // this command runs until the calculated profile time has elapsed;
        // if tuning isn't correct, or an obstacle gets in our way, we may
        // not reach the target heading
        boolean success = pid.atSetpoint();

        // always log failure; if you see this in logs a log it might
        // suggest the need to re-tune
        if (!success) {
            Util.log("[swerve-rotate] !!! MISSED goal %.2f b y%.2f !!!",
                    finalState.position,
                    pid.getError());
        }

        startState = Util.NAN_STATE;
        finalState = Util.NAN_STATE;
        timer.stop();

        if (enableLogging) {
            SmartDashboard.putString("SwerveAutoRotateCommand/Status", success
                    ? "succeeded"
                    : "failed");
        }
    }
}
