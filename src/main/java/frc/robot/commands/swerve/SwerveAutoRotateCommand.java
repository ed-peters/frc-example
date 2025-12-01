package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.MotionProfile;
import frc.robot.util.Util;

import static frc.robot.commands.swerve.SwerveAutoConfig.rotateD;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxAcceleration;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxFeedback;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateMaxVelocity;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateP;
import static frc.robot.commands.swerve.SwerveAutoConfig.rotateTolerance;

public class SwerveAutoRotateCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Rotation2d finalHeading;
    final MotionProfile profile;
    final PIDController pid;
    Rotation2d startHeading;

    public SwerveAutoRotateCommand(SwerveDriveSubsystem drive, Rotation2d finalHeading) {
        this.drive = drive;
        this.finalHeading = finalHeading;
        this.profile = new MotionProfile();
        this.pid = new PIDController(rotateP.getAsDouble(), 0, rotateD.getAsDouble());
        pid.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize() {

        // this is where we start our rotation
        startHeading = drive.getHeading();

        // we calculate a "position" around the circle based on its offset
        // from the starting position - starting at 0 degrees and ending
        // at the final heading
        profile.resetMotion(
                0.0,
                finalHeading.minus(startHeading).getDegrees());

        // we use a maximum acceleration and velocity to ensure a smooth
        // movement throughout the turn
        profile.resetConstraints(
                rotateMaxVelocity.getAsDouble(),
                rotateMaxAcceleration.getAsDouble());

        Util.log("[swerve-rotate] headed from %s to %s",
                startHeading,
                finalHeading);

        // reset PID and restart timer
        Util.resetPid(pid, rotateP, rotateD, rotateTolerance);
        profile.start();
    }

    @Override
    public void execute() {

        // this is where we are supposed to be at this moment, along our
        // circle of rotation - it includes both a position and a velocity
        State desiredState = profile.sample();

        // the velocity is the base component of our drive speed - it's like
        // the "feedforward" component
        double speed = desiredState.velocity;

        // this is the actual position where we should be at this time
        Rotation2d desiredHeading = startHeading
                .plus(Rotation2d.fromDegrees(desiredState.position));
        
        // this adds feedback to speed to correct for discrepancies
        speed += Util.applyClamp(
                pid.calculate(drive.getHeading().getDegrees(), desiredHeading.getDegrees()),
                rotateMaxFeedback);
        
        SmartDashboard.putNumber("SwerveAutoRotateCommand/Error", pid.getError());
        SmartDashboard.putNumber("SwerveAutoRotateCommand/Start", startHeading.getDegrees());
        SmartDashboard.putNumber("SwerveAutoRotateCommand/Next", desiredHeading.getDegrees());
        SmartDashboard.putNumber("SwerveAutoRotateCommand/Final", finalHeading.getDegrees());
        SmartDashboard.putNumber("SwerveAutoRotateCommand/Speed", speed);
        SmartDashboard.putBoolean("SwerveAutoRotateCommand/Running?", true);

        drive.drive("rotate", new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(speed)));
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        double delta = drive.getHeading().getDegrees() - finalHeading.getDegrees();
        if (Math.abs(delta) > rotateTolerance.getAsDouble()) {
            Util.log("[swerve-rotate] !!! FAILED TO REACH GOAL; delta is %.2f", delta);
        } else {
            Util.log("[swerve-rotate] done rotating");
        }
        SmartDashboard.putBoolean("SwerveAutoRotateCommand/Running?", false);
    }
}
