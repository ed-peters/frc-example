package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.PDController;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.commands.swerve.SwerveTargetingConfig.enableLogging;
import static frc.robot.commands.swerve.SwerveTargetingConfig.rotateMaxFeedback;
import static frc.robot.commands.swerve.SwerveTargetingConfig.rotateP;
import static frc.robot.commands.swerve.SwerveTargetingConfig.rotateD;
import static frc.robot.commands.swerve.SwerveTargetingConfig.rotateTolerance;

/**
 * This shows how to automatically rotate the swerve drive to a specific
 * heading. This can be useful for e.g. facing an arena wall or an AprilTag.
 */
public class SwerveRotateCommand extends Command {

    final SwerveDriveSubsystem drive;
    final PDController pid;
    final double targetDegrees;
    double currentHeading;
    double lastCorrection;

    public SwerveRotateCommand(SwerveDriveSubsystem drive, Rotation2d targetHeading) {

        this.drive = drive;
        this.pid = new PDController(rotateP,
                rotateD,
                rotateMaxFeedback,
                rotateTolerance);
        this.targetDegrees = targetHeading.getDegrees();
        this.currentHeading = Double.NaN;
        this.lastCorrection = Double.NaN;

        // this tells the PID controller that there is a "wraparound"
        // at (-180, 180) - it will do the math for us to make sure
        // it doesn't try to correct by going "the long way around"
        pid.enableContinuousInput(-180.0, 180.0);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        // always reset the PID when you're doing closed loop
        pid.reset();
    }

    @Override
    public void execute() {

        // calculate the current heading and correction speed, and
        // apply it to rotate the drive
        currentHeading = drive.getHeading().getDegrees();
        lastCorrection = pid.calculate(currentHeading, targetDegrees);
        drive.drive("heading", new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(lastCorrection)));

        // in normal operation, we're probably going to wind up with
        // many instances of this command. instead of trying to register
        // them all under different names, we'll just have whichever one
        // is running publish the "latest" information for debugging
        if (enableLogging) {
            SmartDashboard.putNumber("SwerveTargetHeadingCommand/LastError", pid.getError());
            SmartDashboard.putNumber("SwerveTargetHeadingCommand/LastCorrection", lastCorrection);
            SmartDashboard.putNumber("SwerveTargetHeadingCommand/TargetHeading", targetDegrees);
            SmartDashboard.putNumber("SwerveTargetHeadingCommand/CurrentHeading", currentHeading);
            SmartDashboard.putBoolean("SwerveTargetHeadingCommand/Running?", true);
        }
    }

    @Override
    public boolean isFinished() {

        // we're done when we're "close enough" to the target heading
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

        // this command could run forever if we can't attain the target
        // heading for some reason. rather than lose control of the
        // robot during a match we will probably put it in a timeout.
        // if it gets interrupted while it's running, we should know that
        // so we can check tuning increase the tolerance or whatever
        if (!pid.atSetpoint()) {
            Util.log("[align-heading] !!! MISSED alignment to %.2f !!!", targetDegrees);
        }

        currentHeading = Double.NaN;
        lastCorrection = Double.NaN;

        if (enableLogging) {
            SmartDashboard.putBoolean("SwerveTargetHeadingCommand/Running?", false);
        }
    }
}
