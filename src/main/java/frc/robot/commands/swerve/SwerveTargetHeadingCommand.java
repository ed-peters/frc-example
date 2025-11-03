package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.PDController;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.commands.swerve.SwerveTargetingConfig.toHeadingMaxFeedback;
import static frc.robot.commands.swerve.SwerveTargetingConfig.toHeadingP;
import static frc.robot.commands.swerve.SwerveTargetingConfig.toHeadingD;
import static frc.robot.commands.swerve.SwerveTargetingConfig.toHeadingTolerance;

/**
 * This shows how to automatically align the swerve drive to a specific
 * heading. This can be useful for facing an arena wall or an AprilTag.
 */
public class SwerveTargetHeadingCommand extends Command {

    final SwerveDriveSubsystem drive;
    final PDController pid;
    final double targetDegrees;
    double currentHeading;
    double lastCorrection;

    public SwerveTargetHeadingCommand(SwerveDriveSubsystem drive, Rotation2d targetHeading) {

        this.drive = drive;
        this.pid = new PDController(toHeadingP,
                    toHeadingD,
                    toHeadingMaxFeedback,
                    toHeadingTolerance);
        this.targetDegrees = targetHeading.getDegrees();
        this.currentHeading = Double.NaN;
        this.lastCorrection = Double.NaN;

        // this tells the PID controller that there is a "wraparound"
        // at (-180, 180) - it will do the math for us to make sure
        // it doesn't try to correct by going "the long way around"
        pid.enableContinuousInput(-180.0, 180.0);

        addRequirements(drive);
    }

    /**
     * In normal operation, there are probably going to be a bunch of
     * instances of this command, so we won't clutter the dashboard with
     * them all; instead, this will let you register them under different
     * names e.g. for testing
     */
    public void addToDash(String name) {
        SmartDashboard.putData(name, builder -> {
            builder.addDoubleProperty("LastError", pid::getError, null);
            builder.addDoubleProperty("LastCorrection", () -> lastCorrection, null);
            builder.addDoubleProperty("TargetHeading", () -> targetDegrees, null);
            builder.addDoubleProperty("CurrentHeading", () -> currentHeading, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
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
    }
}
