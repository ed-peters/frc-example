package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.PDController;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.commands.swerve.SwerveAlignConfig.toHeadingMaxFeedback;
import static frc.robot.commands.swerve.SwerveAlignConfig.toHeadingP;
import static frc.robot.commands.swerve.SwerveAlignConfig.toHeadingD;
import static frc.robot.commands.swerve.SwerveAlignConfig.toHeadingTolerance;

/**
 * Implements automatically aligning the robot to a target heading.
 * Demonstrates using PID control to automatically position the robot.
 */
public class SwerveAlignToHeadingCommand extends Command {

    final SwerveDriveSubsystem drive;
    final PDController pid;
    final double targetDegrees;
    double currentDegrees;
    double lastCorrection;

    public SwerveAlignToHeadingCommand(SwerveDriveSubsystem drive, Rotation2d targetHeading) {

        this.drive = drive;
        this.pid = new PDController(toHeadingP,
                    toHeadingD,
                    toHeadingMaxFeedback,
                    toHeadingTolerance);
        this.targetDegrees = targetHeading.getDegrees();
        this.currentDegrees = Double.NaN;
        this.lastCorrection = Double.NaN;

        // configure the PID controller further
        pid.setTolerance(toHeadingTolerance.getAsDouble());
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
            builder.addDoubleProperty("CurrentHeading", () -> currentDegrees, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        // calculate the current heading and feedback
        currentDegrees = drive.getHeading().getDegrees();
        lastCorrection = Util.applyClamp(
                pid.calculate(currentDegrees, targetDegrees),
                toHeadingMaxFeedback);

        drive.drive("heading", new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(lastCorrection)));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

        // warn about this, in case this command had to be interrupted
        // by a timeout because it never got to the target heading or
        // something like that; that's a sign that it's badly tuned or
        // may need a larger tolerance
        if (!pid.atSetpoint()) {
            Util.log("[align-heading] !!! MISSED alignment to %.2f !!!", targetDegrees);
        }

        currentDegrees = Double.NaN;
        lastCorrection = Double.NaN;
    }
}
