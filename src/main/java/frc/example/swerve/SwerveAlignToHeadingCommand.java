package frc.example.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.example.Util;

import java.util.function.Supplier;

import static frc.example.swerve.SwerveAlignConfig.toHeadingMaxFeedback;
import static frc.example.swerve.SwerveAlignConfig.toHeadingP;
import static frc.example.swerve.SwerveAlignConfig.toHeadingD;
import static frc.example.swerve.SwerveAlignConfig.toHeadingTolerance;

/**
 * Implements automatically aligning the robot to a target heading.
 * Demonstrates using PID control to automatically position the robot.
 * There are likely to be multiple instances of this command, with
 * different targets; each will have its own name ("reef", "speaker",
 * etc.)
 */
public class SwerveAlignToHeadingCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<Rotation2d> headingSupplier;
    final PIDController pid;
    double targetDegrees;
    double currentDegrees;
    double lastCorrection;

    public SwerveAlignToHeadingCommand(String name, SwerveDriveSubsystem drive, Supplier<Rotation2d> headingSupplier) {

        this.drive = drive;
        this.headingSupplier = headingSupplier;
        this.pid = new PIDController(toHeadingP.getAsDouble(), 0.0, toHeadingD.getAsDouble());
        this.targetDegrees = Double.NaN;
        this.currentDegrees = Double.NaN;
        this.lastCorrection = Double.NaN;

        // configure the PID controller further
        pid.setTolerance(toHeadingTolerance.getAsDouble());
        pid.enableContinuousInput(-180.0, 180.0);

        addRequirements(drive);

        SmartDashboard.putData("SwerveAlignToHeading-"+name, builder -> {
            builder.addDoubleProperty("LastError", pid::getError, null);
            builder.addDoubleProperty("LastCorrection", () -> lastCorrection, null);
            builder.addDoubleProperty("TargetHeading", () -> targetDegrees, null);
            builder.addDoubleProperty("CurrentHeading", () -> currentDegrees, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {

        // figure out where we're going this time
        targetDegrees = headingSupplier.get().getDegrees();

        // reset the PID controller to get the latest configuration
        pid.setP(toHeadingP.getAsDouble());
        pid.setD(toHeadingD.getAsDouble());
        pid.setTolerance(toHeadingTolerance.getAsDouble());
        pid.reset();

        Util.log("[swerve] aligning to %.2f", targetDegrees);
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
        if (!pid.atSetpoint()) {
            Util.log("[swerve] !!! MISSED alignment to %.2f !!!", targetDegrees);
        }
        targetDegrees = Double.NaN;
        currentDegrees = Double.NaN;
        lastCorrection = Double.NaN;
    }
}
