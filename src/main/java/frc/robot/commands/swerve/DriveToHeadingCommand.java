package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Dash;
import frc.robot.util.PDController;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveConfig.*;

/**
 * Uses PD control to align the swerve drive to a heading supplied by
 * a {@link DoubleSupplier}
 */
public class DriveToHeadingCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<Rotation2d> headingSupplier;
    final PDController pid;
    double targetDegrees;
    double currentDegrees;
    double lastCorrection;

    public DriveToHeadingCommand(String name, SwerveDriveSubsystem drive, Supplier<Rotation2d> headingSupplier) {

        this.drive = drive;
        this.headingSupplier = headingSupplier;
        this.pid = new PDController(alignHeadingP, alignHeadingD, alignHeadingMax, alignHeadingTolerance);
        this.targetDegrees = Double.NaN;
        this.currentDegrees = Double.NaN;
        this.lastCorrection = Double.NaN;

        // configure the PID controller to apply wraparound
        pid.enableContinuousInput(-180.0, 180.0);

        addRequirements(drive);

        SmartDashboard.putData("SwerveAlignHeading-"+name, builder -> {
            builder.addDoubleProperty("LastError", () -> Util.chopDigits(pid.getError()), null);
            builder.addDoubleProperty("LastCorrection", () -> Util.chopDigits(lastCorrection), null);
            builder.addDoubleProperty("TargetHeading", () -> Util.chopDigits(targetDegrees), null);
            builder.addDoubleProperty("CurrentHeading", () -> Util.chopDigits(currentDegrees), null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {
        targetDegrees = headingSupplier.get().getDegrees();
        pid.reset();
        Dash.log("[swerve] aligning to %.2f", targetDegrees);
    }

    @Override
    public void execute() {
        currentDegrees = drive.getHeading().getDegrees();
        lastCorrection = pid.calculate(currentDegrees, targetDegrees);
        drive.drive("heading", new ChassisSpeeds(0.0, 0.0, Math.toRadians(lastCorrection)));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (!pid.atSetpoint()) {
            Dash.log("[swerve] did NOT manage to align to %.2f", targetDegrees);
        }
        targetDegrees = Double.NaN;
        currentDegrees = Double.NaN;
        lastCorrection = Double.NaN;
    }
}
