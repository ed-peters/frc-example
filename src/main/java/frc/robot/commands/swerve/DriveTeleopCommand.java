package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;

import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveConfig.*;
import static frc.robot.util.Util.chopDigits;
import static frc.robot.util.Util.isBlueAlliance;
import static frc.robot.util.Util.isRotating;

/**
 * Implements teleop driving with optional "drift correction" and
 * "field relative" modes. Read the comments further down for more
 * information about how these work.
 */
public class DriveTeleopCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<ChassisSpeeds> speedSupplier;
    double targetHeading;
    double lastDrift;
    double lastCorrection;

    public DriveTeleopCommand(SwerveDriveSubsystem drive, Supplier<ChassisSpeeds> speedSupplier) {

        this.drive = drive;
        this.speedSupplier = speedSupplier;

        addRequirements(drive);

        SmartDashboard.putData("SwerveTeleopCommand", builder -> {
            builder.addDoubleProperty("TargetHeading", () -> chopDigits(targetHeading), null);
            builder.addDoubleProperty("LastDrift", () -> chopDigits(lastDrift), null);
            builder.addDoubleProperty("LastCorrection", () -> chopDigits(lastCorrection), null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {
        System.out.println("[swerve] entering teleop");
        targetHeading = Double.NaN;
        lastCorrection = Double.NaN;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = speedSupplier.get();
        if (fieldRelative.getAsBoolean()) {
            speeds = convertFromFieldRelative(speeds);
        }
        if (teleopDriftCorrection.getAsBoolean()) {
            speeds = updateForDriftDetection(speeds);
        }
        drive.drive("teleop", speeds);
    }

    /**
     * Implements translation from "field relative" speeds. This actually
     * involves two translations - one based on the driver's POV and
     * another based on how the robot is oriented.
     */
    protected ChassisSpeeds convertFromFieldRelative(ChassisSpeeds incomingSpeeds) {

        // incoming speeds are interpreted like so:
        //   +X goes away from the driver
        //   +Y goes to the driver's left

        ChassisSpeeds fieldRelativeSpeeds;

        // if the driver is on the blue alliance, they are looking
        // at the field "normally" - so going away from them is
        // also +X on the field
        if (isBlueAlliance()) {
            fieldRelativeSpeeds = incomingSpeeds;
        }

        // if they're red, they are actually looking at the field from
        // the opposite side (so going away from them is -Y on the
        // field)
        else {
            fieldRelativeSpeeds= new ChassisSpeeds(
                    -incomingSpeeds.vxMetersPerSecond,
                    -incomingSpeeds.vyMetersPerSecond,
                    incomingSpeeds.omegaRadiansPerSecond);
        }

        // to get to fully robot-relative speeds, we need to consider
        // the current heading of the robot
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                drive.getHeading());
    }

    /**
     * Driving straight with a swerve drive is actually a little tough - they
     * will always tend to rotate a little bit. This implements a simple drift
     * correction.
     */
    protected ChassisSpeeds updateForDriftDetection(ChassisSpeeds speeds) {

        double currentHeading = drive.getHeading().getDegrees();

        double translateSpeed = Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond);

        // if we're moving slowly, or the driver is actively rotating
        // the robot, we will accept the current heading as the "true"
        // target heading
        if (translateSpeed > 0.1 || isRotating(speeds)) {
            targetHeading = currentHeading;
            lastDrift = Double.NaN;
            lastCorrection = Double.NaN;
            return speeds;
        }

        // if we're moving quickly without rotating, we may need to
        // apply some drift correction
        else {
            lastDrift = Util.angleModulus(targetHeading - currentHeading);
            lastCorrection = Util.applyClamp(
                    teleopDriftP.getAsDouble() * lastDrift,
                    teleopDriftMaxFeedback);
            return new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    Math.toRadians(lastCorrection));
        }
    }
}
