package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.Supplier;

import static frc.robot.commands.swerve.SwerveTeleopConfig.fieldRelative;
import static frc.robot.commands.swerve.SwerveTeleopConfig.driftCorrection;
import static frc.robot.commands.swerve.SwerveTeleopConfig.driftMaxFeedback;
import static frc.robot.commands.swerve.SwerveTeleopConfig.driftP;

/**
 * This implements teleop driving using the {@link SwerveTeleopSpeedSupplier}.
 * Demonstrates the following features:
 * <ul>
 *
 *     <li>Drift correction - keeps track of the "intended" robot
 *     heading and, if it seems to drift while the robot is trying
 *     to drive straight, apply some rotation to correct it</li>
 *
 *      <li>Correction of "driver relative" speeds, so that pushing
 *      the joystick away from the driver always sends the robot
 *      away from the driver</li>
 *
 * </ul>
 */
public class SwerveTeleopCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<ChassisSpeeds> speedSupplier;
    double targetHeading;
    double lastDrift;
    double lastCorrection;

    public SwerveTeleopCommand(SwerveDriveSubsystem drive, Supplier<ChassisSpeeds> speedSupplier) {
        this.drive = drive;
        this.speedSupplier = speedSupplier;
        addRequirements(drive);
        SmartDashboard.putData("SwerveTeleopCommand", builder -> {
            builder.addDoubleProperty("TargetHeading", () -> targetHeading, null);
            builder.addDoubleProperty("LastDrift", () -> lastDrift, null);
            builder.addDoubleProperty("LastCorrection", () ->  lastCorrection, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {
        targetHeading = Double.NaN;
        lastDrift = Double.NaN;
        lastCorrection = Double.NaN;
        Util.log("[swerve] entering teleop");
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = speedSupplier.get();
        if (fieldRelative.getAsBoolean()) {
            speeds = convertFromFieldRelative(speeds);
        }
        if (driftCorrection.getAsBoolean()) {
            speeds = updateForDriftDetection(speeds);
        }
        drive.drive("teleop", speeds);
    }

    /**
     * What we usually call "field relative" is actually "driver relative".
     * Converting speeds from driver relative to robot relative involves
     * two translations - one based on the driver's POV and another based on
     * how the robot is oriented.
     */
    protected ChassisSpeeds convertFromFieldRelative(ChassisSpeeds incomingSpeeds) {

        // incoming speeds are interpreted like so:
        //   +X goes away from the driver
        //   +Y goes to the driver's left
        ChassisSpeeds fieldRelativeSpeeds;

        // if the driver is on the blue alliance, they are looking
        // at the field "normally" - so going away from them is
        // also +X on the field
        if (Util.isBlueAlliance()) {
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
     * Driving straight with a swerve drive can be a little tough - they
     * will always tend to rotate a little bit because of the way the gears
     * work. This implements a simple drift correction.</p>
     *
     * See <a href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">this post on Chief Delphi</a>
     * for a detailed explanation of why this happens.
     */
    protected ChassisSpeeds updateForDriftDetection(ChassisSpeeds speeds) {

        double currentHeading = drive.getHeading().getDegrees();

        double translateSpeed = Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond);

        // if we're moving slowly, or the driver is actively rotating
        // the robot, or we don't already have a target heading,
        // we will accept the current heading as the "true"
        // target heading
        if (translateSpeed > 0.1 || Util.isRotating(speeds) || Double.isNaN(targetHeading)) {
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
                    driftP.getAsDouble() * lastDrift,
                    driftMaxFeedback);
            return new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    Math.toRadians(lastCorrection));
        }
    }

    public void end(boolean interrupted) {
        targetHeading = Double.NaN;
        lastDrift = Double.NaN;
        lastCorrection = Double.NaN;
    }
}
