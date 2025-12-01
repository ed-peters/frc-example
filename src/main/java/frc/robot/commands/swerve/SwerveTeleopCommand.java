package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
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
 *
 * This command is implemented so it doesn't depend on a specific swerve
 * drive implementation.</p>
 */
public class SwerveTeleopCommand extends Command {

    final Supplier<Pose2d> poseSupplier;
    final Consumer<ChassisSpeeds> speedConsumer;
    final Supplier<ChassisSpeeds> speedInput;
    Rotation2d targetHeading;
    double lastDrift;
    double lastCorrection;

    public SwerveTeleopCommand(Supplier<ChassisSpeeds> speedInput,
                               Subsystem subsystem,
                               Supplier<Pose2d> poseSupplier,
                               Consumer<ChassisSpeeds> speedConsumer) {
        this.poseSupplier = poseSupplier;
        this.speedConsumer = speedConsumer;
        this.speedInput = speedInput;
        this.targetHeading = Util.NAN_ROTATION;
        this.lastDrift = Double.NaN;
        this.lastCorrection = Double.NaN;

        addRequirements(subsystem);

        SmartDashboard.putData("SwerveTeleopCommand", builder -> {
            builder.addDoubleProperty("TargetHeading", () -> targetHeading.getDegrees(), null);
            builder.addDoubleProperty("LastDrift", () -> lastDrift, null);
            builder.addDoubleProperty("LastCorrection", () ->  lastCorrection, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {
        targetHeading = Util.NAN_ROTATION;
        lastDrift = Double.NaN;
        lastCorrection = Double.NaN;
        Util.log("[swerve] entering teleop");
    }

    @Override
    public void execute() {

        ChassisSpeeds input = speedInput.get();
        Rotation2d currentHeading = poseSupplier.get().getRotation();

        if (fieldRelative.getAsBoolean()) {
            input = convertFromFieldRelative(currentHeading, input);
        }
        if (driftCorrection.getAsBoolean()) {
            input = updateForDriftDetection(currentHeading, input);
        }
        speedConsumer.accept(input);
    }

    /**
     * What we usually call "field relative" is actually "driver relative".
     * Converting speeds from driver relative to robot relative involves
     * two translations - one based on the driver's POV and another based on
     * how the robot is oriented.
     */
    protected ChassisSpeeds convertFromFieldRelative(Rotation2d currentHeading,
                                                     ChassisSpeeds incomingSpeeds) {

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
        // the opposite side (so going away from them is -X on the
        // field, and to their left is -Y)
        else {
            fieldRelativeSpeeds = new ChassisSpeeds(
                    -incomingSpeeds.vxMetersPerSecond,
                    -incomingSpeeds.vyMetersPerSecond,
                    incomingSpeeds.omegaRadiansPerSecond);
        }

        // to get to fully robot-relative speeds, we need to consider
        // the current heading of the robot
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                currentHeading);
    }

    /**
     * Driving straight with a swerve drive can be a little tough - they
     * will always tend to rotate a little bit because of the way the gears
     * work. This implements a simple drift correction.</p>
     *
     * See <a href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">this post on Chief Delphi</a>
     * for a detailed explanation of why this happens.
     */
    protected ChassisSpeeds updateForDriftDetection(Rotation2d currentHeading,
                                                    ChassisSpeeds speeds) {

        double translateSpeed = Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond);

        // we assume we are headed in the "correct" direction if any of the
        // following things is true:
        //   - we are moving really slowly (<10cm/sec)
        //   - the driver is actively rotating the robot
        //   - we don't already have a target heading
        boolean headingIsCorrect = translateSpeed < 0.1
                || Util.isRotating(speeds)
                || Double.isNaN(targetHeading.getDegrees());

        // if our heading is correct, we will "remember" the current heading as
        // the target, and not apply any corrections
        if (headingIsCorrect) {
            targetHeading = currentHeading;
            lastDrift = Double.NaN;
            lastCorrection = Double.NaN;
            return speeds;
        }

        // otherwise, we are moving quickly without rotating. we may need to
        // apply some drift correction.

        // calculate how far off we are from the target heading, and a
        // proportional correction (note that by using "minus" we automatically
        // calculate the shortest direction back to the target (left or right)
        lastDrift = targetHeading.minus(currentHeading).getDegrees();
        lastCorrection = Util.applyClamp(
                driftP.getAsDouble() * lastDrift,
                driftMaxFeedback);

        // replace the rotational component of the supplied speeds (which we
        // know from above is 0) with our calculated correction
        return new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                Math.toRadians(lastCorrection));
    }

    @Override
    public void end(boolean interrupted) {
        targetHeading = Util.NAN_ROTATION;
        lastDrift = Double.NaN;
        lastCorrection = Double.NaN;
    }

    /**
     * @return a teleop command for the supplied drive using the "standard"
     * controls (left stick controls strafing, right stick controls
     * turning, left trigger is sniper, right trigger is turbo)
     */
    public static SwerveTeleopCommand create(CommandXboxController controller,
                                             Subsystem subsystem,
                                             Supplier<Pose2d> poseSupplier,
                                             Consumer<ChassisSpeeds> speedConsumer) {

        // pushing right or forward on the joystick results in negative values, so
        // we invert them before using them
        DoubleSupplier leftX = () -> -controller.getLeftX();
        DoubleSupplier leftY = () -> -controller.getLeftY();
        DoubleSupplier rightX = () -> -controller.getRightX();

        // triggers controller sniper/turbo behavior
        BooleanSupplier sniperTrigger = () -> controller.getLeftTriggerAxis() > 0.5;
        BooleanSupplier turboTrigger = () -> controller.getRightTriggerAxis() > 0.5;

        Supplier<ChassisSpeeds> speedsSupplier = new SwerveTeleopSpeedSupplier(
                leftX,
                leftY,
                rightX,
                turboTrigger,
                sniperTrigger);

        return new SwerveTeleopCommand(
                speedsSupplier,
                subsystem,
                poseSupplier,
                speedConsumer);
    }
}
