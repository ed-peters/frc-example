package frc.robot.commands.swerve;

/*
 * Code pulled from <a href="https://www.chiefdelphi.com/t/introducing-antitipping-lib/508284">here</a>
 */

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.commands.swerve.SwerveTeleopConfig.antiTippingSpeed;
import static frc.robot.commands.swerve.SwerveTeleopConfig.antiTippingThreshold;
import static frc.robot.commands.swerve.SwerveTeleopConfig.antiTippingP;

/**
 * {@code AntiTipping} provides a proportional correction system to prevent
 * the robot from tipping over during operation.
 *
 * <p>It uses pitch and roll measurements (in degrees) to detect excessive
 * inclination and computes a correction velocity in the opposite direction
 * of the tilt. The resulting correction can be added to the robot’s
 * translational velocity to help stabilize it.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>Instantiate with pitch and roll suppliers and initial configuration parameters.
 *   <li>Call {@link #get()} periodically (e.g. once per control loop).
 *   <li>Add the result to your drive command.
 * </ol>
 *
 * <p>The correction is purely proportional: {@code correction = kP * inclinationMagnitude}, and
 * clamped to {@code maxCorrectionSpeed}.
 *
 * @since 2025
 */
public class SwerveTeleopAntiTipper implements Supplier<ChassisSpeeds> {

    private final DoubleSupplier pitchSupplier;
    private final DoubleSupplier rollSupplier;

    private double inclinationMagnitude = 0.0;
    private double yawDirectionDeg = 0.0;
    private boolean isTipping = false;
    private ChassisSpeeds lastSpeeds;

    /**
     * Creates a new {@code AntiTipping} instance.
     *
     * @param pitchSupplier supplier providing the current pitch angle (degrees)
     * @param rollSupplier supplier providing the current roll angle (degrees)
     */
    public SwerveTeleopAntiTipper(
            DoubleSupplier pitchSupplier,
            DoubleSupplier rollSupplier) {

        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
        this.lastSpeeds = Util.NAN_SPEED;

        SmartDashboard.putData("SwerveAntiTipping", builder -> {
            builder.addDoubleProperty("LastTiltDirection", () -> yawDirectionDeg, null);
            builder.addDoubleProperty("LastTiltMagnitude", () -> inclinationMagnitude, null);
            builder.addDoubleProperty("LastSpeedX", () -> Units.metersToFeet(lastSpeeds.vxMetersPerSecond), null);
            builder.addDoubleProperty("LastSpeedY", () -> Units.metersToFeet(lastSpeeds.vyMetersPerSecond), null);
            builder.addBooleanProperty("IsTilting?", () -> isTipping, null);
        });
    }

    /**
     * Updates tipping detection and computes the proportional correction.
     *
     * <p>This method updates internal values (pitch, roll, direction, magnitude, etc.) and generates
     * a correction {@link ChassisSpeeds} vector that can be applied to stabilize the robot.
     * It should be called periodically (e.g. once per control loop).
     */
    @Override
    public ChassisSpeeds get() {

        double pitch = pitchSupplier.getAsDouble();
        double roll = rollSupplier.getAsDouble();

        double ttd = antiTippingThreshold.getAsDouble();
        isTipping = Math.abs(pitch) > ttd || Math.abs(roll) > ttd;

        // Tilt direction (the direction the robot is falling towards)
        Rotation2d tiltDirection = new Rotation2d(Math.atan2(-roll, -pitch));
        yawDirectionDeg = tiltDirection.getDegrees();

        // Tilt magnitude (hypotenuse of pitch and roll)
        inclinationMagnitude = Math.hypot(pitch, roll);

        // Proportional correction
        double mcs = antiTippingSpeed.getAsDouble();
        double correctionSpeed = antiTippingP.getAsDouble() * -inclinationMagnitude;
        correctionSpeed = MathUtil.clamp(correctionSpeed, -mcs, mcs);

        // Correction vector (field-relative)
        Translation2d correctionVector =
                new Translation2d(0, 1).rotateBy(tiltDirection).times(correctionSpeed);

        // WPILib convention: Y axis inverted
        lastSpeeds = new ChassisSpeeds(
                correctionVector.getX(),
                -correctionVector.getY(),
                0);
        return lastSpeeds;
    }
}