package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration properties for swerve targeting commands. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class SwerveAutoConfig {

    // ==================================================================
    // ROTATING
    // ==================================================================

    /**
     * Maximum velocity and acceleration for autorotation in degrees per
     * second; when acceleration is twice velocity we will reach top speed
     * in 0.5s
     */
    public static DoubleSupplier rotateMaxVelocity = pref("SwerveAutoRotate/MaxVelocity", 720.0);
    public static DoubleSupplier rotateMaxAcceleration = pref("SwerveAutoRotate/MaxAcceleration", 1440.0);

    /**
     * Feedback constants for angle correction during auto rotation
     */
    public static DoubleSupplier rotateP = pref("SwerveAutoRotate/kP", 0.4);
    public static DoubleSupplier rotateD = pref("SwerveAutoRotate/kD", 0.0);
    public static DoubleSupplier rotateMaxFeedback = pref("SwerveAutoRotate/MaxFeedback", 1.0);

    /**
     * How "close" in degrees to the target angle will we consider
     * successful?
     */
    public static DoubleSupplier rotateTolerance = pref("SwerveAutoRotate/Tolerance", 1.0);

    // ==================================================================
    // TRANSLATING
    // ==================================================================

    /**
     * Maximum velocity and acceleration for auto translation in feet per
     * second; when acceleration is twice velocity we will reach top speed
     * in 0.5s
     */
    public static DoubleSupplier translateMaxVelocity = pref("SwerveAutoTranslate/MaxVelocity", 15.0);
    public static DoubleSupplier translateMaxAcceleration = pref("SwerveAutoTranslate/MaxAcceleration", 30.0);

    /**
     * Feedback constants for position correction during auto translation
     */
    public static DoubleSupplier translateP = pref("SwerveAutoTranslate/kP",2.0);
    public static DoubleSupplier translateD = pref("SwerveAutoTranslate/kD", 0.0);
    public static DoubleSupplier translateMaxFeedback = pref("SwerveAutoTranslate/MaxFeedback", 10.0);

    /**
     * How "close" in inches to the target position will we consider
     * successful?
     */
    public static DoubleSupplier translateTolerance = pref("SwerveAutoTranslate/Tolerance", 2.0);

}
