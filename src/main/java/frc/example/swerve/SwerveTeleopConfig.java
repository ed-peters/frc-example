package frc.example.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.example.Util.pref;

/**
 * Configuration properties for the sample swerve subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class SwerveTeleopConfig {

    /** Deadband applied to joystick input in teleop */
    public static final DoubleSupplier deadband = pref("SwerveTeleop/Deadband", 0.1);

    /** Exponent applied to joystick input in teleop */
    public static final DoubleSupplier exponent = pref("SwerveTeleop/Exponent", 2.0);

    /** Top translate (X/Y) speed in teleop */
    public static final DoubleSupplier maxTranslate = pref("SwerveTeleop/MaxTranslate", 10.0);

    /** Top rotation speed in teleop */
    public static final DoubleSupplier maxRotate = pref("SwerveTeleop/MaxRotate", 180.0);

    /** Multiplied by top speed in turbo mode */
    public static final DoubleSupplier turboFactor = pref("SwerveTeleop/TurboFactor", 2.0);

    /** Multiplied by top speed in sniper mode */
    public static final DoubleSupplier sniperFactor = pref("SwerveTeleop/SniperFactor", 0.5);

    /** Does the sniper multiple apply to rotation too? */
    public static final BooleanSupplier applySniperToRotate = pref("SwerveTeleop/SniperOnRotate?", true);

    /** Should we apply slew rate limiting in teleop? */
    public static final BooleanSupplier applySlew = pref("SwerveTeleop/ApplySlew?", false);

    /** Slew rate limit () */
    public static final DoubleSupplier slewRate = pref("SwerveTeleop/SlewRate", 4.0);

    /** Driver relative mode */
    public static final BooleanSupplier fieldRelative = pref("SwerveTeleop/FiedRelative?", true);

    /** Turn drift correction on/off */
    public static BooleanSupplier driftCorrection = pref("SwerveTeleop/DriftCorrection?", true);

    /** Feedback constant for drift correction: P */
    public static DoubleSupplier driftP = pref("SwerveTeleop/DriftP", 2.0);

    /** Maximum feedback for drift correction, in degrees per second */
    public static DoubleSupplier driftMaxFeedback = pref("SwerveTeleop/DriftMaxFeedback", 10.0);

}
