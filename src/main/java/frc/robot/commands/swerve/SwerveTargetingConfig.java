package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration properties for swerve targeting commands. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class SwerveTargetingConfig {

    /** Implements a "quiet mode" to prevent spamming the dashboard during competition */
    public static final boolean enableLogging = false;

    // ==================================================================
    // ROTATING
    // ==================================================================

    /** Properties for aligning to a heading */
    public static DoubleSupplier rotateP = pref("SwerveRotate/P", 10.0);
    public static DoubleSupplier rotateD = pref("SwerveRotate/D", 0.0);
    public static DoubleSupplier rotateTolerance = pref("SwerveRotate/Tolerance", 1.0);
    public static DoubleSupplier rotateMaxVelocity = pref("SwerveRotate/MaxVelocity", 120.0);

    // ==================================================================
    // TRANSLATING
    // ==================================================================

    /** Properties for driving to a pose offset */
    public static DoubleSupplier translateP = pref("SwerveTranslate/P",2.0);
    public static DoubleSupplier translateD = pref("SwerveTranslate/D", 0.0);
    public static DoubleSupplier translateMaxFeedback = pref("SwerveTranslate/MaxFeedback", 10.0);
    public static DoubleSupplier translateMaxVelocity = pref("SwerveTranslate/MaxVelocity", 48.0);
    public static DoubleSupplier translateMaxAcceleration = pref("SwerveTranslate/MaxAcceleration", 96.0);
    public static DoubleSupplier translateTolerance = pref("SwerveTranslate/Tolerance", 2.0);

}
