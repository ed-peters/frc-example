package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration properties for swerve targeting commands. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class SwerveTargetingConfig {

    // ==================================================================
    // ALIGN TO HEADING
    // ==================================================================

    /** Properties for aligning to a heading */
    public static DoubleSupplier toHeadingP = pref("AlignToHeading/P", 10.0);
    public static DoubleSupplier toHeadingD = pref("AlignToHeading/D", 0.0);
    public static DoubleSupplier toHeadingTolerance = pref("AlignToHeading/Tolerance", 1.0);
    public static DoubleSupplier toHeadingMaxFeedback = pref("AlignToHeading/MaxFeedback", 120.0);

    // ==================================================================
    // ALIGN TO APRIL TAG
    // ==================================================================

    /** Properties for aligning to an AprilTag (x direction) */
    public static DoubleSupplier tagOffsetP = pref("AlignToTag/OffsetP", 10.0);
    public static DoubleSupplier tagOffsetD = pref("AlignToTag/OffsetD", 10.0);
    public static DoubleSupplier tagOffsetTarget = pref("AlignToTag/OffsetTarget", 10.0);
    public static DoubleSupplier tagOffsetTolerance = pref("AlignToTag/OffsetTolerance", 10.0);

    /** Properties for aligning to an AprilTag (y direction) */
    public static DoubleSupplier tagAreaP = pref("AlignToTag/AreaP", 10.0);
    public static DoubleSupplier tagAreaD = pref("AlignToTag/AreaD", 10.0);
    public static DoubleSupplier tagAreaTarget = pref("AlignToTag/AreaTarget", 10.0);
    public static DoubleSupplier tagAreaTolerance = pref("AlignToTag/AreaTolerance", 10.0);

    /** Properties for aligning to an AprilTag (both directions) */
    public static DoubleSupplier tagMaxFeedback = pref("AlignToTag/MaxFeedback", 10.0);

    // ==================================================================
    // POSE OFFSET
    // ==================================================================

    /** Properties for driving to a pose offset */
    public static DoubleSupplier toPoseP = pref("AlignPose/P", 0.0);
    public static DoubleSupplier toPoseD = pref("AlignPose/D", 0.0);
    public static DoubleSupplier toPoseMaxFeedback = pref("AlignPose/MaxFeedback", 10.0);
    public static DoubleSupplier toPoseMaxVelocity = pref("AlignPose/MaxVelocity", 48.0);
    public static DoubleSupplier toPoseTolerance = pref("AlignPose/Tolerance", 4);
    public static DoubleSupplier toPoseAcceleration = pref("AlignPose/Acceleration", 2.0);

}
