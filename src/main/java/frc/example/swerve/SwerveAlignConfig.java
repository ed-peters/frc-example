package frc.example.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.example.Util.pref;

/**
 * Configuration properties for the sample swerve subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class SwerveAlignConfig {

    // ==================================================================
    // ALIGN TO HEADING
    // ==================================================================

    /** Feedback constant for aligning to a heading: P */
    public static DoubleSupplier toHeadingP = pref("AlignToHeading/P", 10.0);

    /** Feedback constant for aligning to a heading: D */
    public static DoubleSupplier toHeadingD = pref("AlignToHeading/D", 10.0);

    /** Tolerance in degrees for aligning to a heading */
    public static DoubleSupplier toHeadingTolerance = pref("AlignToHeading/Tolerance", 10.0);

    /** Maximum turn rate for aligning to a heading in degrees per second */
    public static DoubleSupplier toHeadingMaxFeedback = pref("AlignToHeading/MaxFeedback", 10.0);

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
    // FOLLOW TRAJECTORY
    // ==================================================================

    public static DoubleSupplier trajectoryMinDistance = pref("swerve_trajectory_min_distance", 10.0);

    public static DoubleSupplier trajectoryMaxVelocity = pref("swerve_trajectory_max_velocity", 10.0);

    public static DoubleSupplier trajectoryAccelerationFactor = pref("swerve_trajectory_acceleration", 10.0);

}
