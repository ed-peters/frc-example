package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration properties for the sample swerve subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class SwerveAlignConfig {

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
    // TRAJECTORY
    // ==================================================================

    /** Properties for driving a trajectory (translation) */
    public static DoubleSupplier trajectoryTranslateP = pref("AlignTrajectory/TranslateP", 10.0);
    public static DoubleSupplier trajectoryTranslateD = pref("AlignTrajectory/TranslateD", 10.0);
    public static DoubleSupplier trajectoryMaxTranslate = pref("AlignTrajectory/MaxTranslate", 10.0);

    /** Properties for driving a trajectory (rotation) */
    public static DoubleSupplier trajectoryRotateP = pref("AlignTrajectory/RotateP", 10.0);
    public static DoubleSupplier trajectoryRotateD = pref("AlignTrajectory/RotateD", 10.0);
    public static DoubleSupplier trajectoryMaxRotate = pref("AlignTrajectory/MaxRotate", 10.0);

    /** Properties for driving a trajectory (all directions) */
    public static DoubleSupplier trajectoryAcceleration = pref("AlignTrajectory/Acceleration", 10.0);
    
    // ==================================================================
    // PID
    // ==================================================================

    /** Properties for driving via PID */
    public static DoubleSupplier pidP = pref("AlignTrajectory/P", 10.0);
    public static DoubleSupplier pidD = pref("AlignPID/D", 10.0);
    public static DoubleSupplier pidMaxVelocity = pref("AlignPID/MaxVelocity", 10.0);
    public static DoubleSupplier pidTolerance = pref("AlignPID/Tolerance", 10.0);
    public static DoubleSupplier pidAcceleration = pref("AlignPID/Acceleration", 10.0);

}
