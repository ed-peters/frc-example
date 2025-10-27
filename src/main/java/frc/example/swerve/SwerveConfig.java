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
public class SwerveConfig {

    /** Distance from center of front wheel to center of back wheel */
    public static final double WHEEL_BASE_M = Units.inchesToMeters(26.25);

    /** Distance from center line of left wheel to center line of right wheel */
    public static final double TRACK_WIDTH_M = Units.inchesToMeters(26.5);

    /** Wheel diameter */
    public static final double WHEEL_DIAM_M = Units.inchesToMeters(3.0);

    /** Wheel circumference */
    public static final double WHEEL_CIRC_M = WHEEL_DIAM_M * Math.PI;

    /** Kinematics for a rectangular robot based on the dimensions above */
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
            new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
            new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2));

    /** Maximum wheel speed - this is applied for hardware safety */
    public static DoubleSupplier maximumWheelSpeed = pref("swerve_max_speed", 15.0);

    /** Enable/disable cosine compensation */
    public static BooleanSupplier cosineCompensation = pref("swerve_cosine", false);

    // ==================================================================
    // ALIGN TO HEADING
    // ==================================================================

    /** Feedback constant for aligning to a heading: P */
    public static DoubleSupplier alignHeadingP = pref("swerve_drift_max_feedback", 10.0);

    /** Feedback constant for aligning to a heading: D */
    public static DoubleSupplier alignHeadingD = pref("swerve_drift_max_feedback", 10.0);

    /** Tolerance in degrees for aligning to a heading */
    public static DoubleSupplier alignHeadingTolerance = pref("swerve_drift_max_feedback", 10.0);

    /** Maximum turn rate for aligning to a heading in degrees per second */
    public static DoubleSupplier alignHeadingMax = pref("swerve_drift_max_feedback", 10.0);

    // ==================================================================
    // FOLLOW TRAJECTORY
    // ==================================================================

    public static DoubleSupplier trajectoryMinDistance = pref("swerve_trajectory_min_distance", 10.0);

    public static DoubleSupplier trajectoryMaxVelocity = pref("swerve_trajectory_max_velocity", 10.0);

    public static DoubleSupplier trajectoryAccelerationFactor = pref("swerve_trajectory_acceleration", 10.0);

}
