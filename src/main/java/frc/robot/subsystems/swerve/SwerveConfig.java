package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

public class SwerveConfig {

    public static final double WHEEL_BASE_M = Units.inchesToMeters(26.25);
    public static final double TRACK_WIDTH_M = Units.inchesToMeters(26.5);
    public static final double WHEEL_DIAM_M = Units.inchesToMeters(3.0);
    public static final double WHEEL_CIRC_M = WHEEL_DIAM_M * Math.PI;

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
    // TELEOP CONFIG
    // ==================================================================

    /** Deadband applied to joystick input in teleop */
    public static final DoubleSupplier deadband = pref("swerve_teleop_deadband", 0.1);

    /** Exponent applied to joystick input in teleop */
    public static final DoubleSupplier exponent = pref("swerve_teleop_exponent", 2.0);

    /** Top translate (X/Y) speed in teleop */
    public static final DoubleSupplier maxTeleopTranslate = pref("swerve_teleop_max_translate", 10.0);

    /** Top rotation speed in teleop */
    public static final DoubleSupplier maxTeleopRotate = pref("swerve_teleop_max_rotate", 180.0);

    /** Multiplied by top speed in turbo mode */
    public static final DoubleSupplier turboFactor = pref("swerve_teleop_turbo_factor", 2.0);

    /** Multiplied by top speed in sniper mode */
    public static final DoubleSupplier sniperFactor = pref("swerve_teleop_sniper_factor", 0.5);

    /** Does the sniper multiple apply to rotation too? */
    public static final BooleanSupplier applySniperToRotate = pref("swerve_teleop_apply_sniper_to_rotate", true);

    /** Should we apply slew rate limiting in teleop? */
    public static final BooleanSupplier applySlew = pref("swerve_teleop_apply_slew", false);

    /** Slew rate limit () */
    public static final DoubleSupplier slewRate = pref("swerve_teleop_slew_rate", 4.0);

    /** Driver relative mode */
    public static final BooleanSupplier fieldRelative = pref("swerve_field_relative", true);

    /** Turn drift correction on/off */
    public static BooleanSupplier teleopDriftCorrection = pref("swerve_drift_correction", true);

    /** Feedback constant for drift correction: P */
    public static DoubleSupplier teleopDriftP = pref("swerve_drift_p", 2.0);

    /** Maximum feedback for drift correction, in degrees per second */
    public static DoubleSupplier teleopDriftMaxFeedback = pref("swerve_drift_max_feedback", 10.0);

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
