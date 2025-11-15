package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Basic utility functions that can be useful
 */
public class Util {

    // ===========================================================
    // USEFUL CONSTANTS
    // ===========================================================

    /** Maximum volts for motors */
    public static final double MAX_VOLTS = 12.0;

    /** Time slice between calls to periodic() */
    public static final double DT = 0.02;

    /** Zero heading */
    public static final Rotation2d ZERO_ROTATION = new Rotation2d();

    /** Heading with no value */
    public static final Rotation2d NAN_ROTATION = new Rotation2d(Double.NaN);

    /** Pose with all zeros */
    public static final Pose2d ZERO_POSE = new Pose2d();

    /** Pose with no values */
    public static final Pose2d NAN_POSE = new Pose2d(Double.NaN, Double.NaN, NAN_ROTATION);

    /** State with all zeros */
    public static final State ZERO_STATE = new State();

    /** State with no values */
    public static final State NAN_STATE = new State(Double.NaN, Double.NaN);

    /** Speed with all zeros */
    public static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds(0.0, 0.0, 0.0);

    /** Speed with no values */
    public static final ChassisSpeeds NAN_SPEED = new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);

    // ===========================================================
    // MISC STUFF
    // ===========================================================

    /**
     * @return the supplied value, clamped to +/- 12 volts
     */
    public static double clampVolts(double volts) {
        return MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
    }

    /**
     * @return the supplied value, clamped within limits defined by the supplier
     */
    public static double applyClamp(double value, DoubleSupplier limit) {
        double lim = limit.getAsDouble();
        return MathUtil.clamp(value, -lim, lim);
    }

    /**
     * @return the supplied angle wrapped to (-180, 180)
     */
    public static double angleModulus(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    /**
     * @return a new supplier that returns true when the original supplier
     * (a) returned true last time and (b) returns false this time (this
     * captures a "falling edge", for example when a button was pressed and
     * then gets released)
     */
    public static BooleanSupplier fallingEdge(BooleanSupplier supplier) {
        AtomicBoolean wasTrue = new AtomicBoolean(false);
        return () -> {
            boolean isTrue = supplier.getAsBoolean();
            boolean hasFallen = wasTrue.get() && !isTrue;
            wasTrue.set(isTrue);
            return hasFallen;
        };
    }


    /**
     * Resets a PID controller to use the most recent tuning constants
     * and clear out accumulated error
     */
    public static void resetPid(PIDController pid,
                         DoubleSupplier p,
                         DoubleSupplier d,
                         DoubleSupplier tolerance) {
        pid.setP(p.getAsDouble());
        pid.setD(d.getAsDouble());
        pid.setTolerance(tolerance.getAsDouble());
        pid.reset();
    }

    // ===========================================================
    // DRIVING & FIELD STUFF
    // ===========================================================

    // holds the field layout once we've loaded it (which happens the
    // first time we request id info)
    static AprilTagFieldLayout layout = null;

    /** @return information about the supplied AprilTag */
    public static Pose2d getAprilTagPose(int id) {
        if (layout == null) {

            // in 2025 there was an issue where the field measurements were
            // different depending on which vendor made the field; if this
            // comes up again you might need to make a configurable property
            // to indicate which field layout to use
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        }
        Optional<Pose3d> pose = layout.getTagPose(id);
        return pose.map(Pose3d::toPose2d).orElse(null);
    }

    /** @return true if the driver's station says we're blue alliance */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }

    /**
     * @return true if the supplied speeds include an XY translation
     * greater than the supplied tolerance (in feet per second) in
     * either direction
     */
    public static boolean isTranslatingMoreThan(ChassisSpeeds speeds, DoubleSupplier tolerance) {
        double tol = tolerance.getAsDouble();
        return Math.abs(Units.metersToFeet(speeds.vxMetersPerSecond)) > tol
                || Math.abs(Units.metersToFeet(speeds.vyMetersPerSecond)) > tol;
    }

    /** @return true if the supplied speeds include a rotation */
    public static boolean isRotating(ChassisSpeeds speeds) {
        return Math.abs(speeds.omegaRadiansPerSecond) > 0.0;
    }

    /** Prefix & loggers for logging pose structs */
    public static final String POSE_LOGGING_PREFIX = "SmartDashboard/SwerveDriveSubsystem/Structs/";

    static final Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

    /**
     * Publish a pose to the dashboard (automatically adds the "SmartDashboard"
     * prefix so it will show up under that topic in the dashboard)
     */
    public static void publishPose(String key, Pose2d val) {

        // see if a publisher already exists
        StructPublisher<Pose2d> publisher = posePublishers.get(key);

        // create it if it doesn't (we add the SmartDashboard prefix so
        // it shows up next to other values we publish)
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic(POSE_LOGGING_PREFIX+key, Pose2d.struct)
                    .publish();
            posePublishers.put(key, publisher);
        }

        publisher.set(val);
    }

    // ===========================================================
    // LOGGING & PREFERENCES
    // ===========================================================

    /**
     * Formats & writes a message to Rio log after formatting (adds a
     * newline to the end of the message if there isn't one)
     */
    public static void log(String message, Object... args) {
        System.out.printf(message, args);
        if (!message.endsWith("%n")) {
            System.out.println();
        }
    }

    /**
     * Fetches a number straight from NetworkTables - this is inefficient
     * and should only be used for debugging
     */
    public static double fetchPreference(String key) {
        return NetworkTableInstance.getDefault()
                .getTable("Preferences")
                .getDoubleTopic(key)
                .getEntry(-100.0)
                .getAsDouble();
    }

    /**
     * @return a {@link BooleanSupplier} providing access to the specified configuration
     * preference (this also has the side effect of saving the value, if it's not already
     * persisted on the robot)
     */
    public static BooleanSupplier pref(String name, boolean defaultValue) {
        log("[util] registering pref %s = %s", name, defaultValue);
        Preferences.initBoolean(name, defaultValue);
        return () -> Preferences.getBoolean(name, defaultValue);
    }

    /**
     * @return a {@link DoubleSupplier} providing access to the specified configuration
     * preference (this also has the side effect of saving the value, if it's not already
     * persisted on the robot)
     */
    public static DoubleSupplier pref(String name, double defaultValue) {
        log("[util] registering pref %s = %.2f", name, defaultValue);
        Preferences.initDouble(name, defaultValue);
        return () -> Preferences.getDouble(name, defaultValue);
    }
}
