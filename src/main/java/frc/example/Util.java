package frc.example;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;

import java.util.Optional;
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

    /** Speed with all zeros */
    public static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds(0.0, 0.0, 0.0);

    /** Speed with no values */
    public static final ChassisSpeeds NAN_SPEED = new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);

    // ===========================================================
    // MATH STUFF
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

    // ===========================================================
    // DRIVING STUFF
    // ===========================================================

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

    // ===========================================================
    // FIELD STUFF
    // ===========================================================

    static AprilTagFieldLayout layout = null;

    /** @return information about the supplied AprilTag */
    public static Pose2d getAprilTagPose(int id) {
        if (layout == null) {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        }
        Optional<Pose3d> pose = layout.getTagPose(id);
        return pose.map(Pose3d::toPose2d).orElse(null);
    }

    /** @return true if the driver's station says we're blue alliance */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
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
     * @return a {@link BooleanSupplier} providing access to the specified configuration
     * preference (this also has the side effect of saving the value, if it's not already
     * persisted on the robot)
     */
    public static BooleanSupplier pref(String name, boolean defaultValue) {
        Preferences.initBoolean(name, defaultValue);
        return () -> Preferences.getBoolean(name, defaultValue);
    }

    /**
     * @return a {@link DoubleSupplier} providing access to the specified configuration
     * preference (this also has the side effect of saving the value, if it's not already
     * persisted on the robot)
     */
    public static DoubleSupplier pref(String name, double defaultValue) {
        Preferences.initDouble(name, defaultValue);
        return () -> Preferences.getDouble(name, defaultValue);
    }
}
