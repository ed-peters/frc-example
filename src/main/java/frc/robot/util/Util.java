package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;

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

    /** Speed with all zeros */
    public static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds(0.0, 0.0, 0.0);

    /** Speed with no values */
    public static final ChassisSpeeds NAN_SPEED = new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);

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

    /** @return true if the driver's station says we're blue alliance */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }

    // ===========================================================
    // BASIC MATH STUFF
    // ===========================================================

    /**
     * @return the value chopped to three decimal places (this can make it
     * easier to read on the dashboard)
     */
    public static double chopDigits(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }

    /**
     * @return the supplier wrapped to return values chopped to three
     * decimal places (this can make it easier to read on the dashboard)
     */
    public static DoubleSupplier chopDigits(DoubleSupplier supplier) {
        return () -> Math.round(supplier.getAsDouble() * 1000.0) / 1000.0;
    }

    /**
     * @return the supplied value clamped to an interval around 0 defined
     * by the supplied limit
     */
    public static double applyClamp(double value, DoubleSupplier limitSupplier) {
        double limit = limitSupplier.getAsDouble();
        return MathUtil.clamp(value, -limit, limit);
    }
    
    /**
     * @return the supplied value clamped to an interval defined by the
     * supplied limits
     */
    public static double applyClamp(double value, DoubleSupplier lowerLimitSupplier, DoubleSupplier upperLimitSupplier) {
        double lo = lowerLimitSupplier.getAsDouble();
        double hi = upperLimitSupplier.getAsDouble();
        return MathUtil.clamp(value, lo, hi);
    }

    public static double clampVolts(double volts) {
        return MathUtil.clamp(volts, -MAX_VOLTS, MAX_VOLTS);
    }

    /** @return the supplied angle wrapped to (-180, 180) */
    public static double angleModulus(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    // ===========================================================
    // PREFERENCES
    // ===========================================================

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
