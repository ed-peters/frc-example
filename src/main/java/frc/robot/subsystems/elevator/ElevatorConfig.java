package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration for a full-featured elevator subsystem
 */
public class ElevatorConfig {

    /** How many inches the elevator travels per motor rotation */
    public static final double inchesPerRotation = 1.35;

    /** Minimum height of elevator in inches */
    public static final DoubleSupplier minHeight = pref("elev_min_height", 1.0);

    /** Maximum height of elevator in inches */
    public static final DoubleSupplier maxHeight = pref("elev_max_height", 90.0);

    /** Maximum velocity when moving between presets */
    public static final DoubleSupplier maxVelocity = pref("elev_max_velocity", 30.0);

    /** Acceleration factor for moving between presets (2.0 means top speed in 0.5s) */
    public static final DoubleSupplier accelerationFactor = pref("elev_acceleration", 2.0);

    /** Maximum voltage in teleop for moving elevator upwards */
    public static final DoubleSupplier maxVoltsUp = pref("elev_max_volts_up", 6.0);

    /** Maximum voltage in teleop for moving elevator downwards */
    public static final DoubleSupplier maxVoltsDown = pref("elev_max_volts_down", 2.0);

    /** PID constant */
    public static final DoubleSupplier p = pref("elev_p", 0.4);

    /** PID constant */
    public static final DoubleSupplier d = pref("elev_d", 0.3);

    /** Maximum feedback from PID controls */
    public static final DoubleSupplier maxFeedback = pref("elev_max_feedback", 2.0);

    /** Feedforward constant */
    public static final DoubleSupplier g = pref("elev_g", 1.3028);

    /** Feedforward constant */
    public static final DoubleSupplier v = pref("elev_v", 0.0991);

    /** Tolerance for how close we have to be before we decide we're "on target" */
    public static final DoubleSupplier positionTolerance = pref("elev_pos_tolerance", 0.25);

    /** Preset height */
    public static final DoubleSupplier heightLevel1 = pref("elev_level1", 10.0);

    /** Preset height */
    public static final DoubleSupplier heightLevel2 = pref("elev_level2", 40.0);

    /** Preset height */
    public static final DoubleSupplier heightLevel3 = pref("elev_level3", 80.0);

}
