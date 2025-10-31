package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration properties for the sample elevator subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class ElevatorConfig {

    /** How many inches the elevator travels per motor rotation */
    public static final double inchesPerRotation = 1.35;

    /** Minimum height of elevator in inches */
    public static final DoubleSupplier minHeight = pref("ElevatorSubsystem/MinHeight", 1.0);

    /** Maximum height of elevator in inches */
    public static final DoubleSupplier maxHeight = pref("ElevatorSubsystem/MaxHeight", 90.0);

    /** Maximum velocity when moving between presets */
    public static final DoubleSupplier maxVelocity = pref("ElevatorSubsystem/MaxVelocity", 30.0);

    /** Acceleration factor for moving between presets (2.0 means top speed in 0.5s) */
    public static final DoubleSupplier accelerationFactor = pref("ElevatorSubsystem/Acceleration", 2.0);

    /** Maximum voltage in teleop for moving elevator upwards */
    public static final DoubleSupplier maxTeleopVolts = pref("ElevatorSubsystem/TeleopVolts", 6.0);

    /** PID constant */
    public static final DoubleSupplier p = pref("ElevatorSubsystem/P", 3.0);

    /** PID constant */
    public static final DoubleSupplier d = pref("ElevatorSubsystem/D", 0.3);

    /** Maximum feedback from PID controls */
    public static final DoubleSupplier maxFeedback = pref("ElevatorSubsystem/MaxFeedback", 2.0);

    /** Feedforward constant */
    public static final DoubleSupplier g = pref("ElevatorSubsystem/G", 1.3028);

    /** Feedforward constant */
    public static final DoubleSupplier v = pref("ElevatorSubsystem/V", 0.1);

    /** Tolerance for how close we have to be before we decide we're "on target" */
    public static final DoubleSupplier tolerance = pref("ElevatorSubsystem/Tolerance", 0.25);

    /** Preset height */
    public static final DoubleSupplier presetL1 = pref("ElevatorSubsystem/L1", 10.0);

    /** Preset height */
    public static final DoubleSupplier presetL2 = pref("ElevatorSubsystem/L2", 40.0);

    /** Preset height */
    public static final DoubleSupplier presetL3 = pref("ElevatorSubsystem/L3", 80.0);

}
