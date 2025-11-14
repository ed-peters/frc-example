package frc.robot.subsystems.intake;

import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

/**
 * Configuration properties for the sample intake subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class IntakeConfig {

    /** Gear ratio of intake wheel */
    public static final double gearRatio = 1.0;

    /** Diameter of wheel in inches */
    public static final double wheelDiameterInches = 4.0;

    /** Circumference of wheel in feet */
    public static final double wheelCircumferenceFeet = (wheelDiameterInches * Math.PI) / 12.0;

    /** Weight of the wheel in pounds */
    public static final double wheelMassLbs = 0.62;

    /** Should the motor brake be enabled by default? */
    public static final boolean defaultBrakeEnabled = true;

    /**
     * Feedforward and feedback tuning constants; you'll determine values
     * for these during tuning
     */
    public static final DoubleSupplier v = Util.pref("IntakeSubsystem/kV", 1.0);
    public static final DoubleSupplier p = Util.pref("IntakeSubsystem/kP", 1.0);
    public static final DoubleSupplier d = Util.pref("IntakeSubsystem/kD", 1.0);

    /** How close to target velocity will we consider being "good enough"? */
    public static final DoubleSupplier tolerance = Util.pref("IntakeSubsystem/Tolerance", 1.0);

    /** Preset speeds for different tasks, in feet per second */
    public static final DoubleSupplier collectSpeed = Util.pref("IntakeSubsystem/CollectSpeed", 1.0);
    public static final DoubleSupplier repositionSpeed = Util.pref("IntakeSubsystem/RepositionSpeed", 1.0);
    public static final DoubleSupplier feedSpeed = Util.pref("IntakeSubsystem/FeedSpeed", 1.0);
    public static final DoubleSupplier ejectSpeed = Util.pref("IntakeSubsystem/EjectSpeed", 1.0);

    /** Preset durations for time-based tasks, in seconds */
    public static final DoubleSupplier ejectSeconds = Util.pref("IntakeSubsystem/EjectSeconds", 1.0);
    public static final DoubleSupplier repositionSeconds = Util.pref("IntakeSubsystem/RepositionSeconds", 1.0);

}
