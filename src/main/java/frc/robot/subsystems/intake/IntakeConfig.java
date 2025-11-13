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

    /** Feedforward tuning parameter */
    public static final DoubleSupplier v = Util.pref("IntakeSubsystem/V", 1.0);

    /** Feedback tuning parameter */
    public static final DoubleSupplier p = Util.pref("IntakeSubsystem/P", 1.0);

    /** Feedback tuning parameter */
    public static final DoubleSupplier d = Util.pref("IntakeSubsystem/D", 1.0);

    /** How close to target velocity will we consider being "good enough"? */
    public static final DoubleSupplier tolerance = Util.pref("IntakeSubsystem/Tolerance", 1.0);

    /** Preset speed for collecting gamepieces */
    public static final DoubleSupplier collectSpeed = Util.pref("IntakeSubsystem/CollectSpeed", 1.0);

    /** Preset speed for repositioning gamepieces in the intake */
    public static final DoubleSupplier repositionSpeed = Util.pref("IntakeSubsystem/RepositionSpeed", 1.0);

    /** Duration for running during the repositioning movement */
    public static final DoubleSupplier repositionSeconds = Util.pref("IntakeSubsystem/RepositionSeconds", 1.0);

    /** Preset speed for feeding gamepieces to the shooter */
    public static final DoubleSupplier feedSpeed = Util.pref("IntakeSubsystem/FeedSpeed", 1.0);

    /** Preset speed for clearing the intake */
    public static final DoubleSupplier ejectSpeed = Util.pref("IntakeSubsystem/EjectSpeed", 1.0);

    /** Duration for running during the clearing movement */
    public static final DoubleSupplier ejectSeconds = Util.pref("IntakeSubsystem/EjectSeconds", 1.0);
    
}
