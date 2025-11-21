package frc.robot.subsystems.shooter;

import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

/**
 * Configuration properties for the sample shooter subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class ShooterConfig {

    /** Configuration parameters for the motor */
    public static final int maxAmps = 40;
    public static final double rampRate = 0.1;

    /** Gear ratio of intake wheel */
    public static final double gearRatio = 1.0;

    /** Diameter of wheel in inches */
    public static final double wheelDiameterInches = 4.0;

    /** Circumference of wheel in feet */
    public static final double wheelCircumferenceFeet = (wheelDiameterInches * Math.PI) / 12.0;

    /** Weight of the wheel in pounds */
    public static final double wheelMassLbs = 0.62;

    /** Should the motor brake be enabled by default? */
    public static final boolean defaultBrakeEnabled = false;

    /**
     * Feedforward and feedback tuning constants; you'll determine values
     * for these during tuning
     */
    public static final DoubleSupplier v = Util.pref("ShooterSubsystem/kV", 1.0);
    public static final DoubleSupplier p = Util.pref("ShooterSubsystem/kP", 1.0);

    /** How close to target velocity will we consider being "good enough"? */
    public static final DoubleSupplier tolerance = Util.pref("ShooterSubsystem/Tolerance", 1.0);

    /** Preset speeds for different shots, in feet per second */
    public static final DoubleSupplier speed1 = Util.pref("ShooterSubsystem/CollectSpeed", 1.0);
    public static final DoubleSupplier speed2 = Util.pref("ShooterSubsystem/RepositionSpeed", 1.0);
    public static final DoubleSupplier speed3 = Util.pref("ShooterSubsystem/FeedSpeed", 1.0);

}
