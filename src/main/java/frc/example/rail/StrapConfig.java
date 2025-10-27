package frc.example.rail;

import frc.example.Util;

import java.util.function.DoubleSupplier;

/**
 * Configuration properties for the sample intake subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class StrapConfig {

    /** Should the motor brake be enabled by default? */
    public static final boolean defaultBrakeEnabled = true;

    /** How many volts to apply when closing the strap */
    public static final DoubleSupplier closeVolts = Util.pref("RailSubsystem/CloseVolts", 9.0);

    /** Velocity below which the motor might be considered stalled */
    public static final DoubleSupplier closeStallVelocity = Util.pref("RailSubsystem/CloseStallVelocity", 1.0);

    /** How many seconds running below the minimum velocity will be considered stalled */
    public static final DoubleSupplier closeStallSeconds = Util.pref("RailSubsystem/CloseStallSeconds", 0.5);
}
