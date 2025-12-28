package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration properties for the sample elevator subsystem. Stuff that
 * will (probably) never change can simply be Java constants, but stuff
 * that we might tweak/tune should be in {@link edu.wpi.first.wpilibj.Preferences}
 */
public class ElevatorConfig {

    /**
     * How many inches the elevator travels per motor rotation; this is
     * used to calculate height and velocity from the motor
     */
    public static final double inchesPerRotation = 1.35;

    /**
     * Minimum and maximum height of elevator in inches; used to prevent
     * us from going to far
     */
    public static final DoubleSupplier minHeight = pref("ElevatorSubsystem/MinHeight", 1.0);
    public static final DoubleSupplier maxHeight = pref("ElevatorSubsystem/MaxHeight", 90.0);

    /**
     * Maximum velocity when moving between presets; used to prevent us
     * from going too fast
     */
    public static final DoubleSupplier maxVelocity = pref("ElevatorSubsystem/MaxVelocity", 120.0);

    /**
     * Maximum acceleration moving between presets; as an example when the
     * acceleration is twice the velocity, it will take 0.5s to reach top speed
     */
    public static final DoubleSupplier maxAcceleration = pref("ElevatorSubsystem/MaxAcceleration", 240.0);

    /**
     * Maximum voltage in teleop for moving elevator; see
     * {@link frc.robot.commands.elevator.ElevatorTuningCommand}
     */
    public static final DoubleSupplier maxTeleopVolts = pref("ElevatorSubsystem/TeleopVolts", 6.0);

    /**
     * Feedforward and feedback tuning constants; you'll determine values
     * for these using the {@link frc.robot.commands.elevator.ElevatorTuningCommand}
     */
    public static final DoubleSupplier p = pref("ElevatorSubsystem/kP", 3.0);
    public static final DoubleSupplier d = pref("ElevatorSubsystem/kD", 0.3);
    public static final DoubleSupplier g = pref("ElevatorSubsystem/kG", 1.3028);
    public static final DoubleSupplier v = pref("ElevatorSubsystem/kV", 0.0992);

    /**
     * Maximum feedback from feedback; you will probably want to keep this
     * to low single digits. If it's too high, you could wind up with a
     * huge power spike if the target height is too far away from the
     * current height.
     */
    public static final DoubleSupplier maxFeedback = pref("ElevatorSubsystem/MaxFeedback", 2.0);

    /**
     * Tolerance for how close (in inches) we have to be before we consider
     * ourselves to be "at" a goal height
     */
    public static final DoubleSupplier tolerance = pref("ElevatorSubsystem/Tolerance", 0.25);

    /**
     * Preset heights in inches, for things like scoring positions; see
     * {@link ElevatorPreset} for how these get used
     */
    public static final DoubleSupplier presetL1 = pref("ElevatorSubsystem/L1", 10.0);
    public static final DoubleSupplier presetL2 = pref("ElevatorSubsystem/L2", 40.0);
    public static final DoubleSupplier presetL3 = pref("ElevatorSubsystem/L3", 80.0);

}
