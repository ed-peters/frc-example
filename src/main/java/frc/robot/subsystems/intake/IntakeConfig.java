package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

public class IntakeConfig {

    // Physical construction properties
    public static final double maxVolts = 12.0;
    public static final double gearRatio = 1.0;
    public static final double wheelCircumferenceInches = 12.56;

    // ratio of motor velocity to linear speed of wheel contact point
    public static final double velocityFactor = gearRatio * (wheelCircumferenceInches / 12.0) * Math.PI;

    // Feedforward and feedback properties
    public static final DoubleSupplier v = pref("intake_v", 0.1);
    public static final DoubleSupplier p = pref("intake_p", 0.1);
    public static final DoubleSupplier d = pref("intake_d", 0.1);

    // Tolerance in rotations per second
    public static final DoubleSupplier velocityTolerance = pref("intake_velocity_tolerance", 2.0);

    // Preset speeds and times
    public static final DoubleSupplier pickupSpeed = pref("intake_pickup_speed", 3.0);
    public static final DoubleSupplier feedSpeed = pref("intake_feed_speed", 1.0);
    public static final DoubleSupplier ejectSpeed = pref("intake_eject_speed", 2.0);
    public static final DoubleSupplier ejectSeconds = pref("intake_eject_secs", 2.0);
}
