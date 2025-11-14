package frc.robot.subsystems.auto;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

public class AutonomousConfig {

    public static final DoubleSupplier p = pref("Autonomous/kP", 1.0);
    public static final DoubleSupplier d = pref("Autonomous/kD", 0.0);

}
