package frc.robot.subsystems.strap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Motor;
import frc.robot.util.Util;

/**
 * Implements the {@link Motor} interface along with a simple flag to
 * trigger a "stall" condition.
 */
public class StrapMotorSim implements Motor {

    /** Top speed in RPS */
    public static final double FREE_VELOCITY = 50.0;

    /** Amperage when running freely or stalled */
    public static final double FREE_AMPS = 1.0;
    public static final double STALL_AMPS = 10.0;

    double position;
    double velocity;
    double amps;
    boolean stalled;

    public StrapMotorSim() {
        position = 0.0;
        velocity = 0.0;
        amps = 0.0;
        stalled = false;

        SmartDashboard.putData("StrapMotorSim", builder -> {
            builder.addBooleanProperty("Stalled?", () -> stalled, val -> stalled = val);
        });
    }

    @Override
    public void applyBrake(boolean brake) {
        Util.log("[strap-sim] no support for brakes");
    }

    @Override
    public boolean isBrakeEnabled() {
        return false;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public double getCurrent() {
        return 1.0;
    }

    @Override
    public void applyVolts(double volts) {

        // we can always apply 0 volts to get 0 velocity
        if (volts == 0.0) {
            velocity = 0.0;
            amps = 0.0;
        }

        // if voltage is positive when stalled, we don't move and current goes up
        else if (stalled && volts > 0.0) {
            velocity = 0.0;
            amps = STALL_AMPS;
        }

        // otherwise we move based on proportion of max voltage
        else {
            velocity = FREE_VELOCITY * (volts / Util.MAX_VOLTS);
            position += velocity * Util.DT;
            amps = FREE_AMPS;
        }
    }
}
