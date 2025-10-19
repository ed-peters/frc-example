package frc.robot.util;

import java.util.function.DoubleSupplier;

/**
 * Implements feedforward calculations using {@link DoubleSupplier}
 * instances to fetch constants
 */
public class Feedforward {

    final DoubleSupplier g;
    final DoubleSupplier v;

    public Feedforward(DoubleSupplier v) {
        this(null, v);
    }

    public Feedforward(DoubleSupplier g, DoubleSupplier v) {
        this.g = g;
        this.v = v;
    }

    /** @return feedforward for an arm */
    public double arm(double position, double velocity) {
        return g.getAsDouble() * Math.cos(Math.toRadians(position))
                + v.getAsDouble() * velocity;
    }

    /** @return feedforward for an elevator */
    public double elevator(double velocity) {
        return g.getAsDouble() + v.getAsDouble() * velocity;
    }

    /** @return feedforward for a flywheel */
    public double wheel(double velocity) {
        return v.getAsDouble() * velocity;
    }
}
