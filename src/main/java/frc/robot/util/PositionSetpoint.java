package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import java.util.function.DoubleSupplier;

/**
 * Subclass of {@link State} that includes logic for "clamping" a
 * position based on min/max position and max velocity (accessed
 * via {@link DoubleSupplier} instances)
 */
public class PositionSetpoint extends State {

    final DoubleSupplier minPosition;
    final DoubleSupplier maxPosition;
    final DoubleSupplier maxVelocity;

    public PositionSetpoint(DoubleSupplier minPosition,
                            DoubleSupplier maxPosition,
                            DoubleSupplier maxVelocity) {
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.maxVelocity = maxVelocity;
        this.position = Double.NaN;
        this.velocity = Double.NaN;
    }

    /** @return do we have a setpoint? */
    public boolean hasSetpoint() {
        return Double.isFinite(position);
    }

    /** Sets position and velocity to NaN */
    public void clear() {
        position = Double.NaN;
        velocity = Double.NaN;
    }

    /** Set position and velocity to target values, applying clamping logic */
    public void set(double position, double velocity) {

        // you can't be beyond the limits
        double minP = minPosition.getAsDouble();
        double maxP = maxPosition.getAsDouble();
        this.position = MathUtil.clamp(position, minP, maxP);

        // you can't go faster than the max velocity
        double maxV = maxVelocity.getAsDouble();
        this.velocity = MathUtil.clamp(velocity, -maxV, maxV);

        // you can't move down if you're at the bottom
        if (this.position == minP && this.velocity < 0.0) {
            this.velocity = 0.0;
        }

        // you can't move up if you're at the top
        if (this.position == maxP && this.velocity > 0.0) {
            this.velocity = 0.0;
        }
    }
}
