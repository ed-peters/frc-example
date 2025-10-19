package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

/**
 * Customization of a PIDController that:
 * <ul>
 *
 *     <li>Only applies P and D controls;</li>
 *
 *     <li>Includes an optional "max feedback" property that caps
 *     how much feedback is returned; and,</li>
 *
 *     <li>Accesses P and D constants and tolerance through the
 *     {@link DoubleSupplier} interface, and picks up the latest
 *     values whenever it is reset;</li>
 *
 * </ul>
 */
public class PDController extends PIDController {

    final DoubleSupplier p;
    final DoubleSupplier d;
    final DoubleSupplier maxFeedback;
    final DoubleSupplier tolerance;

    public PDController(DoubleSupplier p,
                        DoubleSupplier d,
                        DoubleSupplier tolerance) {
        this(p, d, null, tolerance);
    }

    public PDController(DoubleSupplier p,
                        DoubleSupplier d,
                        DoubleSupplier maxFeedback,
                        DoubleSupplier tolerance) {
        super(p.getAsDouble(), 0.0, d.getAsDouble());
        this.p = p;
        this.d = d;
        this.maxFeedback = maxFeedback;
        this.tolerance = tolerance;
    }

    /** Resets PID error calculations and rereads configuration properties */
    public void reset() {
        setP(p.getAsDouble());
        setD(d.getAsDouble());
        setTolerance(tolerance.getAsDouble());
        super.reset();
    }

    /** @return feedback calculation */
    public double calculate(double measurement) {
        double value = super.calculate(measurement);
        if (maxFeedback != null) {
            value = Util.applyClamp(value, maxFeedback);
        }
        return value;
    }
}
