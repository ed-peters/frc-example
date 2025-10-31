package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

/**
 * Subclass of {@link PIDController} that provides a couple of useful
 * behaviors:
 *
 * <ul>
 *
 *     <li>All configuration properties are sourced from {@link DoubleSupplier}
 *     instances, so they can be controlled from preferences; calling
 *     {@link #reset()} will re-read configuration as well as resetting
 *     error calculations</li>
 *
 *     <li>Adds an optional "maximum feedback" cap, to help avoid
 *     overcorrecting when the setpoint is too far away from the current
 *     value</li>
 *
 * </ul>
 */
public class PDController extends PIDController {

    final DoubleSupplier p;
    final DoubleSupplier d;
    final DoubleSupplier maxFeedback;
    final DoubleSupplier tolerance;
    double maxF;

    public PDController(DoubleSupplier p,
                        DoubleSupplier d,
                        DoubleSupplier maxFeedback,
                        DoubleSupplier tolerance) {
        super(p.getAsDouble(), 0.0, d.getAsDouble());
        this.p = p;
        this.d = d;
        this.maxFeedback = maxFeedback;
        this.tolerance = tolerance;
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
    }

    /**
     * Resets error calculations and values from configuration
     */
    @Override
    public void reset() {
        setP(p.getAsDouble());
        setD(d.getAsDouble());
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
        if (maxFeedback != null) {
            maxF = maxFeedback.getAsDouble();
        } else {
            maxF = Double.NaN;
        }
        super.reset();
    }

    /**
     * @return the calculated feedback (capped to a maximum if one
     * was supplied)
     */
    @Override
    public double calculate(double measurement, double setpoint) {
        double value = super.calculate(measurement, setpoint);
        if (Double.isFinite(maxF)) {
            return MathUtil.clamp(value, -maxF, maxF);
        }
        return value;
    }

    /**
     * @return is the supplied measurement close enough to the
     * supplied setpoint (based on configured error tolerance)
     */
    public boolean at(double measurement, double setpoint) {
        return Math.abs(measurement - setpoint) < getErrorTolerance();
    }
}
