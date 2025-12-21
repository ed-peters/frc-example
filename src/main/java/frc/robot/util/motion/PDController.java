package frc.robot.util.motion;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

/**
 * Adds the following useful functionality to a normal {@link PIDController}:
 * <ul>
 *
 *     <li>It doesn't use I - we generally stay away from it because of
 *     <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/common-control-issues.html#integral-term-windup">integral
 *     windup</a></li>
 *
 *     <li>Parameters are received from {@link DoubleSupplier} instances,
 *     so they can be managed via {@link edu.wpi.first.wpilibj.Preferences}
 *     </li>
 *
 *     <li>Includes the option for a "max feedback" limit, to avoid a
 *     "mousetrap" effect when e.g. a remembered setpoint is too far away from
 *     the measurement (we encountered on an elevator which would settle to the
 *     bottom slowly when disabled, and then go crazy when re-enabled)</li>
 * </ul>
 */
public class PDController extends PIDController {

    /**
     * Holder for configuration parameters
     * @param p supplier for the p parameter
     * @param d supplier for the d parameter
     * @param tolerance supplier for the tolerance parameter
     * @param maxFeedback supplier for the feedback limit
     */
    public record Parameters(DoubleSupplier p,
                             DoubleSupplier d,
                             DoubleSupplier tolerance,
                             DoubleSupplier maxFeedback) {

    }

    final DoubleSupplier p;
    final DoubleSupplier d;
    final DoubleSupplier tolerance;
    final DoubleSupplier maxFeedback;

    /**
     * Creates a {@link PDController} with the supplied parameters
     * @param params parameters for the controller
     */
    public PDController(Parameters params) {
        this(params.p(), params.d(), params.tolerance(), params.maxFeedback());
    }

    /**
     * Creates a {@link PIDController} with no tolerance and no max feedback
     * @param p supplier for the p parameter
     * @param d supplier for the d parameter
     * @param tolerance supplier for the tolerance parameter
     * @param maxFeedback supplier for the feedback limit
     */
    public PDController(DoubleSupplier p,
                        DoubleSupplier d,
                        DoubleSupplier tolerance,
                        DoubleSupplier maxFeedback) {
        super(p.getAsDouble(), 0.0, d.getAsDouble());
        this.p = p;
        this.d = d;
        this.tolerance = tolerance;
        this.maxFeedback = maxFeedback;
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
    }

    /**
     * Resets all parameters from their associated {@link DoubleSupplier} and
     * accumulated error
     */
    @Override
    public void reset() {
        setP(p.getAsDouble());
        setD(d.getAsDouble());
        if (tolerance != null) {
            setTolerance(tolerance.getAsDouble());
        }
        super.reset();
    }

    /**
     * @param measurement the measured value
     * @return calculated feedback (if there is a feedback limit, the return
     * value is clamped)
     */
    @Override
    public double calculate(double measurement) {
        double fb = super.calculate(measurement);
        if (maxFeedback != null) {
            fb = Util.applyClamp(fb, maxFeedback);
        }
        return fb;
    }
}
