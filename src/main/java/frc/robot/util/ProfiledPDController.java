package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

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
 *     <li>Special constructors for translation and rotation that will
 *     convert top speed and acceleration based on the appropriate units
 *     (degrees, feet) to the ones used internally (radians, meters)</li>
 *
 * </ul>
 */
public class ProfiledPDController extends ProfiledPIDController {

    interface UnitTranslator {
        double apply(double value);
    }

    final DoubleSupplier p;
    final DoubleSupplier d;
    final DoubleSupplier maxTranslation;
    final DoubleSupplier acceleration;
    final UnitTranslator translator;

    private ProfiledPDController(DoubleSupplier p,
                                 DoubleSupplier d,
                                 DoubleSupplier maxTranslation,
                                 DoubleSupplier acceleration,
                                 UnitTranslator translator) {
        super(p.getAsDouble(), 0.0, d.getAsDouble(), makeConstraints(maxTranslation, acceleration, translator));
        this.p = p;
        this.d = d;
        this.maxTranslation = maxTranslation;
        this.acceleration = acceleration;
        this.translator = translator;
    }

    /**
     * Resets PID error calculations and re-reads the latest configuration
     * values
     */
    public void reset() {
        setP(p.getAsDouble());
        setD(d.getAsDouble());
        setConstraints(makeConstraints(maxTranslation, acceleration, translator));
        super.reset(Util.ZERO_STATE);
    }

    /**
     * @return a new controller for rotation (max velocity will be converted
     * to radians, and output will wrap around at (-PI, PI)
     */
    public static ProfiledPDController rotationController(DoubleSupplier p,
                                                          DoubleSupplier d,
                                                          DoubleSupplier maxVelocity,
                                                          DoubleSupplier acceleration) {
        ProfiledPDController controller = new ProfiledPDController(p, d, maxVelocity, acceleration, Math::toRadians);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }

    /**
     * @return a new controller for translation (max velocity will be
     * converted to meters)
     */
    public static ProfiledPDController translationController(DoubleSupplier p,
                                                             DoubleSupplier d,
                                                             DoubleSupplier maxVelocity,
                                                             DoubleSupplier acceleration) {
        return new ProfiledPDController(p, d, maxVelocity, acceleration, Units::feetToMeters);
    }

    /**
     * @return new constraints
     */
    private static Constraints makeConstraints(DoubleSupplier maxTranslation,
                                               DoubleSupplier acceleration,
                                               UnitTranslator translator) {
        double maxR = translator.apply(maxTranslation.getAsDouble());
        double maxA = maxR * acceleration.getAsDouble();
        return new Constraints(maxR, maxA);
    }
}
