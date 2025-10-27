package frc.example.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.example.Util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.example.swerve.SwerveTeleopConfig.applySlew;
import static frc.example.swerve.SwerveTeleopConfig.applySniperToRotate;
import static frc.example.swerve.SwerveTeleopConfig.deadband;
import static frc.example.swerve.SwerveTeleopConfig.exponent;
import static frc.example.swerve.SwerveTeleopConfig.maxRotate;
import static frc.example.swerve.SwerveTeleopConfig.maxTranslate;
import static frc.example.swerve.SwerveTeleopConfig.slewRate;
import static frc.example.swerve.SwerveTeleopConfig.sniperFactor;
import static frc.example.swerve.SwerveTeleopConfig.turboFactor;

/**
 * Supplier for ChassisSpeeds that processes stick input and provides
 * several useful options:
 * <ul>
 *
 *     <li>Max translate/rotate speed (to convert 0.0 - 1.0 values to
 *     a valid field speed)</li>
 *
 *     <li>Deadband (highly recommended for joysticks that don't
 *     report 0.0 values when centered)</li>
 *
 *     <li>Exponent (lore has it that squaring or cubing small
 *     input values gives better control)</li>
 *
 *     <li>"Turbo" and "Sniper" Factors for translation (allows you to
 *     implement stuff like a simple "double my top speed" control)</li>
 *
 *     <li>One-sided slew rate limiting for translation (i.e.
 *     don't accelerate from 0 instantaneously, but "snap back"
 *     to 0 if they release the joystick)</li>
 *
 * </ul>
 *
 * For all configuration properties, translation units in feet and rotation
 * units are degrees. The resulting output is in meters and radians, in
 * accordance with prophecy.
 */
public class SwerveTeleopSpeedSupplier implements Supplier<ChassisSpeeds> {

    public enum Mode {
        TURBO,
        SNIPER,
        NONE
    }

    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier omega;
    BooleanSupplier turboTrigger;
    BooleanSupplier sniperTrigger;
    SlewRateLimiter limiterX;
    SlewRateLimiter limiterY;
    double inX;
    double inY;
    double inO;
    double lastX;
    double lastY;
    double lastOmega;

    public SwerveTeleopSpeedSupplier(DoubleSupplier x,
                                     DoubleSupplier y,
                                     DoubleSupplier omega,
                                     BooleanSupplier turboTrigger,
                                     BooleanSupplier sniperTrigger) {
        this.x = x;
        this.y = y;
        this.omega = omega;
        this.turboTrigger = turboTrigger;
        this.sniperTrigger = sniperTrigger;
        this.lastX = Double.NaN;
        this.lastY = Double.NaN;
        this.lastOmega = Double.NaN;

        SmartDashboard.putData("TurboSniperSpeedSupplier", builder -> {
            builder.addStringProperty("Mode", () -> getMode().toString(), null);
            builder.addDoubleProperty("InputX", () -> inX, null);
            builder.addDoubleProperty("InputY", () -> inY, null);
            builder.addDoubleProperty("InputOmega", () -> inO, null);
            builder.addDoubleProperty("SpeedX", () -> lastX, null);
            builder.addDoubleProperty("SpeedY", () -> lastX, null);
            builder.addDoubleProperty("SpeedOmega", () -> lastX, null);
        });
    }

    private Mode getMode() {
        if (sniperTrigger.getAsBoolean()) {
            return Mode.SNIPER;
        } else if (turboTrigger.getAsBoolean()) {
            return Mode.TURBO;
        } else {
            return Mode.NONE;
        }
    }

    @Override
    public ChassisSpeeds get() {

        inX = x.getAsDouble();
        inY = y.getAsDouble();
        inO = omega.getAsDouble();

        // get conditioned values for x, y, omega
        lastX = conditionInput(inX);
        lastY = conditionInput(inY);
        lastOmega = conditionInput(inO);

        // ensure that the point defined by (x, y) lies on the unit
        // circle - when we scale them by the maximum translate speed
        // this will prevent us from shooting off too fast at an angle
        double d = Math.hypot(lastX, lastY);
        if (d > 1.0) {
            lastX /= d;
            lastY /= d;
        }

        // convert to speeds
        double mt = maxTranslate.getAsDouble();
        lastX *= mt;
        lastY *= mt;
        lastOmega *= maxRotate.getAsDouble();

        // update slew settings and apply slew rate limiting if necessary
        checkSlew();
        if (limiterX != null) {
            lastX = slewLimit(lastX, limiterX);
        }
        if (limiterY != null) {
            lastY = slewLimit(lastY, limiterY);
        }

        switch (getMode()) {

            // if we're in sniper mode, we might want to slow down the rotation too
            case SNIPER:
                double sf = sniperFactor.getAsDouble();
                lastX *= sf;
                lastY *= sf;
                if (applySniperToRotate.getAsBoolean()) {
                    lastOmega *= sf;
                }
                break;

            // if we're in turbo mode, we only change translation
            case TURBO:
                double tf = turboFactor.getAsDouble();
                lastX *= tf;
                lastY *= tf;
                break;

        }

        return new ChassisSpeeds(
                Units.feetToMeters(lastX),
                Units.feetToMeters(lastY),
                Math.toRadians(lastOmega));
    }

    // make sure that we're either applying or not applying slew rate limiting
    // depending on the configuration property
    private void checkSlew() {
        if (applySlew.getAsBoolean()) {
            if (limiterX == null) {
                double slew = slewRate.getAsDouble();
                limiterX = new SlewRateLimiter(slew);
                limiterY = new SlewRateLimiter(slew);
                Util.log("[SwerveSpeedSupplier] enabling slew rate limiting @ %.2f", slew);
            }
        } else if (limiterX != null) {
            limiterX = null;
            limiterY = null;
            Util.log("[SwerveSpeedSupplier] disabling slew rate limiting");
        }
    }

    private double slewLimit(double value, SlewRateLimiter limiter) {
        if (value == 0.0) {
            limiter.reset(0.0);
        } else {
            value = limiter.calculate(value);
        }
        return value;
    }

    private double conditionInput(double input) {
        input = MathUtil.clamp(input, -1.0, 1.0);
        input = MathUtil.applyDeadband(input, deadband.getAsDouble());
        input = Math.copySign(Math.pow(input, exponent.getAsDouble()), input);
        return input;
    }
}
