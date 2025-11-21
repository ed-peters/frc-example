package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.Consumer;

/**
 * Implementation of the {@link Motor} interface for a Rev motor
 * attached to a SparkMax motor.
 */
public class SparkMotor implements Motor {

    final SparkMax motor;
    final RelativeEncoder relativeEncoder;
    final AbsoluteEncoder absoluteEncoder;
    boolean brakeEnabled;

    public SparkMotor(int channelId,
                      boolean useAbsoluteEncoder,
                      Consumer<SparkMaxConfig> configurator) {

        // we use brushless motors and all of them have relative encoders
        this.motor = new SparkMax(channelId, MotorType.kBrushless);
        this.relativeEncoder = motor.getEncoder();

        // in some cases we want to use an external absolute encoder
        // which will measure a mechanism position rather than the
        // motor's position
        if (useAbsoluteEncoder) {
            this.absoluteEncoder = motor.getAbsoluteEncoder();
        } else {
            this.absoluteEncoder = null;
        }

        configure(configurator);
    }

    /** @return is the motor brake enabled? */
    @Override
    public boolean isBrakeEnabled() {
        return brakeEnabled;
    }

    /** @return the position of the motor */
    @Override
    public double getPosition() {
        return absoluteEncoder != null
                ? absoluteEncoder.getPosition()
                : relativeEncoder.getPosition();
    }

    /** @return the velocity of the motor */
    @Override
    public double getVelocity() {
        return absoluteEncoder != null
                ? absoluteEncoder.getPosition()
                : relativeEncoder.getPosition();
    }

    /** @return output current */
    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    /** Applies voltage */
    @Override
    public void applyVolts(double volts) {
        motor.setVoltage(volts);
    }

    /** Enables/disables motor brake */
    @Override
    public void applyBrake(boolean enable) {
        configure(config -> {
            config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        });
        brakeEnabled = enable;
    }

    /*
     * Creates and applies a new configuration to the motor
     */
    protected void configure(Consumer<SparkMaxConfig> configurator) {
        SparkMaxConfig config = new SparkMaxConfig();
        configurator.accept(config);
        motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
}
