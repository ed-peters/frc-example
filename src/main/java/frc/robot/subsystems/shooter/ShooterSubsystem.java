package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.shooter.ShooterIdleCommand;
import frc.robot.commands.shooter.ShooterRpsCommand;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.shooter.ShooterConfig.defaultBrakeEnabled;
import static frc.robot.subsystems.shooter.ShooterConfig.gearRatio;
import static frc.robot.subsystems.shooter.ShooterConfig.maxAmps;
import static frc.robot.subsystems.shooter.ShooterConfig.p;
import static frc.robot.subsystems.shooter.ShooterConfig.rampRate;
import static frc.robot.subsystems.shooter.ShooterConfig.tolerance;
import static frc.robot.subsystems.shooter.ShooterConfig.v;
import static frc.robot.subsystems.shooter.ShooterConfig.wheelCircumferenceFeet;

/**
 * Example of an shooter subsystem based on the 2024 Crescendo robot.
 * It shows how to:
 *
 * <ul>
 *
 *     <li>Use hardware-based PID on a REV SparkMax, to take advantage
 *     of the very fast update rate for the built in encoder</li>
 *
 *     <li>Define special "preset" target speeds that can be controlled
 *     via configuration</li>
 *
 *     <li>Using proxies to allow commands to capture updates configuration
 *     values each time they run</li>
 *
 *     <li>Use command composition to create complex actions that
 *     combine sensors and timeouts</li>
 *
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

    final SparkMax motor;
    final RelativeEncoder encoder;
    final SparkClosedLoopController controller;
    double lastP;
    double lastV;
    String currentCommand;
    double targetWheelRps;

    public ShooterSubsystem(int motorChannel) {

        motor = new SparkMax(motorChannel, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        // the encoder's native units are rotations per minute; this will
        // change it to be wheel rotations per second
        config.encoder.velocityConversionFactor(gearRatio / 60.0);

        // if something gets jammed and a motor stalls, amperage will
        // spike and we could burn out a motor; setting a current limit
        // is a good safety practice
        config.smartCurrentLimit(maxAmps);

        // giving a little bit of ramp up time to the motor helps save
        // wear on the mechanism when we instantly jump to a high speed
        config.openLoopRampRate(rampRate);
        config.closedLoopRampRate(rampRate);

        config.idleMode(defaultBrakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        this.encoder = motor.getEncoder();
        this.controller = motor.getClosedLoopController();
        this.currentCommand = "";
        this.targetWheelRps = Double.NaN;

        // this will have the effect of configuring the PID controller
        // with default values
        this.lastP = Double.NaN;
        this.lastV = Double.NaN;
        resetPid();

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("CurrentCommand", () -> currentCommand, null);
            builder.addDoubleProperty("MotorAmps", motor::getOutputCurrent, null);
            builder.addDoubleProperty("MotorVolts", motor::getBusVoltage, null);
            builder.addDoubleProperty("WheelVelocity", encoder::getVelocity, null);
            builder.addDoubleProperty("LinearVelocity", this::getLinearVelocity, null);
            builder.addDoubleProperty("TargetWheelVelocity", () -> targetWheelRps, null);
            builder.addBooleanProperty("AtSetpoint?", this::atSetpoint, null);
        });
    }

    /** @return wheel velocity in rotations per second */
    public double getWheelVelocity() {
        return encoder.getVelocity();
    }

    /** @return linear velocity of the shooter wheel in feet per second */
    public double getLinearVelocity() {
        return getWheelVelocity() * wheelCircumferenceFeet;
    }

    /** @return are we close enough to our setpoint? */
    public boolean atSetpoint() {
        return MathUtil.isNear(targetWheelRps, getWheelVelocity(), tolerance.getAsDouble());
    }

    /**
     * Reset the PID controller; closed loop commands should call this when
     * they initialize
     */
    public void resetPid() {

        double thisP = p.getAsDouble();
        double thisV = v.getAsDouble();

        // configuration happens via CAN bus messages; to avoid spamming
        // the CAN bus we'll only change the configuration if someone has
        // changed property values in the dashboard
        if (thisP != lastP || thisV != lastV) {

            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.p(p.getAsDouble());
            config.closedLoop.velocityFF(v.getAsDouble());

            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    /**
     * Runs the subsystem in closed loop mode, targeting a specific wheel
     * velocity in rotations per second
     */
    public void closedLoop(String command, double wheelRps) {

        // remember the current command and goal
        currentCommand = command;
        targetWheelRps = wheelRps;

        // the motor controller handles feedback and feedforward calculations
        controller.setReference(wheelRps, ControlType.kVelocity);
    }

    /** Runs the system in open loop mode */
    public void openLoop(String command, double volts) {

        // remember the current command
        currentCommand = command;

        // in open loop mode, we have no target, feedback or feedforward
        targetWheelRps = Double.NaN;
        motor.setVoltage(volts);
    }

    /** @return a command that will "release" the wheel */
    public Command idleCommand() {
        return new ShooterIdleCommand(this);
    }

    /** @return a command that to run at the supplied wheel speed */
    public Command rpsCommand(String command, DoubleSupplier speedSupplier) {
        return new ShooterRpsCommand(this, command, speedSupplier);
    }

    /** @return a command that to run at the supplied linear velocity */
    public Command fpsCommand(String command, DoubleSupplier fps) {

        // we can reuse commands with different inputs to create a
        // variety of different behaviors
        return rpsCommand(command, () -> fps.getAsDouble() / wheelCircumferenceFeet);
    }

    /** @return a command to run the shooter at the specified preset speed */
    public Command presetCommand(ShooterPreset preset) {
        return fpsCommand(preset.name(), preset.getSpeed());
    }

    /**
     * @return a command to allow tuning the shooter by setting the target
     * velocity from the dashboard
     */
    public Command tuningCommand() {

        // being able to set the tuning speed from the dashboard is hella
        // useful during tuning
        SmartDashboard.putNumber("ShooterSubsystem/TuningVelocity", 0.0);
        return rpsCommand("tuning", () ->
            SmartDashboard.getNumber("ShooterSubsystem/TuningVelocity", 0.0));
    }
}
