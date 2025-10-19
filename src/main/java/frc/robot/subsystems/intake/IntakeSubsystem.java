package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Dash;
import frc.robot.util.PDController;
import frc.robot.util.Feedforward;
import frc.robot.util.Motor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConfig.*;

/**
 * Simple subsystem for a spinning flywheel with closed loop control
 * based on the linear velocity of the wheel's outer edge in feet per
 * second
 */
public class IntakeSubsystem extends SubsystemBase {

    final Motor motor;
    final BooleanSupplier sensor;
    final Feedforward feedforward;
    final PDController pid;
    String currentCommand;
    double targetVelocity;
    double lastFeedforward;
    double lastFeedback;
    double lastVolts;

    public IntakeSubsystem(Motor motor, BooleanSupplier sensor) {
        this.motor = motor;
        this.sensor = sensor;
        this.feedforward = new Feedforward(v);
        this.pid = new PDController(p, d, velocityTolerance);
        this.currentCommand = "";
        this.targetVelocity = Double.NaN;
        this.lastFeedforward = Double.NaN;
        this.lastFeedback = Double.NaN;
        this.lastVolts = Double.NaN;
        motor.applyBrakes(false);
    }

    /** @return linear velocity in feet per second */
    public double getLinearVelocity() {
        return motor.getVelocity() * velocityFactor;
    }

    /** @return target wheel velocity in rotations per second */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** @return wheel velocity error */
    public double getVelocityError() {
        return getTargetVelocity() - getLinearVelocity();
    }

    /** @return do we have a gamepiece? */
    public boolean hasGamepiece() {
        return sensor.getAsBoolean();
    }

    /** @return are we in closed loop and  "close enough" to target speed? */
    public boolean isAtSetpoint() {
        return Double.isFinite(targetVelocity) && pid.atSetpoint();
    }

    /**
     * Operate in open loop mode, applying a fixed voltage
     */
    public void openLoop(String command, double volts) {
        currentCommand = command;
        targetVelocity = Double.NaN;
        lastFeedforward = Double.NaN;
        lastFeedback = Double.NaN;
        lastVolts = MathUtil.clamp(volts, -maxVolts, maxVolts);
        motor.applyVolts(lastVolts);
    }

    /**
     * Operate in closed loop mode to attain a certain wheel velocity
     * in rotations per second
     */
    public void closedLoop(String command, double wheelVelocity) {
        currentCommand = command;
        targetVelocity = wheelVelocity;
        lastFeedforward = feedforward.wheel(targetVelocity);
        lastFeedback = pid.calculate(getLinearVelocity(), targetVelocity);
        lastVolts = MathUtil.clamp(lastFeedforward + lastFeedback, -maxVolts, maxVolts);
        motor.applyVolts(lastVolts);
    }

    /**
     * Publish state to dashboard
     */
    public void periodic() {
        Dash.publish("IntakeSubsystem/MotorVelocity", motor.getVelocity());
        Dash.publish("IntakeSubsystem/LastFeedback", lastFeedback);
        Dash.publish("IntakeSubsystem/LastFeedforward", lastFeedforward);
        Dash.publish("IntakeSubsystem/LastVolts", lastVolts);
        Dash.publish("IntakeSubsystem/Sensor", sensor.getAsBoolean());
        Dash.publish("IntakeSubsystem/VelocityCurrent", getLinearVelocity());
        Dash.publish("IntakeSubsystem/VelocityError", getVelocityError());
        Dash.publish("IntakeSubsystem/VelocityTarget", getTargetVelocity());
    }

    /** @return a command to achieve a specified linear velocity */
    public Command fixedSpeedCommand(IntakePreset preset) {
        return run(() -> closedLoop(preset.name(), preset.speed.getAsDouble()));
    }

    /** @return a command to achieve a specified linear velocity */
    public Command fixedSpeedCommand(String name, DoubleSupplier speedSupplier) {
        return run(() -> closedLoop(name, speedSupplier.getAsDouble()));
    }

    /** @return a command allowing us to tune the subsystem */
    public Command tuningCommand() {
        String property = "IntakeTuningCommand/TuningVelocity";
        SmartDashboard.putNumber(property, 1.0);
        return run(() -> {
            pid.reset();
            closedLoop("tuning", SmartDashboard.getNumber(property, 5.0));
        });
    }

    /** @return a command to stop the subsystem by applying 0 volts */
    public Command stopCommand() {
        return run(() -> openLoop("stop", 0.0));
    }

    /** @return a command to run at pickup speed until we have a gamepiece */
    public Command pickupCommand() {
        return fixedSpeedCommand(IntakePreset.PICKUP).until(this::hasGamepiece);
    }

    /** @return a command to run backwards at eject speed for a few seconds */
    public Command ejectCommand() {
        return run(() -> closedLoop("eject", -ejectSpeed.getAsDouble()))
                .withTimeout(ejectSeconds.getAsDouble());
    }
}
