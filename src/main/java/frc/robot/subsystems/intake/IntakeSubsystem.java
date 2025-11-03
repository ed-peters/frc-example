package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.IntakeIdleCommand;
import frc.robot.commands.intake.IntakeRpsCommand;
import frc.robot.util.Motor;
import frc.robot.util.PDController;
import frc.robot.util.Util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConfig.d;
import static frc.robot.subsystems.intake.IntakeConfig.defaultBrakeEnabled;
import static frc.robot.subsystems.intake.IntakeConfig.ejectSeconds;
import static frc.robot.subsystems.intake.IntakeConfig.gearRatio;
import static frc.robot.subsystems.intake.IntakeConfig.p;
import static frc.robot.subsystems.intake.IntakeConfig.repositionSeconds;
import static frc.robot.subsystems.intake.IntakeConfig.tolerance;
import static frc.robot.subsystems.intake.IntakeConfig.v;
import static frc.robot.subsystems.intake.IntakeConfig.wheelCircumferenceFeet;

/**
 * Example of an intake subsystem based on the 2025 Reefscape robot.
 * It shows how to:
 *
 * <ul>
 *
 *     <li>Combine simple feedforward with PID control to achieve a
 *     fixed target velocity</li>
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
 *
 * This could serve as the basis for any subsystem whose main component
 * is a spinning wheel (e.g. a shooter or indexer)
 */
public class IntakeSubsystem extends SubsystemBase {

    final Motor motor;
    final BooleanSupplier sensor;
    final PDController pid;

    // displaying what a subsystem "thinks" it's doing on the dashboard
    // is really helpful when you're debugging the robot
    String currentCommand;

    double targetRps;
    double lastFeedforward;
    double lastFeedback;
    double lastVolts;

    public IntakeSubsystem(Motor motor, BooleanSupplier sensor) {

        this.motor = motor;
        this.sensor = sensor;
        this.pid = new PDController(p, d, null, tolerance);
        this.currentCommand = "";
        this.targetRps = Double.NaN;

        motor.applyBrake(defaultBrakeEnabled);

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("CurrentCommand", () -> currentCommand, null);
            builder.addDoubleProperty("MotorAmps", motor::getCurrent, null);
            builder.addDoubleProperty("MotorVelocity", motor::getVelocity, null);
            builder.addDoubleProperty("WheelVelocity", this::getWheelVelocity, null);
            builder.addDoubleProperty("LinearVelocity", this::getLinearVelocity, null);
            builder.addBooleanProperty("AtSetpoint?", pid::atSetpoint, null);
            builder.addBooleanProperty("HasGamepiece?", sensor, null);
            builder.addBooleanProperty("Brake?", motor::isBrakeEnabled, motor::applyBrake);
        });
    }

    /** @return wheel velocity in rotations per second */
    public double getWheelVelocity() {
        return motor.getVelocity() * gearRatio;
    }

    /** @return linear velocity of the shooter wheel in feet per second */
    public double getLinearVelocity() {
        return getWheelVelocity() * wheelCircumferenceFeet;
    }

    /** Runs the subsystem in closed loop mode */
    public void closedLoop(String command, double rps) {

        // remember the current command and goal
        currentCommand = command;
        targetRps = rps;

        // calculate feedforward, feedback and total voltage
        lastFeedforward = v.getAsDouble() * targetRps;
        lastFeedback = pid.calculate(getWheelVelocity(), targetRps);
        lastVolts = Util.clampVolts(lastFeedforward + lastFeedback);

        motor.applyVolts(lastVolts);
    }

    /** Runs the system in open loop mode */
    public void openLoop(String command, double volts) {

        // remember the current command
        currentCommand = command;

        // in open loop mode, we have no target, feedback or feedforward
        targetRps = Double.NaN;
        lastFeedforward = Double.NaN;
        lastFeedback = Double.NaN;
        lastVolts = Util.clampVolts(volts);

        motor.applyVolts(volts);
    }

    /** @return a command that will "release" the wheel */
    public Command idleCommand() {
        return new IntakeIdleCommand(this);
    }

    /** @return a command that to run at the supplied wheel speed */
    public Command rpsCommand(String command, DoubleSupplier speedSupplier) {
        return new IntakeRpsCommand(this, command, speedSupplier);
    }

    /** @return a command that to run at the supplied linear velocity */
    public Command fpsCommand(String command, DoubleSupplier fps) {

        // we can reuse commands with different inputs to create a
        // variety of different behaviors
        return rpsCommand(command, () -> fps.getAsDouble() / wheelCircumferenceFeet);
    }

    /** @return a command to run the intake at the specified preset speed */
    public Command presetCommand(IntakePreset preset) {
        return fpsCommand(preset.name(), preset.getSpeed());
    }

    /**
     * @return a command to allow tuning the shooter by setting the target
     * velocity from the dashboard
     */
    public Command tuningCommand() {

        // being able to set the tuning speed from the dashboard is hella
        // useful during tuning
        SmartDashboard.putNumber("IntakeSubsystem/TuningVelocity", 0.0);
        return rpsCommand("tuning", () ->
            SmartDashboard.getNumber("IntakeSubsystem/TuningVelocity", 0.0));
    }

    /**
     * @return a command that will run the intake backwards at eject speed
     * for a configured number of seconds
     */
    public Command ejectCommand() {

        // when a command depends on a value that might change in between
        // executions, we use a proxy to make sure we get the most recent
        // value whenever it executes
        return Commands.deferredProxy(() ->
            presetCommand(IntakePreset.EJECT).withTimeout(ejectSeconds.getAsDouble()));
    }

    /**
     * @return a command that will run the intake forward at collect speed
     * until a piece has been picked up, then run at reposition speed for
     * the specified number of seconds
     */
    public Command collectCommand() {

        // in Reefscape, once we collected a gamepiece we had to jiggle
        // it around a little in the intake to get it to the right
        // position for scoring; this shows how to implement a sequence
        // of different speeds by using command composition, sensors
        // and timing
        return Commands.deferredProxy(() -> {

            Command collectUntilSensor = presetCommand(IntakePreset.COLLECT)
                    .until(sensor);

            Command repositionUntilTimeout = presetCommand(IntakePreset.REPOSITION)
                    .withTimeout(repositionSeconds.getAsDouble());

            return collectUntilSensor.andThen(repositionUntilTimeout);
        });
    }
}
