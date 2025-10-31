package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import static frc.robot.subsystems.intake.IntakeConfig.collectSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.ejectSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.feedSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.repositionSeconds;
import static frc.robot.subsystems.intake.IntakeConfig.repositionSpeed;
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

    // we use presets to define certain special speeds that correspond
    // to actions in the game (e.g. different phases of intake)
    public enum Preset {
        COLLECT,
        REPOSITION,
        FEED,
        EJECT
    }

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

    /** @return the speed associated with the specified preset */
    public double getPresetSpeed(Preset preset) {

        return switch (preset) {

            case COLLECT -> collectSpeed.getAsDouble();
            case FEED -> feedSpeed.getAsDouble();
            case REPOSITION -> repositionSpeed.getAsDouble();

            // when we have a configuration property that specifies a "negative
            // speed", we want to be careful that we always treat it as negative,
            // even if someone forgot to add a minus sign
            case EJECT -> -Math.abs(ejectSpeed.getAsDouble());
        };
    }

    /**
     * @return a command that will "release" the wheel by applying
     * zero voltage (this will probably be the default command for
     * this subsystem)
     */
    public Command idleCommand() {

        return new Command() {

            // before running this command we will reset the PID to
            // clear the setpoint and error calculations
            @Override
            public void initialize() {
                Util.log("[intake] idling");
                pid.reset();
            }

            // to execute we just set the voltage to 0
            @Override
            public void execute() {
                currentCommand = "idle";
                targetRps = Double.NaN;
                lastFeedforward = Double.NaN;
                lastFeedback = Double.NaN;
                lastVolts = 0.0;
                motor.applyVolts(lastVolts);
            }
        };
    }

    /**
     * @return a command that will run the shooter in closed-loop mode
     * at the supplied wheel velocity
     */
    public Command rpsCommand(String command, double rps) {

        return new Command() {

            // before running this command we will reset the PID with
            // the most recent configuration values, and clear its
            // calculated error from previous runs
            @Override
            public void initialize() {

                // logging commands when they initialize is a good way to
                // keep track of what the robot "thinks" it's doing for
                // debugging
                Util.log("[intake] running %s%n", command);
                pid.reset();
            }

            // to execute we just calculate feedforward and feedback
            // voltage
            @Override
            public void execute() {
                currentCommand = command;
                targetRps = rps;
                lastFeedforward = v.getAsDouble() * targetRps;
                lastFeedback = pid.calculate(getWheelVelocity(), targetRps);
                lastVolts = Util.clampVolts(lastFeedforward + lastFeedback);
                motor.applyVolts(lastVolts);
            }
        };
    }

    /**
     * @return a command that will run the intake in closed-loop mode
     * at the supplied linear velocity
     */
    public Command fpsCommand(String command, double fps) {

        // we can reuse commands with different inputs to create a
        // variety of different behaviors
        return rpsCommand(command, fps / wheelCircumferenceFeet);
    }

    /**
     * @return a command to run the intake at the specified preset speed
     */
    public Command presetCommand(Preset preset) {

        // note that this "captures" the configured preset speed when
        // the command is created - if you bind this directly to a joystick
        // button, it will ignore any later changes in configuration.
        // see below for examples of using a proxy command to avoid this.
        return fpsCommand(preset.name(), getPresetSpeed(preset));
    }

    /**
     * @return a command to allow tuning the shooter by setting the target
     * velocity from the dashboard
     */
    public Command tuningCommand() {

        // this allows setting velocity from the dashboard
        DoubleSupplier pref = Util.pref("IntakeSubsystem/TuningVelocity", 0.0);

        // when a command depends on a value that might change in between
        // executions, we use a proxy to make sure we get the most recent
        // value whenever it executes
        return Commands.deferredProxy(() -> rpsCommand("tuning", pref.getAsDouble()));
    }

    /**
     * @return a command that will run the intake backwards at eject speed
     * for a configured number of seconds
     */
    public Command ejectCommand() {

        // same principle as above - use proxy commands to make sure we
        // get the most recent timeout values
        return Commands.deferredProxy(() ->
            presetCommand(Preset.EJECT).withTimeout(ejectSeconds.getAsDouble()));
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

            Command collectUntilSensor = presetCommand(Preset.COLLECT)
                    .until(sensor);

            Command repositionUntilTimeout = presetCommand(Preset.REPOSITION)
                    .withTimeout(repositionSeconds.getAsDouble());

            return collectUntilSensor.andThen(repositionUntilTimeout);
        });
    }
}
