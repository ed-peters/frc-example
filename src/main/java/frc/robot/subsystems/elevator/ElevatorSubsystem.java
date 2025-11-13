package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.ElevatorHoldCommand;
import frc.robot.commands.elevator.ElevatorTeleopCommand;
import frc.robot.commands.elevator.ElevatorTrapezoidCommand;
import frc.robot.commands.elevator.ElevatorTuningCommand;
import frc.robot.util.Motor;
import frc.robot.util.PDController;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.d;
import static frc.robot.subsystems.elevator.ElevatorConfig.g;
import static frc.robot.subsystems.elevator.ElevatorConfig.inchesPerRotation;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxFeedback;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxHeight;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxVelocity;
import static frc.robot.subsystems.elevator.ElevatorConfig.minHeight;
import static frc.robot.subsystems.elevator.ElevatorConfig.p;
import static frc.robot.subsystems.elevator.ElevatorConfig.tolerance;
import static frc.robot.subsystems.elevator.ElevatorConfig.v;

/**
 * Example of an elevator subsystem based on the 2025 Reefscape robot.
 * It shows how to:
 *
 * <ul>
 *
 *     <li>Combine simple feedforward, PID control and trapezoid motion
 *     profiles to achieve a fixed target position with smooth-but-fast
 *     motion</li>
 *
 *     <li>Use a "goal" position to reflect moving under closed-loop
 *     control while maintaining a long-term target destination</li>
 *
 *     <li>Define special "preset" target speeds that can be controlled
 *     via configuration</li>
 *
 *     <li>Use command composition to create complex actions that
 *     combine sensors and timeouts</li>
 *
 * </ul>
 *
 * This could serve as the basis for any subsystem whose main component
 * is a spinning wheel (e.g. a shooter or indexer)
 */
public class ElevatorSubsystem extends SubsystemBase {

    final Motor motor;
    final PDController pid;
    String currentCommand;

    // we'll keep these updated during periodic mode
    double currentHeight;
    double currentVelocity;

    // we keep track of a "next" height and velocity, which is what we're
    // aiming for in the current scheduler cycle, but we also have a "goal"
    // with is the long term target if we're doing e.g. a trapezoid motion
    // profile (see the ElevatorTrapezoidCommand and the ElevatorHoldCommand
    // for how those get used)
    double nextHeight;
    double nextVelocity;
    double goalHeight;

    // we keep track of our voltage calculations for logging
    double lastFeedforward;
    double lastFeedback;
    double lastVolts;

    public ElevatorSubsystem(Motor motor) {

        this.motor = motor;
        this.pid = new PDController(p, d, maxFeedback, tolerance);
        this.currentCommand = "";
        this.goalHeight = Double.NaN;
        this.nextHeight = Double.NaN;
        this.nextVelocity = Double.NaN;
        this.lastFeedforward = Double.NaN;
        this.lastFeedback = Double.NaN;
        this.lastVolts = Double.NaN;

        SmartDashboard.putData("ElevatorSubsystem", builder -> {
            builder.addStringProperty("CurrentCommand", () -> currentCommand, null);
            builder.addDoubleProperty("LastFeedforward", () -> lastFeedforward, null);
            builder.addDoubleProperty("LastFeedback", () -> lastFeedback, null);
            builder.addDoubleProperty("LastVolts", () -> lastVolts, null);
            builder.addDoubleProperty("MotorAmps", motor::getCurrent, null);
            builder.addDoubleProperty("MotorVelocity", motor::getVelocity, null);
            builder.addDoubleProperty("HeightCurrent", this::getCurrentHeight, null);
            builder.addDoubleProperty("HeightError", pid::getError, null);
            builder.addDoubleProperty("HeightGoal", () -> goalHeight, null);
            builder.addDoubleProperty("HeightTarget", () -> nextHeight, null);
            builder.addDoubleProperty("VelocityCurrent", this::getCurrentVelocity, null);
            builder.addDoubleProperty("VelocityError", () -> nextVelocity - getCurrentVelocity(), null);
            builder.addDoubleProperty("VelocityTarget", () -> nextVelocity, null);
            builder.addBooleanProperty("AtGoal?", this::isAtGoal, null);
            builder.addBooleanProperty("Brake?", motor::isBrakeEnabled, motor::applyBrake);
        });
    }

    /**
     * Runs every scheduler loop to calculate current height and velocity
     * for use in command processing
     */
    @Override
    public void periodic() {
        currentHeight = motor.getPosition() * inchesPerRotation;
        currentVelocity = motor.getVelocity() * inchesPerRotation;
    }

    /** @return the elevator's current height in inches */
    public double getCurrentHeight() {
        return currentHeight;
    }

    /** @return the elevator's current velocity in inches per second */
    public double getCurrentVelocity() {
        return currentVelocity;
    }

    /** @return the current goal height */
    public double getGoalHeight() {
        return goalHeight;
    }

    /** @return is the elevator "close enough" to the specified height? */
    public boolean isAt(double height) {
        return MathUtil.isNear(height, getCurrentHeight(), tolerance.getAsDouble());
    }

    /** @return true if the elevator has a goal and is near it */
    public boolean isAtGoal() {
        return Double.isFinite(goalHeight) && isAt(goalHeight);
    }

    // =============================================================
    // OPEN LOOP
    // =============================================================

    /**
     * Run the elevator in open-loop mode by directly specifying the voltage;
     * this has the side effect of clearing all the target information
     * (including the goal height)
     */
    public void openLoop(String command, double volts) {

        currentCommand = command;

        // when we're open-loop mode, we don't have any targets in mind
        goalHeight = Double.NaN;
        nextHeight = Double.NaN;
        nextVelocity = Double.NaN;

        // clear out feedforward and feedback, and clamp volts
        lastFeedforward = Double.NaN;
        lastFeedback = Double.NaN;
        lastVolts = Util.clampVolts(volts);

        motor.applyVolts(volts);
    }

    // =============================================================
    // CLOSED LOOP
    // =============================================================

    /**
     * Run the elevator in closed-loop mode by specifying a "next" height
     * and velocity, and an optional final goal height
     */
    public void closedLoop(String command,
                            double height,
                            double velocity,
                            double goal) {

        currentCommand = command;

        // set the goal and "next" height and velocity
        // we clamp these to make sure we don't go too far or too fast
        goalHeight = MathUtil.clamp(goal, minHeight.getAsDouble(), maxHeight.getAsDouble());
        nextHeight = MathUtil.clamp(height, minHeight.getAsDouble(), maxHeight.getAsDouble());
        nextVelocity = Util.applyClamp(velocity, maxVelocity);

        // calculate feedforward and feedback and sum them
        lastFeedforward = g.getAsDouble() + v.getAsDouble() * nextVelocity;
        lastFeedback = pid.calculate(getCurrentHeight(), nextHeight);
        lastVolts = Util.clampVolts(lastFeedforward + lastFeedback);

        motor.applyVolts(lastVolts);
    }

    /**
     * Reset the PID controller and trapezoid profile to reflect the
     * most recent configuration, and clear calculated error; all closed
     * loop commands (hold, tune, trapezoid) should call this when they
     * initialize
     */
    public void resetPid() {
        pid.reset();
    }

    // =============================================================
    // COMMANDS
    // =============================================================

    /** @return a command to run the elevator by joystick control */
    public Command teleopCommand(DoubleSupplier input) {
        return new ElevatorTeleopCommand(this, input);
    }

    /** @return a command that will release the elevator by applying 0 volts */
    public Command releaseCommand() {

        // You don't want this to be the default command unless you're
        // carefully watching the elevator (for instance, while tuning it),
        // since it means the elevator will come down quickly from whatever
        // height it's at
        return run(() -> openLoop("release", 0.0));
    }

    /** @return a command for tuning the elevator */
    public Command tuningCommand() {
        return new ElevatorTuningCommand(this);
    }

    /** @return a command to move the elevator to a preset height */
    public Command moveToHeightCommand(String name, DoubleSupplier heightSupplier) {
        return new ElevatorTrapezoidCommand(this, name, heightSupplier);
    }

    /** @return a command to move the elevator to a preset height */
    public Command moveToHeightCommand(ElevatorPreset preset) {
        return moveToHeightCommand(preset.name(), preset.getHeight());
    }

    /** @return a command that holds the elevator still */
    public Command holdCommand() {
        return new ElevatorHoldCommand(this);
    }
}
