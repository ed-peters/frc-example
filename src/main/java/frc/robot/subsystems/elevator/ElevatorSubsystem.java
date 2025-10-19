package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.ElevatorHoldCommand;
import frc.robot.commands.elevator.ElevatorTeleopCommand;
import frc.robot.commands.elevator.ElevatorToHeightCommand;
import frc.robot.commands.elevator.ElevatorTuningCommand;
import frc.robot.util.Dash;
import frc.robot.util.Dash.Toggle;
import frc.robot.util.PDController;
import frc.robot.util.Feedforward;
import frc.robot.util.Motor;
import frc.robot.util.PositionSetpoint;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.*;

/**
 * Implementation of a subsystem to control an elevator. Provides
 * position-based control and supports trapezoidal motion control.
 */
public class ElevatorSubsystem extends SubsystemBase {

    final Motor motor;
    final Feedforward feedforward;
    final PDController pid;
    final PositionSetpoint target;
    String currentCommand;
    double lastFeedforward;
    double lastFeedback;
    double lastVolts;
    Toggle brakeToggle;

    public ElevatorSubsystem(Motor motor) {

        this.motor = motor;
        this.feedforward = new Feedforward(g, v);
        this.pid = new PDController(p, d, maxFeedback, positionTolerance);
        this.target = new PositionSetpoint(minHeight, maxHeight, maxVelocity);
        this.currentCommand = "";
        this.lastFeedforward = Double.NaN;
        this.lastFeedback = Double.NaN;
        this.lastVolts = Double.NaN;

        // allow toggling brake mode from the dashboard
        this.brakeToggle = Dash.toggle("ElevatorSubsystem/Brake?",
                true,
                motor::applyBrakes);
    }

    /** @return current height in inches per second */
    public double getCurrentHeight() {
        return motor.getPosition() * inchesPerRotation;
    }

    /** @return target height in inches (NaN if we're in open loop) */
    public double getTargetHeight() {
        return target.position;
    }

    /** @return current velocity in inches per second */
    public double getCurrentVelocity() {
        return motor.getVelocity() * inchesPerRotation;
    }

    /** @return target velocity in inches per second (NaN if we're in open loop) */
    public double getTargetVelocity() {
        return target.velocity;
    }

    /** @return are we in closed loop mode? */
    public boolean isClosedLoop() {
        return target.hasSetpoint();
    }

    /** Resets the PID controller */
    public void resetPid() {
        pid.reset();
    }

    /** @return height error (NaN if we're in open loop) */
    public double getHeightError() {
        return isClosedLoop()
                    ? getTargetHeight() - getCurrentHeight()
                    : Double.NaN;
    }

    /** @return height error (NaN if we're in open loop) */
    public double getVelocityError() {
        return isClosedLoop()
                ? getTargetVelocity() - getCurrentVelocity()
                : Double.NaN;
    }

    /** @return do we have a target height and are we "close enough" to it? */
    public boolean isAtTarget() {
        return target.hasSetpoint() && isAt(target.position);
    }

    /** @return are we "closed enough" to the supplied height? */
    public boolean isAt(double height) {
        return MathUtil.isNear(height, getCurrentHeight(), positionTolerance.getAsDouble());
    }

    /**
     * @return a trapezoid profile with the most current
     * constraint values
     */
    public TrapezoidProfile createProfile() {
        double maxV = maxVelocity.getAsDouble();
        double maxA = maxV * accelerationFactor.getAsDouble();
        return new TrapezoidProfile(new Constraints(maxV, maxA));
    }

    // ==============================================================
    // OPEN LOOP
    // ==============================================================

    /**
     * Operates in open loop mode by applying the supplied voltage.
     * Also clears target height/velocity and feedforward/feedback.
     */
    public void openLoop(String command, double volts) {
        currentCommand = command;
        target.clear();
        lastFeedforward = Double.NaN;
        lastFeedback = Double.NaN;
        lastVolts = volts < 0.0
                ? MathUtil.clamp(volts, -maxVoltsDown.getAsDouble(), 0.0)
                : MathUtil.clamp(volts, 0.0, maxVoltsUp.getAsDouble());
        motor.applyVolts(lastVolts);
    }

    // ==============================================================
    // CLOSED LOOP
    // ==============================================================

    /**
     * Operates in closed loop mode by setting the target height/velocity
     * and calculating feedforward/feedback. Whenever the command name
     * changes, the PID will also be reset.
     */
    public void closedLoop(String command, double height, double velocity) {

        currentCommand = command;
        target.set(height, velocity);

        // calculate feedforward and feedback
        lastFeedforward = feedforward.elevator(target.velocity);
        lastFeedback = pid.calculate(getCurrentHeight(), target.position);
        lastVolts = Util.clampVolts(lastFeedforward + lastFeedback);

        motor.applyVolts(lastVolts);
    }

    // ==============================================================
    // PERIODIC
    // ==============================================================

    /**
     * Publish state to dashboard
     */
    @Override
    public void periodic() {

        // publish diagnostic information
        Dash.publish("ElevatorSubsystem/HeightCurrent", getCurrentHeight());
        Dash.publish("ElevatorSubsystem/HeightError", getHeightError());
        Dash.publish("ElevatorSubsystem/HeightTarget", getTargetHeight());
        Dash.publish("ElevatorSubsystem/VelocityCurrent", getCurrentVelocity());
        Dash.publish("ElevatorSubsystem/VelocityError", getVelocityError());
        Dash.publish("ElevatorSubsystem/VelocityTarget", getTargetVelocity());
        Dash.publish("ElevatorSubsystem/LastFeedforward", lastFeedforward);
        Dash.publish("ElevatorSubsystem/LastFeedback", lastFeedback);
        Dash.publish("ElevatorSubsystem/LastVolts", lastVolts);

        // check if we need to flip the brake mode
        brakeToggle.check();
    }

    // ==============================================================
    // COMMANDS
    // ==============================================================

    /** @return a teleop command */
    public Command teleopCommand(DoubleSupplier input) {
        return new ElevatorTeleopCommand(this, input);
    }

    /** @return a command to hold still at the current height */
    public Command holdCommand() {
        return new ElevatorHoldCommand(this);
    }

    /** @return a command to "release" the elevator by applying 0 volts */
    public Command releaseCommand() {
        return run(() -> openLoop("release", 0.0));
    }

    /** @return a command to move to a preset height */
    public Command presetCommand(ElevatorHeight preset) {
        return new ElevatorToHeightCommand(this, preset.name(), preset.height);
    }

    /** @return a command to move to a desired height */
    public Command moveToHeightCommand(String name, DoubleSupplier heightSupplier) {
        return new ElevatorToHeightCommand(this, name, heightSupplier);
    }

    /** @return a tuning command */
    public Command tuningCommand() {
        return new ElevatorTuningCommand(this);
    }
}
