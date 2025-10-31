package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Motor;
import frc.robot.util.PDController;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.accelerationFactor;
import static frc.robot.subsystems.elevator.ElevatorConfig.d;
import static frc.robot.subsystems.elevator.ElevatorConfig.g;
import static frc.robot.subsystems.elevator.ElevatorConfig.inchesPerRotation;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxFeedback;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxHeight;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxTeleopVolts;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxVelocity;
import static frc.robot.subsystems.elevator.ElevatorConfig.minHeight;
import static frc.robot.subsystems.elevator.ElevatorConfig.p;
import static frc.robot.subsystems.elevator.ElevatorConfig.presetL1;
import static frc.robot.subsystems.elevator.ElevatorConfig.presetL2;
import static frc.robot.subsystems.elevator.ElevatorConfig.presetL3;
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

    public static final String TUNING_VELOCITY_KEY = "ElevatorSubsystem/TuningVelocity";

    public enum Preset {
        L1,
        L2,
        L3
    }

    final Motor motor;
    final PDController pid;
    TrapezoidProfile profile;
    String currentCommand;
    double goalHeight;
    double timeToGoal;
    double nextHeight;
    double nextVelocity;
    double lastFeedforward;
    double lastFeedback;
    double lastVolts;

    public ElevatorSubsystem(Motor motor) {

        this.motor = motor;
        this.pid = new PDController(p, d, maxFeedback, tolerance);
        this.currentCommand = "";
        this.goalHeight = Double.NaN;
        this.timeToGoal = Double.NaN;
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
            builder.addDoubleProperty("HeightCurrent", this::getHeight, null);
            builder.addDoubleProperty("HeightError", pid::getError, null);
            builder.addDoubleProperty("HeightGoal", () -> goalHeight, null);
            builder.addDoubleProperty("HeightNext", () -> nextHeight, null);
            builder.addDoubleProperty("TimeToGoal", () -> timeToGoal, null);
            builder.addDoubleProperty("VelocityCurrent", this::getVelocity, null);
            builder.addDoubleProperty("VelocityError", () -> nextVelocity - getVelocity(), null);
            builder.addDoubleProperty("VelocityNext", () -> nextVelocity, null);
            builder.addBooleanProperty("AtGoal?", this::atGoal, null);
            builder.addBooleanProperty("Brake?", motor::isBrakeEnabled, motor::applyBrake);
        });
    }

    /** @return the elevator's current height in inches */
    public double getHeight() {
        return motor.getPosition() * inchesPerRotation;
    }

    /** @return the elevator's current velocity in inches per second */
    public double getVelocity() {
        return motor.getVelocity() * inchesPerRotation;
    }

    /**
     * @return does the elevator have a goal height and, if so, is it
     * "close enough" to its goal?
     */
    public boolean atGoal() {
        return Double.isFinite(goalHeight) && pid.at(getHeight(), goalHeight);
    }

    /** @return the height associated with the specified preset */
    public DoubleSupplier getPresetHeight(Preset preset) {
        return switch (preset) {
            case L1 -> presetL1;
            case L2 -> presetL2;
            case L3 -> presetL3;
        };
    }

    /**
     * Run the elevator in open-loop mode by directly specifying the voltage;
     * this has the side effect of clearing all the target information
     * (including the goal height)
     */
    private void openLoop(String command, double volts) {
        currentCommand = command;
        goalHeight = Double.NaN;
        nextHeight = Double.NaN;
        nextVelocity = Double.NaN;
        lastFeedforward = Double.NaN;
        lastFeedback = Double.NaN;
        lastVolts = Util.clampVolts(volts);
        motor.applyVolts(volts);
    }

    /**
     * Reset the PID controller and trapezoid profile to reflect the
     * most recent configuration, and clear calculated error
     */
    private void resetPidAndProfile() {

        // reset the PID
        pid.reset();

        // it's a little easier to think about acceleration as a multiple
        // of velocity, rather than an absolute number
        double maxV = maxVelocity.getAsDouble();
        double maxA = maxV * accelerationFactor.getAsDouble();
        profile = new TrapezoidProfile(new Constraints(maxV, maxA));
    }

    /**
     * Run the elevator in closed-loop mode by specifying a "next" height
     * and velocity, and an optional final goal height
     */
    private void closedLoop(String command,
                            double height,
                            double velocity,
                            double goal) {
        currentCommand = command;
        goalHeight = goal;
        nextHeight = MathUtil.clamp(height, minHeight.getAsDouble(), maxHeight.getAsDouble());
        nextVelocity = Util.applyClamp(velocity, maxVelocity);
        lastFeedforward = g.getAsDouble() + v.getAsDouble() * nextVelocity;
        lastFeedback = pid.calculate(getHeight(), nextHeight);
        lastVolts = Util.clampVolts(lastFeedforward + lastFeedback);
        motor.applyVolts(lastVolts);
    }

    /**
     * @return a command that will run the elevator up or down at a
     * voltage controlled by a joystick
     */
    public Command teleopCommand(DoubleSupplier supplier) {
        return run(() -> {
            double input = supplier.getAsDouble() * maxTeleopVolts.getAsDouble();
            if (input > 0.0) {
                openLoop("tele-up", input);
            } else {
                openLoop("tele-down", input);
            }
        });
    }

    /**
     * @return a command that will release the elevator by applying zero
     * voltage (this should probably NOT be the default command, or else
     * the elevator may come crashing down)
     */
    public Command releaseCommand() {
        return run(() -> {
            openLoop("release", 0.0);
        });
    }

    /**
     * @return a command that tries to apply a constant velocity to
     * the elevator from whatever height it's currently at
     */
    public Command tuningCommand() {

        // this allows setting velocity from the dashboard
        SmartDashboard.putNumber(TUNING_VELOCITY_KEY, 0.0);

        return new Command() {

            // every cycle we will calculate a new target height and velocity
            double tuningHeight;
            double tuningVelocity;
            double tuningGoal;

            @Override
            public void initialize() {

                // reset PID each time we run to get latest configuration
                resetPidAndProfile();

                // start wherever the elevator is now
                tuningHeight = getHeight();
                tuningVelocity = SmartDashboard.getNumber(TUNING_VELOCITY_KEY, 0.0);

                // our goal depends on the velocity
                if (tuningVelocity < 0.0) {
                    tuningGoal = minHeight.getAsDouble();
                } else if (tuningVelocity > maxVelocity.getAsDouble()) {
                    tuningGoal = maxVelocity.getAsDouble();
                } else {
                    tuningGoal = tuningHeight;
                }

                Util.log("[elevator] tuning from %.2f @ %.2f", tuningHeight, tuningVelocity);
            }

            @Override
            public void execute() {

                // apply the target height and velocity
                closedLoop("tuning", tuningHeight, tuningVelocity, tuningGoal);

                // if we haven't yet hit the limits, calculate next height
                // based on velocity
                boolean belowMax = tuningHeight < maxHeight.getAsDouble();
                boolean aboveMin = tuningHeight > minHeight.getAsDouble();
                if (aboveMin && belowMax) {
                    tuningHeight += tuningVelocity * Util.DT;
                }
            }
        };
    }

    /**
     * @return a command that will move the elevator to a specific height
     * using a trapezoid motion profile
     */
    public Command trapezoidCommand(String command, DoubleSupplier heightSupplier) {

        return new Command() {

            final Timer timer = new Timer();
            State startState;
            State finalState;

            @Override
            public void initialize() {

                // reset PID and profile each time we run to get latest configuration
                resetPidAndProfile();

                // start wherever the elevator is now and move to the target
                startState = new State(getHeight(), getVelocity());
                finalState = new State(heightSupplier.getAsDouble(), 0.0);

                Util.log("[elevator] moving to %s @ %.2f",
                        command,
                        finalState.position);

                timeToGoal = profile.totalTime();

                timer.restart();
            }

            @Override
            public void execute() {
                double now = timer.get();
                State next = profile.calculate(now, startState, finalState);
                closedLoop(command, next.position, next.velocity, finalState.position);
                timeToGoal = profile.totalTime() - now;
            }

            @Override
            public boolean isFinished() {
                return profile.isFinished(timer.get());
            }

            @Override
            public void end(boolean interrupted) {
                // we finish when time has expired on the profiled motion
                // if the elevator isn't properly tuned, we might not have made it
                if (!atGoal()) {
                    Util.log("[elevator] !!! MISSED goal %s !!!", command);
                }
                timer.stop();
                timeToGoal = Double.NaN;
            }
        };
    }

    /**
     * @return a command to move the elevator slowly to a preset height
     * using the most current motion profile
     */
    public Command presetCommand(Preset preset) {
        return trapezoidCommand(preset.name(), getPresetHeight(preset));
    }

    /**
     * @return a command that holds the elevator still - the target height
     * will either be the last closed-loop target height (if we were just
     * operating in closed loop), or the current height (if we were just
     * in teleop mode)
     */
    public Command holdCommand() {
        return run(() -> {

            double holdHeight;

            // if there IS a target height, let's keep holding at that height
            // (this will ensure a smooth transition if a trapezoid motion
            // command just finished)
            if (Double.isFinite(nextHeight)) {
                holdHeight = nextHeight;
            }

            // otherwise, we were probably just in teleop mode or something;
            // we will just hold still at the current height
            else {
                holdHeight = getHeight();
            }

            // note that we totally ignore the goal height here; if we just
            // finished a trapezoid command, we want to keep it the same
            closedLoop("hold", holdHeight, 0.0, goalHeight);
        });
    }
}
