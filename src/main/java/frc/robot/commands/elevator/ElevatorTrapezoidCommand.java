package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.Util;
import frc.robot.util.motion.Motion;
import frc.robot.util.motion.Motions;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.maxAcceleration;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxVelocity;

/**
 * Uses a trapezoidal motion profile to move the elevator to a specific
 * height while honoring the maximum velocity and acceleration specified
 * in configuration.</p>
 *
 * This command won't work until you've properly tuned G, V and P. See
 * the comments on {@link ElevatorTuningCommand} for more about that.</p>
 *
 * In 2025 we tried using only P for the elevator. It worked OK, but we
 * would overshoot our target position a bit. If you experience this,
 * consider adding a small value of D (maybe starting at 0.1) to compensate
 * for that.</p>
 *
 * See the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#response-types">WPILib docs</a>
 * for some notes on what properly tuned (or "damped") motion looks like.</p>
 */
public class ElevatorTrapezoidCommand extends Command {

    final String name;
    final ElevatorSubsystem elevator;
    final DoubleSupplier heightSupplier;
    final Timer timer;
    Motion<State> motion;
    double goalHeight;

    public ElevatorTrapezoidCommand(ElevatorSubsystem elevator,
                                    String name,
                                    DoubleSupplier heightSupplier) {
        this.name = name;
        this.elevator = elevator;
        this.heightSupplier = heightSupplier;
        this.timer = new Timer();

        addRequirements(elevator);
        setName("ElevatorTrapezoidCommand");
    }

    @Override
    public void initialize() {

        // capture goal height
        goalHeight = heightSupplier.getAsDouble();
        Util.log("[elevator] moving to %s @ %.2f", name, goalHeight);

        // calculate motion profile
        motion = Motions.trapezoid(
                maxVelocity,
                maxAcceleration,
                elevator.getCurrentHeight(),
                elevator.getCurrentVelocity(),
                goalHeight);

        // reset stuff and get going
        elevator.resetPid();
        timer.restart();
    }

    @Override
    public void execute() {
        State state = motion.sample(timer.get());
        elevator.closedLoop(name,
                state.position,
                state.velocity,
                goalHeight);
    }

    @Override
    public boolean isFinished() {
        return motion == null || timer.hasElapsed(motion.totalTime());
    }

    @Override
    public void end(boolean interrupted) {

        // this command runs until the calculated profile time has elapsed;
        // if tuning isn't correct we may not reach the target height (we
        // might overshoot or undershoot). if you see this warning in the
        // logs a lot, you should probably redo your tuning.
        if (!elevator.isAtGoal()) {
            Util.log("[elevator] !!! MISSED goal %s !!!", name);
        }

        timer.stop();
    }
}
