package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Moves the elevator to a preset position using a trapezoidal motion
 * profile
 */
public class ElevatorToHeightCommand extends Command {

    final ElevatorSubsystem elevator;
    final String presetName;
    final DoubleSupplier heightSupplier;
    final Timer timer;
    State startState;
    State finalState;
    TrapezoidProfile profile;

    public ElevatorToHeightCommand(ElevatorSubsystem elevator,
                                   String presetName,
                                   DoubleSupplier heightSupplier) {
        this.elevator = elevator;
        this.presetName = presetName;
        this.heightSupplier = heightSupplier;
        this.timer = new Timer();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

        // reset the PID to refresh config and clear previous error
        elevator.resetPid();

        // calculate state and end state and fetch latest motion profile
        startState = new State(elevator.getCurrentHeight(), elevator.getCurrentVelocity());
        finalState = new State(heightSupplier.getAsDouble(), 0.0);
        profile = elevator.createProfile();

        System.out.printf("[elevator] moving to %s at %.3f%n", presetName, finalState.position);

        // start the timer going
        timer.restart();
    }

    @Override
    public void execute() {
        State nextState = profile.calculate(timer.get(), startState, finalState);
        elevator.closedLoop(presetName, nextState.position, nextState.velocity);
    }

    @Override
    public boolean isFinished() {
        // we are done when the time is elapsed
        // if we didn't get to the target position, the tuning is off
        return timer.get() > profile.totalTime();
    }

    @Override
    public void end(boolean interrupted) {
        double current = elevator.getCurrentHeight();
        double target = elevator.getTargetHeight();
        if (elevator.isAt(finalState.position)) {
            System.out.printf("[elevator] made it to preset %s%n", presetName);
        } else {
            System.out.printf("[elevator] DID NOT make it to preset %s%n", presetName);
        }
        timer.stop();
    }
}
