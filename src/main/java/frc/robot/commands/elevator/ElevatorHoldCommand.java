package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Holds the elevator at a constant height, determined by the most
 * recently applied setpoint, or the current height (if we were in
 * open loop mode)
 */
public class ElevatorHoldCommand extends Command {

    final ElevatorSubsystem elevator;
    double holdHeight;

    public ElevatorHoldCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.holdHeight = Double.NaN;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

        // reset the PID to refresh config and clear previous error
        elevator.resetPid();

        // we calculate a new hold height every time we run
        holdHeight = elevator.getTargetHeight();

        // if there's a current setpoint, we'll use that
        // this avoid "droop" when transitioning out of a preset height command
        if (Double.isFinite(holdHeight)) {
            System.out.printf("[elevator] holding at previous setpoint %.3f%n", holdHeight);
        }

        // if there isn't a current setpoint, we'll use the current height
        else {
            holdHeight = elevator.getCurrentHeight();
            System.out.printf("[elevator] holding at current position %.3f%n", holdHeight);
        }
    }

    @Override
    public void execute() {
        elevator.closedLoop("hold", holdHeight, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        holdHeight = Double.NaN;
    }
}
