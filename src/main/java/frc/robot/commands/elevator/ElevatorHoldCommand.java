package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.Util;

/**
 * Command for holding the elevator still. This is a good default command
 * for the elevator once it's tuned - in between moving to different preset
 * heights, it will hold still where it ended moving.
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

        double goalHeight = elevator.getGoalHeight();

        // when this command first tuns, we'll calculate the correct
        // height to hold at. if we had a goal height set, and we're
        // there, let's remain there
        if (Double.isFinite(goalHeight) && elevator.isAtGoal()) {
            holdHeight = goalHeight;
        }

        // if we didn't then one of two things happened: either (a) we
        // were just in teleop or (b) we were interrupted on our way
        // to a goal; either way, we'll just hold at the current height
        else {
            holdHeight = elevator.getCurrentHeight();
        }

        Util.log("[elevator] holding @ %.2f", holdHeight);

        elevator.resetPid();
    }

    @Override
    public void execute() {
        elevator.closedLoop("hold", holdHeight, 0.0, Double.NaN);
    }
}
