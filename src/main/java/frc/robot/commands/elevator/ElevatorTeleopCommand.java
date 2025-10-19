package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.*;

/**
 * Operates the elevator in teleop mode, holding still when there
 * is no input
 */
public class ElevatorTeleopCommand extends Command {

    final ElevatorSubsystem elevator;
    final DoubleSupplier input;
    double holdPosition;

    public ElevatorTeleopCommand(ElevatorSubsystem elevator, DoubleSupplier input) {
        this.elevator = elevator;
        this.input = input;
        this.holdPosition = Double.NaN;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

        // reset the PID to refresh config and clear previous error
        elevator.resetPid();

        System.out.println("[elevator] entering teleop");
    }

    @Override
    public void execute() {

        double volts = MathUtil.clamp(input.getAsDouble() * maxVolts, -maxVolts, maxVolts);

        // if we're applying volts, we clear the hold position
        if (volts != 0.0) {
            elevator.openLoop("tele-move", volts);
            holdPosition = Double.NaN;
        }

        // otherwise, we're holding still
        else {

            // the first time we hold still after moving, we capture current position
            if (Double.isNaN(holdPosition)) {
                holdPosition = elevator.getCurrentHeight();
                System.out.printf("[elevator] holding @ %.3f%n", holdPosition);
            }

            elevator.closedLoop("tele-hold", holdPosition, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        holdPosition = Double.NaN;
    }
}
