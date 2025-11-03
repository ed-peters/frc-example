package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.maxTeleopVolts;

/**
 * Simple teleop command for an elevator. This is mainly for testing - during
 * competition you really want to use different preset heights. It's too
 * hard to position the elevator accurately by hand during teleop.
 */
public class ElevatorTeleopCommand extends Command {

    final ElevatorSubsystem elevator;
    final DoubleSupplier input;
    double lastInput;
    double maxVolts;

    public ElevatorTeleopCommand(ElevatorSubsystem elevator, DoubleSupplier input) {
        this.elevator = elevator;
        this.input = input;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        lastInput = Double.NaN;

        // applying 12 full volts is probably too much for any reasonable
        // elevator; we apply a limit here for safety. if you're really
        // committed to teleop, you should probably have different limits
        // for going up and down (since gravity will make you go much faster
        // on the way down)
        maxVolts = maxTeleopVolts.getAsDouble();

        Util.log("[elevator] entering teleop; max volts = =%.2f", maxVolts);
    }

    @Override
    public void execute() {
        lastInput = MathUtil.clamp(input.getAsDouble(), -1.0, 1.0);
        elevator.openLoop("teleop", MathUtil.clamp(
                lastInput * Util.MAX_VOLTS,
                -maxVolts,
                maxVolts));
    }

    @Override
    public void end(boolean interrupted) {
        lastInput = Double.NaN;
        Util.log("[elevator] ending teleop");
    }
}
