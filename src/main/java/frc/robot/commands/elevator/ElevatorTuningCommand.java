package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.util.Util;

import static frc.robot.subsystems.elevator.ElevatorConfig.maxHeight;
import static frc.robot.subsystems.elevator.ElevatorConfig.minHeight;

/**
 * This is used for tuning. It's a simple process:
 * <ul>
 *
 *     <li>Set G, V, P and D to 0.</li>
 *
 *     <li>Release the elevator (apply 0 volts) and hold it at any
 *     starting position you want.</li>
 *
 *     <li>Set the tuning velocity to 0, Keep running this command
 *     with larger and larger values of G until the elevator just
 *     stays still where it starts (velocity should be ~0.0). If the
 *     elevator starts rising on its own, G is too high. You
 *     should get G dialed in to 2-3 digits of accuracy.</li>
 *
 *     <li>Now that you have G, set the tuning velocity to
 *     something over 0 (say, 6 inches per second). Run this command
 *     repeatedly, increasing V until the elevator achieves the target
 *     velocity. If the elevator is running too fast, V is too high.
 *     You should be able to get it accurate to less than 1 inch per
 *     second</li>
 *
 *     <li>Now that you have G and V, increase P to somewhere in the
 *     low single digits. This should remove any remaining position
 *     inaccuracy and you should be very closely tracking the
 *     target position.</li>
 *
 * </ul>
 *
 * See the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html">WPILib docs</a>
 * for another way to explain this. Note that we're ignoring the A
 * parameter for feedforward.
 */
public class ElevatorTuningCommand extends Command {

    public static final String PROP = "ElevatorSubsystem/TuningVelocity";

    final ElevatorSubsystem elevator;
    double tuningVelocity;
    double nextHeight;

    public ElevatorTuningCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.nextHeight = Double.NaN;
        this.tuningVelocity = Double.NaN;
        addRequirements(elevator);
        SmartDashboard.putNumber(PROP, 0.0);
    }

    @Override
    public void initialize() {

        nextHeight = elevator.getCurrentHeight();
        tuningVelocity = SmartDashboard.getNumber(PROP, 0.0);
        Util.log("[elevator] tuning from %.2f @ %.2f", nextHeight, tuningVelocity);

        elevator.resetPid();
    }

    @Override
    public void execute() {

        elevator.closedLoop("tuning", nextHeight, tuningVelocity, Double.NaN);

        // if we haven't yet hit the limits, calculate next height
        // based on velocity
        boolean belowMax = nextHeight < maxHeight.getAsDouble();
        boolean aboveMin = nextHeight > minHeight.getAsDouble();
        if (aboveMin && belowMax) {
            nextHeight += tuningVelocity * Util.DT;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.log("[elevator] tuning ended");
        nextHeight = Double.NaN;
        tuningVelocity = Double.NaN;
    }
}