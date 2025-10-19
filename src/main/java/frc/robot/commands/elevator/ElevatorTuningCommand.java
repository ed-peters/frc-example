package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static frc.robot.util.Util.chopDigits;

/**
 * Moves the elevator at a constant velocity, which can be controlled from
 * the dashboard
 */
public class ElevatorTuningCommand extends Command {

    final ElevatorSubsystem elevator;
    double tuningVelocity;
    double nextHeight;

    public ElevatorTuningCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.tuningVelocity = 5.0;
        this.nextHeight = Double.NaN;
        addRequirements(elevator);
        SmartDashboard.putData(getName(), builder -> {
            builder.addDoubleProperty("NextHeight", () -> chopDigits(nextHeight), null);
            builder.addDoubleProperty("TuningVelocity", () -> chopDigits(tuningVelocity), val -> tuningVelocity = val);
            builder.addDoubleProperty("HeightError", chopDigits(elevator::getHeightError), null);
            builder.addDoubleProperty("VelocityError", chopDigits(elevator::getVelocityError), null);
        });
    }

    @Override
    public void initialize() {

        // reset the PID to refresh config and clear previous error
        elevator.resetPid();

        // start at the elevator's current height
        nextHeight = elevator.getCurrentHeight();

        System.out.printf("[elevator] tuning w/ pos=%.3f, vel=%.3f%n", nextHeight, tuningVelocity);
    }

    @Override
    public void execute() {

        // apply the target height and velocity
        elevator.closedLoop("tuning", nextHeight, tuningVelocity);

        // update the target height for the next go-round
        nextHeight += tuningVelocity * 0.02;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("quitting tuning");
        nextHeight = Double.NaN;
    }
}
