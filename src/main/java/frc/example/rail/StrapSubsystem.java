package frc.example.rail;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.example.Motor;
import frc.example.Util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.example.rail.StrapConfig.closeStallVelocity;
import static frc.example.rail.StrapConfig.closeVolts;
import static frc.example.rail.StrapConfig.defaultBrakeEnabled;

/**
 * Example of a subsystem controlling a mechanical strap based on the
 * 2025 Reefscape robot. It shows how to:
 *
 * <ul>
 *
 *     <li>Build a super-simple open-loop subsystem</li>
 *
 *     <li>Get teleop input from a joystick</li>
 *
 *     <li>Use command compositions to build complex behaviors out
 *     of simple ones</li>
 *
 *     <li>Use "stall detection" to run a motor until it has hit some
 *     kind of physical limit</li>
 *
 * </ul>
 */
public class StrapSubsystem extends SubsystemBase {

    final Motor motor;
    String currentCommand;

    public StrapSubsystem(Motor motor) {

        this.motor = motor;
        this.currentCommand = "";

        motor.applyBrake(defaultBrakeEnabled);

        SmartDashboard.putData("StrapSubsystem", builder -> {
            builder.addStringProperty("CurrentCommand", () -> currentCommand, null);
            builder.addDoubleProperty("MotorVelocity", motor::getVelocity, null);
            builder.addDoubleProperty("MotorAmps", motor::getCurrent, null);
            builder.addBooleanProperty("Brake?", motor::isBrakeEnabled, motor::applyBrake);
        });
    }

    /**
     * @return a command that applies zero volts (this should probably be
     * the default command for this subsystem)
     */
    public Command idleCommand() {
        return run(() -> {
            currentCommand = "idle";
            motor.applyVolts(0.0);
        });
    }

    /**
     * @return a command that will run the motor forward or backward at
     * a voltage controlled by a joystick
     */
    public Command teleopCommand(DoubleSupplier supplier) {
        return run(() -> {

            // assumes joystick input (-1.0, 1.0) and turns it into
            // a voltage value
            double input = Util.clampVolts(supplier.getAsDouble() * Util.MAX_VOLTS);

            currentCommand = input < 0 ? "tele-open" : "tele-close";
            motor.applyVolts(input);
        });
    }

    /**
     * @return a command that will apply voltage to the motor until it
     * is "stalled"; this is determined by checking the number of times
     * the motor velocity is reported below a configured limit
     */
    public Command closeCommand() {

        return Commands.deferredProxy(() -> {

            Util.log("[strap] closing ...");

            // closing the strap is simple - apply some volts
            Command close = run(() -> motor.applyVolts(closeVolts.getAsDouble()));

            // a debouncer is a filter that only changes state if the
            // underlying signal sustains a change for a period of time
            Debouncer debouncer = new Debouncer(
                    closeStallVelocity.getAsDouble(),
                    DebounceType.kRising);

            // this will check whether we're below the stall velocity and
            // supply that to the debouncer, which will only return "true"
            // if it's been happening consistently for a period of time
            BooleanSupplier isStalled = () -> {
                boolean belowVelocity = motor.getVelocity() < closeStallVelocity.getAsDouble();
                return debouncer.calculate(belowVelocity);
            };

            // we consider ourselves "stalled" when we've been below the
            // stall velocity for the configured timeout period
            return close.until(isStalled);
        });
    }
}
