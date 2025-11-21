package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.Util;

/**
 * Applies 0 volts so the shooter is idle
 */
public class ShooterIdleCommand extends Command {

    final ShooterSubsystem shooter;

    public ShooterIdleCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Util.log("[shooter] idling");
    }

    @Override
    public void execute() {
        shooter.openLoop("release", 0.0);
    }
}
