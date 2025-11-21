package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

public class ShooterRpsCommand extends Command {

    final ShooterSubsystem shooter;
    final String name;
    final DoubleSupplier speedSupplier;
    double rps;

    public ShooterRpsCommand(ShooterSubsystem shooter, String name, DoubleSupplier speedSupplier) {
        this.shooter = shooter;
        this.name = name;
        this.speedSupplier = speedSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetPid();
        rps = speedSupplier.getAsDouble();
        Util.log("[shooter] running %s @ %.2f", name, rps);
    }

    @Override
    public void execute() {
        shooter.closedLoop(name, rps);
    }
}
