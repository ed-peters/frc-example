package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

public class IntakeRpsCommand extends Command {

    final IntakeSubsystem intake;
    final String name;
    final DoubleSupplier speedSupplier;
    double rps;

    public IntakeRpsCommand(IntakeSubsystem intake, String name, DoubleSupplier speedSupplier) {
        this.intake = intake;
        this.name = name;
        this.speedSupplier = speedSupplier;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.resetPid();
        rps = speedSupplier.getAsDouble();
        Util.log("[intake] running %s @ %.2f", name, rps);
    }

    @Override
    public void execute() {
        intake.closedLoop(name, rps);
    }
}
