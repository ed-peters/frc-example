package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.Util;

public class IntakeIdleCommand extends Command {

    final IntakeSubsystem intake;

    public IntakeIdleCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        Util.log("[intake] idling");
    }

    @Override
    public void execute() {
        intake.openLoop("release", 0.0);
    }
}
