package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.auto.AutonomousSubsystem;
import frc.robot.subsystems.swerve.SwerveChassisSim;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;

public class AutoSimRobot extends TimedRobot {

    CommandXboxController controller;
    SwerveDriveSubsystem drive;
    AutonomousSubsystem auto;
    Command command;

    public AutoSimRobot() {
        drive = new SwerveDriveSubsystem(new SwerveChassisSim());
        auto = new AutonomousSubsystem(drive);
        controller = new CommandXboxController(0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        command = auto.generateCommand();
        CommandScheduler.getInstance().schedule(command);
    }
}
