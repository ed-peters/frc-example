package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorPreset;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorMotorSim;

/**
 * Implementation of {@link TimedRobot} that can be used to test
 * elevator-related commands or practice tuning
 */
public class ElevatorSimRobot extends TimedRobot {

    CommandXboxController controller;
    ElevatorSubsystem elevator;
    ElevatorMotorSim sim;

    public ElevatorSimRobot() {

        sim = new ElevatorMotorSim();
        elevator = new ElevatorSubsystem(sim);
        controller = new CommandXboxController(0);

        // mapping for tuning
        // default behavior is to release the motor (apply 0 voltage)
        // only button mapped are for tuning and setting sim height
        // elevator.setDefaultCommand(elevator.releaseCommand());
        // controller.a().whileTrue(elevator.tuningCommand());
        // controller.b().whileTrue(elevator.holdCommand());
        // controller.x().onTrue(elevator.runOnce(() -> {
        //     System.out.println("setting sim height");
        //     sim.setHeight(70.0);
        // }));

        // mapping for presets
        // default behavior is to hold still (should be tuned accurately)
        // buttons are mapped to different preset heights
        elevator.setDefaultCommand(elevator.holdCommand());
        controller.a().onTrue(elevator.moveToHeightCommand(ElevatorPreset.L1));
        controller.b().onTrue(elevator.moveToHeightCommand(ElevatorPreset.L3));
        controller.x().whileTrue(elevator.releaseCommand());
        controller.y().onTrue(elevator.runOnce(() -> {
            sim.setHeight(70.0);
        }));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
