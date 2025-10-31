// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorMotorSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Preset;

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
        elevator.setDefaultCommand(elevator.releaseCommand());
        controller.a().whileTrue(elevator.tuningCommand());
        controller.b().onTrue(elevator.runOnce(() -> {
            System.out.println("setting sim height");
            sim.setHeight(70.0);
        }));

        // elevator.setDefaultCommand(elevator.holdCommand());
        // controller.x().whileTrue(elevator.tuningCommand());
        // controller.y().onTrue(elevator.presetCommand(Preset.L1));
        // controller.a().onTrue(elevator.presetCommand(Preset.L2));
        // controller.b().onTrue(elevator.presetCommand(Preset.L3));
        
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
