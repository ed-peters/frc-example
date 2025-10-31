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
        controller.a().onTrue(elevator.presetCommand(Preset.L1));
        controller.b().onTrue(elevator.presetCommand(Preset.L3));        
        controller.x().whileTrue(elevator.releaseCommand());
        controller.y().onTrue(elevator.runOnce(() -> {
            System.out.println("setting sim height");
            sim.setHeight(70.0);
        }));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
