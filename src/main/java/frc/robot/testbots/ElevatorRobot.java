// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorHeight;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.SimElevator;

public class ElevatorRobot extends TimedRobot {

    CommandXboxController controller;
    ElevatorSubsystem elevator;

    public ElevatorRobot() {
        elevator = new ElevatorSubsystem(new SimElevator());
        controller = new CommandXboxController(0);
        elevator.setDefaultCommand(elevator.holdCommand());
        controller.x().whileTrue(elevator.tuningCommand());
        controller.y().onTrue(elevator.presetCommand(ElevatorHeight.LEVEL1));
        controller.a().onTrue(elevator.presetCommand(ElevatorHeight.LEVEL2));
        controller.b().onTrue(elevator.presetCommand(ElevatorHeight.LEVEL3));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
