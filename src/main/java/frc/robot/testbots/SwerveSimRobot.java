// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveChassisSim;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem.Direction;

public class SwerveSimRobot extends TimedRobot {

    SwerveChassisSim sim;
    SwerveDriveSubsystem drive;
    CommandXboxController controller;

    public SwerveSimRobot() {

        sim = new SwerveChassisSim();
        drive = new SwerveDriveSubsystem(sim);
        controller = new CommandXboxController(0);

        drive.setDefaultCommand(drive.teleopCommand(controller));
        controller.a().onTrue(drive.alignToWallCommand(Direction.NORTH));
        controller.b().onTrue(drive.alignToWallCommand(Direction.EAST));
        controller.x().onTrue(drive.driveToOffsetCommand(new Translation2d(36.0, 72.0)));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
