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
import frc.robot.subsystems.vision.LimelightSim;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.commands.vision.ThreeStageTargetingCommand;

public class SwerveSimRobot extends TimedRobot {

    SwerveChassisSim sim;
    SwerveDriveSubsystem drive;
    LimelightSubsystem limelight;
    LimelightSim limelightSim;
    CommandXboxController controller;

    public SwerveSimRobot() {

        sim = new SwerveChassisSim();
        drive = new SwerveDriveSubsystem(sim);
        limelight = new LimelightSubsystem(drive);
        limelightSim = new LimelightSim(drive);
        controller = new CommandXboxController(0);

        drive.setDefaultCommand(drive.teleopCommand(controller));
        controller.a().onTrue(drive.alignToWallCommand(Direction.NORTH));
        controller.b().onTrue(drive.alignToWallCommand(Direction.EAST));
        controller.x().onTrue(drive.driveToOffsetCommand(new Translation2d(36.0, 72.0)));
        controller.y().onTrue(new ThreeStageTargetingCommand(drive, limelight, () -> null));
    }

    @Override
    public void robotPeriodic() {
        limelightSim.updateFakePoses();
        CommandScheduler.getInstance().run();
    }
}
