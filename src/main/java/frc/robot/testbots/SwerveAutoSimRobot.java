// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveAutoPoseCommand;
import frc.robot.commands.vision.ThreeStageTargetingCommand;
import frc.robot.subsystems.swerve.SwerveChassisSim;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem.Direction;
import frc.robot.subsystems.vision.LimelightSim;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.Util;

/**
 * Implementation of {@link TimedRobot} that can be used to test
 * swerve drive functionality in a simulator. This is hella useful
 * when you need to work through stuff and don't have the actual
 * robot.
 */
public class SwerveAutoSimRobot extends TimedRobot {

    SwerveDriveSubsystem drive;
    LimelightSubsystem limelight;
    LimelightSim limelightSim;
    CommandXboxController controller;

    public SwerveAutoSimRobot() {

        drive = new SwerveDriveSubsystem(new SwerveChassisSim());
        limelight = new LimelightSubsystem(drive);
        limelightSim = new LimelightSim(drive);
        controller = new CommandXboxController(0);

        drive.setDefaultCommand(drive.teleopCommand(controller));

        Transform2d t = new Transform2d(
            1.0,
            2.0,
            Rotation2d.kZero);

        // test orienting the chassis to a specific heading
        controller.a().onTrue(new SwerveAutoPoseCommand(drive, p -> p.transformBy(t)));
        controller.b().onTrue(new SwerveAutoPoseCommand(drive, p -> {
            return new Pose2d(p.getTranslation(), p.getRotation().plus(Rotation2d.kCCW_Pi_2));
        }));
        controller.x().onTrue(drive.runOnce(() -> drive.resetPose(Util.ZERO_POSE)));
    }

    @Override
    public void robotPeriodic() {
        limelightSim.updateFakePoses();
        CommandScheduler.getInstance().run();
    }
}
