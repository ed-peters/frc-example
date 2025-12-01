// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

        controller.a().onTrue(drive.alignToWallCommand(Direction.NORTH));
        controller.b().onTrue(drive.rotateBy(Rotation2d.fromDegrees(15.0)));
        controller.x().onTrue(drive.driveTo(new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(30.0))));

        /*
        // drives to a position one meter in front of, and two meters to the
        // left of the current position
        controller.a().onTrue(drive.driveToOffsetCommand(new Translation2d(1.0, 2.0)));

        // rotates 90 degrees CCW (to the left)
        controller.b().onTrue(drive.rotateCommand(Rotation2d.kCCW_90deg));

        // drive to an absolute position
        controller.x().onTrue(new SwerveAutoPoseCommand(drive, new Pose2d(
                1.0,
                1.0,
                Rotation2d.fromDegrees(45.0))));
*/

        // zeros the pose of the robot
        controller.y().onTrue(drive.runOnce(() -> drive.resetPose(Util.ZERO_POSE)));
    }

    @Override
    public void robotPeriodic() {
        limelightSim.updateFakePoses();
        CommandScheduler.getInstance().run();
    }
}
