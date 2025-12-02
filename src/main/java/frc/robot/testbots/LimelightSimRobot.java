// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.LimelightResetPoseCommand;
import frc.robot.commands.vision.LimelightCompoundTargetingCommand;
import frc.robot.subsystems.swerve.SwerveChassisSim;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwervePoseCalculator.PoseType;
import frc.robot.subsystems.vision.LimelightSim;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.Util;

/**
 * Implementation of {@link TimedRobot} that shows the use of the
 * {@link LimelightSubsystem} and simulated Limelight
 */
public class LimelightSimRobot extends TimedRobot {

    SwerveDriveSubsystem drive;
    LimelightSubsystem limelight;
    LimelightSim limelightSim;
    CommandXboxController controller;

    public LimelightSimRobot() {

        drive = new SwerveDriveSubsystem(new SwerveChassisSim());

        // this is how you connect the limelight subsystem to a drive
        limelight = new LimelightSubsystem(
                drive::getPose,
                drive::addVisionEstimate);

        // this tells the limelight simulator to use the odometry-only
        // pose estimate as the "base" pose for its fake tracking data
        limelightSim = new LimelightSim(
                () -> drive.getPoseCalculator().getPoseEstimate(PoseType.ODOMETRY)
        );

        controller = new CommandXboxController(0);

        // a will reset the robot's pose based on vision (using the simulator,
        // this will cause a small jump in pose because the simulator uses
        // fake poses with a small offset from the real pose)
        controller.a().onTrue(limelightPoseCommand());

        // b will home in on the current target in view
        controller.b().onTrue(targetingCommand());

        drive.setDefaultCommand(teleopCommand());

    }

    /**
     * This is how you use the {@link SwerveTeleopCommand} with the simulated
     * swerve drive
     */
    private Command teleopCommand() {
        return SwerveTeleopCommand.create(
                controller,
                drive,
                drive::getPose,
                speeds -> drive.drive("teleop", speeds));
    }

    /**
     * This is how you use the {@link LimelightResetPoseCommand} with the
     * simulated swerve drive
     */
    private Command limelightPoseCommand() {
        return new LimelightResetPoseCommand(limelight, drive, drive::resetPose);
    }

    /**
     * This is how you use the {@link LimelightCompoundTargetingCommand} with
     * the simulated swerve drive
     */
    private Command targetingCommand() {
        return LimelightCompoundTargetingCommand.create(limelight,
                drive,
                drive::getPose,
                speeds -> drive.drive("ll-target", speeds));
    }

    /**
     * This is how you can use the AprilTag API to find the tag that is
     * closest to the robot
     */
    private void updateClosestAprilTag() {
        Pose2d currentPose = drive.getPose();
        Pose2d closestPose = Util.getClosestAprilTag(currentPose).pose.toPose2d();
        Util.publishPose("ClosestAprilTag", closestPose);
    }

    @Override
    public void robotPeriodic() {
        limelightSim.updateFakePoses();
        CommandScheduler.getInstance().run();
        updateClosestAprilTag();
    }
}
