// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.TargetingCommandBuilder;
import frc.robot.subsystems.swerve.SwerveChassisSim;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwervePoseCalculator.PoseType;
import frc.robot.subsystems.vision.LimelightSim;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.Util;

import java.util.Set;
import java.util.function.Predicate;

/**
 * Implementation of {@link TimedRobot} that shows the use of the
 * {@link LimelightSubsystem} and simulated Limelight
 */
public class LimelightSimRobot extends TimedRobot {

    public static final Set<Integer> BLUE_REEF_TAGS = Set.of(
        17, 18, 19, 20, 21, 22
    );

    SwerveDriveSubsystem drive;
    LimelightSubsystem limelight;
    LimelightSim limelightSim;
    TargetingCommandBuilder builder;
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

        // this will build some of our targeting commands for us
        builder = new TargetingCommandBuilder(
                limelight,
                drive,
                drive::getPose,
                speeds -> drive.drive("ll-target", speeds));

        controller = new CommandXboxController(0);

        // a will reset the robot's pose based on vision
        controller.a().onTrue(limelightPoseCommand());

        // b will home in on the nearest blue reef tag which is currently in
        // view of the camera
        controller.b().onTrue(drive.defer(() -> {
            AprilTag tag = closestBlueReefTag();
            return builder.reefscapeTargetingCommand(tag, null);
        }));

        // x will home in on the closest blue reef tag (which doesn't have to
        // be in view)
        controller.x().onTrue(drive.defer(() -> {
            AprilTag tag = closestBlueReefTag();
            return builder.swankyTargetingCommand(tag, 3.0, null);
        }));

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
     * @return a command to reset the drive's current pose to agree with
     * whatever is coming from the vision subsystem. This is very useful on
     * e.g. a practice field where you may not be able to start at a
     * predictable position.</p>
     *
     * In the simulator, this will cause a small jump in pose because the
     * simulator uses fake poses with a small offset from the real pose.</p>
     */
    private Command limelightPoseCommand() {
        return drive.runOnce(() -> {
            Pose2d newPose = limelight.getCurrentPose();
            if (newPose == null) {
                Util.log("[ll-pose] !!! NO TAG IN VIEW !!!");
            } else {
                drive.resetPose(newPose);
            }
        });
    }

    /**
     * @return the closest blue reef tag to the robot's current pose
     */
    private AprilTag closestBlueReefTag() {

        // this predicate will test whether the tag is a blue reef tag and
        // whether it's <10 feet from the pose of the robot whenever it is run
        Predicate<AprilTag> tagSelector = tag -> {
            Pose2d currentPose = drive.getPose();
            boolean reefTag = BLUE_REEF_TAGS.contains(tag.ID);
            boolean closeEnough = Util.feetBetween(currentPose, tag.pose.toPose2d()) < 10.0;
            return reefTag && closeEnough;
        };

        return Util.getClosestAprilTagMatching(
                drive.getPose(),
                tagSelector);
    }

    @Override
    public void robotPeriodic() {

        limelightSim.updateFakePoses();
        CommandScheduler.getInstance().run();

        // this shows how you can publish the pose of the closest april tag
        // to the robot; e.g. for debugging
        AprilTag closestTag = Util.getClosestAprilTagMatching(
                drive.getPose(),
                tag -> true);
        Util.publishPose("ClosestAprilTag", closestTag.pose.toPose2d());
    }
}
