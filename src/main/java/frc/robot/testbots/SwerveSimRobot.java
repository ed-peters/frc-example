// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveAutoPoseCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.swerve.SwerveChassisSim;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Implementation of {@link TimedRobot} that shows the use of the simulated
 * swerve chassis and the use of swerve commands
 */
public class SwerveSimRobot extends TimedRobot {

    public enum ArenaWall {
        EAST,
        WEST,
        NORTH,
        SOUTH
    }

    SwerveDriveSubsystem drive;
    CommandXboxController controller;

    public SwerveSimRobot() {

        drive = new SwerveDriveSubsystem(new SwerveChassisSim());
        controller = new CommandXboxController(0);

        drive.setDefaultCommand(teleopCommand());

        // this maps the d-pad to let you point to specific arena walls
        controller.povUp().onTrue(faceTheWallCommand(ArenaWall.NORTH));
        controller.povLeft().onTrue(faceTheWallCommand(ArenaWall.EAST));
        controller.povDown().onTrue(faceTheWallCommand(ArenaWall.SOUTH));
        controller.povRight().onTrue(faceTheWallCommand(ArenaWall.WEST));

        // a will rotate a little bit left (spam it to turn around fully)
        controller.a().onTrue(rotateSlightlyCommand(15.0));

        // b will scoot forward and flip around
        controller.b().onTrue(scootAndFlipAround());

        // x will rotate us slowly around the center of the blue reef
        controller.x().onTrue(rotateAroundReefCommand(90.0));

        // y will reset us to the zero pose
        controller.y().onTrue(drive.runOnce(() -> drive.resetPose(Pose2d.kZero)));

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
     * This is how you use the {@link SwerveAutoPoseCommand} with the simulated
     * swerve drive to move to a fixed pose on the field
     */
    private Command moveToFixedPose(Pose2d pose) {
        return new SwerveAutoPoseCommand(
                drive,
                drive::getPose,
                speeds -> drive.drive("auto", speeds),
                pose);
    }

    /**
     * This is how you wrap the above function to implement pointing at a fixed
     * heading when the driver presses a button
     */
    private Command faceTheWallCommand(ArenaWall wall) {

        // the target heading is fixed, depending on the target wall
        Rotation2d heading = switch (wall) {
            case EAST -> Rotation2d.kZero;
            case WEST -> Rotation2d.k180deg;
            case NORTH -> Rotation2d.kCCW_90deg;
            case SOUTH -> Rotation2d.kCW_90deg;
        };

        // use a deferred proxy because we dynamically calculate the target
        // pose when the command is run
        return drive.defer(() -> {
            Pose2d oldPose = drive.getPose();
            Pose2d newPose = new Pose2d(oldPose.getTranslation(), heading);
            return moveToFixedPose(newPose);
        });
    }

    /**
     * This is how you wrap the above function to implement rotating a small
     * amount from the current heading
     */
    private Command rotateSlightlyCommand(double degrees) {

        Rotation2d rotation = Rotation2d.fromDegrees(degrees);

        // use a deferred proxy because we dynamically calculate the target
        // pose when the command is run
        return drive.defer(() -> {
            Pose2d oldPose = drive.getPose();
            Pose2d newPose = new Pose2d(
                    oldPose.getTranslation(),
                    oldPose.getRotation().plus(rotation));
            return moveToFixedPose(newPose);
        });
    }

    /**
     * This is how you use a custom center of rotation to move the robot
     * around an anchor point on the field
     */
    private Command rotateAroundReefCommand(double degreesPerSecond) {

        ChassisSpeeds speeds = new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(degreesPerSecond));

        // this is the center of the blue reef on the 2025 Reefscape fields
        Translation2d reefCenter = new Translation2d(
                Units.inchesToMeters(144.0 + (209.49 - 144.0) / 2.0),
                Units.inchesToMeters(130.17 + (186.82 - 130.17) / 2.0));

        // when we run, we calculate the relative position of the reef center
        // to the center of the robot, and that becomes our center of rotation
        return drive.defer(() -> {
            Translation2d robotCenter = drive.getPose().getTranslation();
            Translation2d relativeCenter = reefCenter.minus(robotCenter);
            return drive.run(() -> {
                drive.drive("rotate-reef", speeds, relativeCenter);
            });
        });
    }

    /**
     * This is how you wrap the above function to implement a more complex
     * position relative to the current robot pose
     */
    private Command scootAndFlipAround() {

        // this transformation will move the robot one meter straight ahead,
        // and turn it around to face the position is was just occupying
        Transform2d transform = new Transform2d(
                new Translation2d(1.0, 0.0),
                Rotation2d.k180deg);

        return drive.defer(() -> {
            Pose2d oldPose = drive.getPose();
            Pose2d newPose = oldPose.transformBy(transform);
            return moveToFixedPose(newPose);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
