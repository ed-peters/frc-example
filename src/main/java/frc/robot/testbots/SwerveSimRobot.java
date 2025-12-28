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
        controller.a().onTrue(SwerveAutoPoseCommand.relative(
                        drive,
                        drive::getPose,
                        speeds -> drive.drive("auto", speeds),
                        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(15.0))));

        // b will move the robot one meter straight ahead, and turn it around
        // to face the position is was just occupying
        controller.b().onTrue(SwerveAutoPoseCommand.relative(
                        drive,
                        drive::getPose,
                        speeds -> drive.drive("auto", speeds),
                        new Pose2d(1.0, 0.0, Rotation2d.k180deg)));

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
     * Creates a command to move to a fixed pose
     */
    private Command moveToFixedPose(Pose2d pose) {
        return SwerveAutoPoseCommand.absolute(
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

        return SwerveAutoPoseCommand.relative(
                drive,
                drive::getPose,
                speeds -> drive.drive("face-"+wall, speeds),
                new Pose2d(0.0, 0.0, heading));
    }

    /**
     * This shows how you can combine (a) calculating a new heading so you are
     * facing an object on the field, and (b) using a custom center of rotation,
     * to rotate around a field object (in this case, the blue reef)
     */
    private Command rotateAroundReefCommand(double degreesPerSecond) {

        // this is the pose of the blue reef on the 2025 Reefscape fields
        // (we don't really care about the heading)
        Pose2d reefCenter = new Pose2d(new Translation2d(
                Units.inchesToMeters(144.0 + (209.49 - 144.0) / 2.0),
                Units.inchesToMeters(130.17 + (186.82 - 130.17) / 2.0)),
                Rotation2d.kZero);

        // this is how fast we will rotate around the reef, once we're
        // facing it
        ChassisSpeeds rotateAroundReefSpeed = new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(degreesPerSecond));

        // we defer our calculations until the moment the command is run, since
        // they depend on the current position of the robot
        return drive.defer(() -> {

            // this calculates where the reef center is relative to the robot at
            // the moment the command starts
            Pose2d relativeCenter = reefCenter.relativeTo(drive.getPose());

            // this is the heading that will face the robot in the direction of the
            // center of the reef
            Rotation2d rotationAngle = drive.getHeading().plus(
                    relativeCenter.getTranslation().getAngle());

            // this command will face the robot in that direction
            Command faceReef = moveToFixedPose(new Pose2d(
                    drive.getPose().getTranslation(), 
                    rotationAngle));

            // once we're facing the reef, the center of rotation is directly
            // in front of us, at the same distance it was earlier
            Translation2d centerOfRotation = new Translation2d(
                    relativeCenter.getTranslation().getNorm(),
                    0.0);

            // this will move us around that center of rotation
            Command rotateAroundReef = drive.run(() -> {
                drive.drive(
                        "rotate-reef", 
                        rotateAroundReefSpeed, 
                        centerOfRotation);
            });

            return faceReef.andThen(rotateAroundReef);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
