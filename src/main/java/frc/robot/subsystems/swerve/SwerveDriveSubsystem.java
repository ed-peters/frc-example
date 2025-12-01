package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.swerve.SwerveAutoRotateCommand;
// import frc.robot.commands.swerve.SwerveAutoTranslateCommand;
import frc.robot.util.Util;
import frc.robot.commands.swerve.SwerveTeleopCommand;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveConfig.kinematics;

/**
 * Interface for a swerve drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    public enum Direction {
        NORTH,
        SOUTH,
        EAST,
        WEST
    }

    final SwerveChassis chassis;
    final SwervePoseCalculator poseCalculator;
    final SwerveKinematicsCalculator stateCalculator;
    final List<Consumer<Pose2d>> poseResetListeners;
    Rotation2d latestGyroHeading;
    Pose2d latestPoseEstimate;
    ChassisSpeeds latestSpeeds;
    String currentCommand;

    public SwerveDriveSubsystem(SwerveChassis chassis) {

        this.chassis = chassis;
        this.poseCalculator = new SwervePoseCalculator(chassis, Util.ZERO_POSE);
        this.stateCalculator = new SwerveKinematicsCalculator(chassis);
        this.poseResetListeners = new ArrayList<>();
        this.latestSpeeds = Util.NAN_SPEED;
        this.latestGyroHeading = chassis.getGyroHeading();
        this.latestPoseEstimate = Util.ZERO_POSE;
        this.latestSpeeds = chassis.getCurrentSpeed();
        this.currentCommand = "";

        SmartDashboard.putData("SwerveDriveSubsystem", builder -> {
            builder.addDoubleProperty("GyroHeading", () -> latestGyroHeading.getDegrees(), null);
            builder.addDoubleProperty("PoseX", () -> Units.metersToFeet(latestPoseEstimate.getX()), null);
            builder.addDoubleProperty("PoseY", () -> Units.metersToFeet(latestPoseEstimate.getY()), null);
            builder.addDoubleProperty("PoseDegrees", () -> latestPoseEstimate.getRotation().getDegrees(), null);
            builder.addDoubleProperty("SpeedX", () -> Units.metersToFeet(latestSpeeds.vxMetersPerSecond), null);
            builder.addDoubleProperty("SpeedY", () -> Units.metersToFeet(latestSpeeds.vyMetersPerSecond), null);
            builder.addDoubleProperty("SpeedOmega", () -> Units.radiansToDegrees(latestSpeeds.omegaRadiansPerSecond), null);
        });
    }

    /** @return current speeds */
    public ChassisSpeeds getCurrentSpeed() {
        return chassis.getCurrentSpeed();
    }

    /** @return the pose calculator */
    public SwervePoseCalculator getPoseCalculator() {
        return poseCalculator;
    }

    /** @return kinematics for the drive */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** @return the heading of the robot */
    public Rotation2d getHeading() {
        return latestPoseEstimate.getRotation();
    }

    /** @return the rate of change of the heading in degrees per second */
    public double getYawRate() {
        return chassis.getYawRate();
    }

    /** @return the pose as calculated purely on the odometry */
    public Pose2d getPose() {
        return latestPoseEstimate;
    }

    /**
     * Adds a listener that gets called whenever the robot's pose gets
     * reset (for instance, at the beginning of auto)
     */
    public void addPoseResetListener(Consumer<Pose2d> listener) {
        poseResetListeners.add(listener);
    }

    /**
     * Reset the pose of the robot to the specified value. This will also
     * notify anything listening for pose resets.
     */
    public void resetPose(Pose2d newPose) {
        poseCalculator.resetPose(newPose);
        for (Consumer<Pose2d> listener : poseResetListeners) {
            listener.accept(newPose);
        }
        latestPoseEstimate = newPose;
        Util.log("[swerve] reset pose to %s and notified %d listeners",
                newPose,
                poseResetListeners.size());
    }

    /**
     * Tells the robot to drive at the specified speeds in "robot
     * relative" coordinates, with the center of rotation being the
     * center of the robot
     */
    public void drive(String command, ChassisSpeeds speeds) {
        drive(command, speeds, Translation2d.kZero);
    }

    /**
     * Tells the robot to drive at the specified speeds in "robot
     * relative" coordinates, with the specified center of rotation
     */
    public void drive(String command, ChassisSpeeds speeds, Translation2d centerOfRotation) {
        currentCommand = command;
        latestSpeeds = speeds;
        chassis.setModuleStates(stateCalculator.calculateStates(
                speeds,
                centerOfRotation));
    }

    /**
     * Updates and publishes odometry
     */
    @Override
    public void periodic() {
        poseCalculator.calculateLatestPoses();
        latestGyroHeading = chassis.getGyroHeading();
        latestPoseEstimate = poseCalculator.getPoseEstimate();
    }

    // ========================================================
    // COMMANDS
    // ========================================================

    /** @return a teleop command for this drive and controller */
    public Command teleopCommand(CommandXboxController controller) {
        return SwerveTeleopCommand.create(this, controller);
    }

    /** @return a command to rotate the robot around a point on the field */
    public Command rotateAroundPointCommand(Supplier<Pose2d> pointSupplier,
                                            DoubleSupplier speedSupplier) {

        // we'll use a proxy command because we need to calculate the
        // center of rotation each time we run the command
        return Commands.deferredProxy(() -> {

            // the center of rotation is the difference between the target
            // point and the current pose
            Pose2d point = pointSupplier.get();
            Translation2d centerOfRotation = point
                    .minus(getPose())
                    .getTranslation();

            Util.log("[swerve] rotating around point = %s; COR = %s",
                    point,
                    centerOfRotation);

            return run(() -> {
                drive(
                        "rotate",
                        new ChassisSpeeds(0.0, 0.0, speedSupplier.getAsDouble()),
                        centerOfRotation);
            });
        });
    }

    /** @return a command to align the robot to an arena wall */
    public Command alignToWallCommand(Direction direction) {

        // we can determine the angle from the direction once here
        Rotation2d angle = switch (direction) {
            case NORTH -> Rotation2d.fromDegrees(90.0);
            case SOUTH -> Rotation2d.fromDegrees(-90.0);
            case EAST -> Rotation2d.fromDegrees(0.0);
            case WEST -> Rotation2d.fromDegrees(180.0);
        };

        // we'll use a proxy command so it picks up the latest tuning
        // properties every time it runs
        // return Commands.deferredProxy(() -> new SwerveAutoRotateCommand(this, angle));
        throw new UnsupportedOperationException();
    }

    /** @return a command to drive to a relative offset of the current position */
    public Command driveToOffsetCommand(Translation2d offset) {

        // we'll use a proxy command so it picks up the latest tuning
        // properties every time it runs
        // return Commands.deferredProxy(() -> new SwerveAutoTranslateCommand(this, offset));
        throw new UnsupportedOperationException();
    }

    /** @return a command to set the pose to 0 */
    public Command zeroPoseCommand() {
        return runOnce(() -> resetPose(Util.ZERO_POSE));
    }
}
