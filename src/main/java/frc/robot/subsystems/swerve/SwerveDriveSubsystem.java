package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveTranslateCommand;
import frc.robot.util.Util;
import frc.robot.commands.swerve.SwerveRotateCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;

import static frc.robot.subsystems.swerve.SwerveConfig.kinematics;
import static frc.robot.subsystems.swerve.SwerveConfig.maximumWheelSpeed;

/**
 * Interface for a swerve drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    /** Implements a "quiet mode" to prevent spamming the dashboard during competition */
    public static final boolean ALL_THE_LOGS = true;

    public enum Direction {
        NORTH,
        SOUTH,
        EAST,
        WEST
    }

    final SwerveChassis chassis;
    final SwerveDriveOdometry odometry;
    final SwerveDrivePoseEstimator estimator;
    Pose2d lastVisionPose;
    Pose2d lastOdometryPose;
    Pose2d lastFusedPose;
    double lastVisionTimestamp;
    ChassisSpeeds lastSpeeds;
    Rotation2d currentHeading;
    String currentCommand;

    public SwerveDriveSubsystem(SwerveChassis chassis) {

        this.currentCommand = "";
        this.currentHeading = chassis.getHeading();
        this.chassis = chassis;
        this.odometry = new SwerveDriveOdometry(
                kinematics,
                chassis.getHeading(),
                chassis.getModulePositions());
        this.estimator = new SwerveDrivePoseEstimator(
                kinematics,
                chassis.getHeading(),
                chassis.getModulePositions(),
                Util.ZERO_POSE);
        this.lastVisionPose = Util.NAN_POSE;
        this.lastOdometryPose = Util.ZERO_POSE;
        this.lastFusedPose = Util.ZERO_POSE;
        this.lastSpeeds = Util.ZERO_SPEED;
        this.currentCommand = "";

        SmartDashboard.putData("SwerveDriveSubsystem", builder -> {
            builder.addDoubleProperty("Heading", () -> currentHeading.getDegrees(), null);
            builder.addDoubleProperty("Speed/X", () -> lastSpeeds.vxMetersPerSecond, null);
            builder.addDoubleProperty("Speed/Y", () -> lastSpeeds.vyMetersPerSecond, null);
            builder.addDoubleProperty("Speed/Omega", () -> Math.toDegrees(lastSpeeds.omegaRadiansPerSecond), null);
            if (ALL_THE_LOGS) {
                builder.addDoubleProperty("Poses/Vision/X", () -> lastVisionPose.getX(), null);
                builder.addDoubleProperty("Poses/Vision/Y", () -> lastVisionPose.getY(), null);
                builder.addDoubleProperty("Poses/Vision/Omega", () -> lastVisionPose.getRotation().getDegrees(), null);
                builder.addDoubleProperty("Poses/Vision/Timestamp", () -> lastVisionTimestamp, null);
                builder.addDoubleProperty("Poses/Fused/X", () -> lastFusedPose.getX(), null);
                builder.addDoubleProperty("Poses/Fused/Y", () -> lastFusedPose.getY(), null);
                builder.addDoubleProperty("Poses/Fused/Omega", () -> lastFusedPose.getRotation().getDegrees(), null);
                builder.addDoubleProperty("Poses/Odometry/X", () -> lastOdometryPose.getX(), null);
                builder.addDoubleProperty("Poses/Odometry/Y", () -> lastOdometryPose.getY(), null);
                builder.addDoubleProperty("Poses/Odometry/Omega", () -> lastOdometryPose.getRotation().getDegrees(), null);
            }
        });
    }

    /** @return kinematics for the drive */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * @return the current heading of the robot (you should always get this
     * from the gyro)
     */
    public Rotation2d getHeading() {
        return chassis.getHeading();
    }

    /** @return the pose as calculated purely on the odometry */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return the most recent vision-based estimate of the pose of the robot
     * (this will never be null, but might be all NaNs if there is no latest
     * pose)
     */
    public Pose2d getVisionPose() {
        return lastVisionPose;
    }

    /** @return the fused estimate (vision + odometry) of the pose of the robot */
    public Pose2d getFusedPose() {
        return estimator.getEstimatedPosition();
    }

    /**
     * Add a vision pose to the estimator. The parameters indicate how much
     * to "trust" the vision estimate, and approximately how old it is (in
     * seconds since the robot started up).
     */
    public void addVisionPose(Pose2d pose, Matrix<N3,N1> stdDevs, double timestamp) {
        if (pose != null) {
            estimator.setVisionMeasurementStdDevs(stdDevs);
            estimator.addVisionMeasurement(pose, timestamp);
            lastVisionPose = pose;
            lastVisionTimestamp = timestamp;
        } else {
            lastVisionPose = Util.NAN_POSE;
            lastVisionTimestamp = Double.NaN;
        }
    }

    /**
     * Reset the pose of the robot to the specified value
     */
    public void resetPose(Pose2d pose) {
        Util.log("[swerve] resetting pose to %s", pose);
        odometry.resetPose(pose);
        estimator.resetPose(pose);
        chassis.resetHeading(pose.getRotation());
    }

    /**
     * Reset the pose of the robot to (0, 0, 0)
     */
    public void resetToZeroPose() {
        resetPose(Util.ZERO_POSE);
    }

    /**
     * Reset the heading of the robot to the supplied angle
     */
    public void resetHeading(Rotation2d heading) {
        Pose2d currentPose = getFusedPose();
        Pose2d newPose = new Pose2d(currentPose.getTranslation(), heading);
        resetPose(newPose);
    }

    /**
     * Reset the heading of the robot to the 0
     */
    public void resetToZeroHeading() {
        resetHeading(Util.ZERO_ROTATION);
    }

    /**
     * Reset the pose of the robot to whatever the vision system thinks it is
     */
    public void resetWithVisionPose() {
        Pose2d pose = getVisionPose();
        if (pose == null || pose == Util.NAN_POSE) {
            Util.log("[swerve] refusing to reset pose from vision (no available pose)");
        } else {
            resetPose(pose);
        }
    }

    /**
     * Tells the robot to drive at the specified speeds in "robot
     * relative" coordinates
     */
    public void drive(String command, ChassisSpeeds speeds) {

        currentCommand = command;

        lastSpeeds = speeds;

        // use kinematics to turn drive speed into wheel states
        // this tells each wheel how fast to spin and at what angle
        SwerveModuleState [] states = kinematics.toSwerveModuleStates(speeds);

        // this will scale speeds down so no single wheel is ever turning
        // faster than the absolute maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                maximumWheelSpeed.getAsDouble());

        // drive!
        chassis.setModuleStates(states);
    }

    /**
     * Tells the robot to rotate around the specified point at the
     * specified speed
     */
    public void rotateAround(Translation2d centerOfRotation, double degreesPerSecond) {

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(degreesPerSecond));

        // this does the equations to decide how fast to spin the wheels
        // and in which directions, to accomplish rotation around the
        // supplied point
        SwerveModuleState [] states = kinematics.toSwerveModuleStates(speeds, centerOfRotation);

        // same as above - scale speeds down to avoid rotating any of
        // them faster than the hardware will allow
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                Units.feetToMeters(maximumWheelSpeed.getAsDouble()));

        chassis.setModuleStates(states);
    }

    /**
     * Updates and publishes odometry
     */
    @Override
    public void periodic() {

        currentHeading = getHeading();

        SwerveModulePosition [] positions = chassis.getModulePositions();

        // update both the odometry and the fused pose estimator
        odometry.update(currentHeading, positions);
        estimator.update(currentHeading, positions);

        // calculate these each go-round for monitoring
        lastFusedPose = getFusedPose();
        lastOdometryPose = getOdometryPose();

        // publish them as structs so we can see them in advantage scope
        Util.publishPose("FusedPose", lastFusedPose);
        Util.publishPose("OdometryPose", lastOdometryPose);
        Util.publishPose("VisionPose", lastVisionPose);
    }

    // ========================================================
    // COMMANDS
    // ========================================================

    /** @return a teleop command for this drive and controller */
    public Command teleopCommand(CommandXboxController controller) {
        return SwerveTeleopCommand.create(this, controller);
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
        return Commands.deferredProxy(() -> new SwerveRotateCommand(this, angle));
    }

    /** @return a command to drive to a relative offset of the current position */
    public Command driveToOffsetCommand(Translation2d offset) {

        // we'll use a proxy command so it picks up the latest tuning
        // properties every time it runs
        return Commands.deferredProxy(() -> new SwerveTranslateCommand(this, offset));
    }

    /** @return a command to set the pose to 0 */
    public Command zeroPoseCommand() {
        return runOnce(() -> resetPose(Util.ZERO_POSE));
    }

    /**
     * @return a command to set the pose to the most recent vision pose (this
     * is very useful on test fields)
     */
    public Command useVisionPoseCommand() {
        return runOnce(() -> {
            if (lastVisionPose == null) {
                Util.log("[swerve] can't reset pose (no vision pose available)");
            } else {
                resetPose(lastVisionPose);
            }
        });
    }
}
