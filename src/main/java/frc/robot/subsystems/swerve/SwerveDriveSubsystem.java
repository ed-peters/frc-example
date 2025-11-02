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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveAlignPidCommand;
import frc.robot.util.Util;
import frc.robot.commands.swerve.SwerveAlignToHeadingCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.swerve.SwerveTeleopSpeedSupplier;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.SwerveConfig.kinematics;
import static frc.robot.subsystems.swerve.SwerveConfig.maximumWheelSpeed;

/**
 * Interface for a swerve drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    public static final String POSE_LOGGING_PREFIX = "SmartDashboard/SwerveDriveSubsystem/Poses";

    static Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

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
        this.lastVisionPose = Util.ZERO_POSE;
        this.lastOdometryPose = Util.ZERO_POSE;
        this.lastFusedPose = Util.ZERO_POSE;
        this.lastSpeeds = Util.ZERO_SPEED;
        this.currentCommand = "";

        SmartDashboard.putData("SwerveDriveSubsystem", builder -> {
            builder.addDoubleProperty("Heading", () -> currentHeading.getDegrees(), null);
            builder.addDoubleProperty("Speed/X", () -> lastSpeeds.vxMetersPerSecond, null);
            builder.addDoubleProperty("Speed/Y", () -> lastSpeeds.vyMetersPerSecond, null);
            builder.addDoubleProperty("Speed/Omega", () -> Math.toDegrees(lastSpeeds.omegaRadiansPerSecond), null);
            builder.addDoubleProperty("VisionPose/X", () -> lastVisionPose.getX(), null);
            builder.addDoubleProperty("VisionPose/Y", () -> lastVisionPose.getY(), null);
            builder.addDoubleProperty("VisionPose/Omega", () -> lastVisionPose.getRotation().getDegrees(), null);
            builder.addDoubleProperty("FusedPose/X", () -> lastFusedPose.getX(), null);
            builder.addDoubleProperty("FusedPose/Y", () -> lastFusedPose.getY(), null);
            builder.addDoubleProperty("FusedPose/Omega", () -> lastFusedPose.getRotation().getDegrees(), null);
            builder.addDoubleProperty("OdometryPose/X", () -> lastOdometryPose.getX(), null);
            builder.addDoubleProperty("OdometryPose/Y", () -> lastOdometryPose.getY(), null);
            builder.addDoubleProperty("OdometryPose/Omega", () -> lastOdometryPose.getRotation().getDegrees(), null);
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

    /** @return the current pose as calculated purely on the odometry */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /** @return the most recent vision-based estimate of the pose of the robot */
    public Pose2d getVisionPose() {
        return lastVisionPose;
    }

    /** @return the fused estimate (vision + odometry) of the pose of the robot */
    public Pose2d getFusedPose() {
        return estimator.getEstimatedPosition();
    }

    /**
     * Add a vision pose to the estimator. The parameters indicate how much
     * to "trust" the vision estimate, and approximately how old it is in
     * seconds since the robot started up.
     */
    public void addVisionPose(Pose2d pose, Matrix<N3,N1> stdDevs, double timestamp) {
        estimator.setVisionMeasurementStdDevs(stdDevs);
        estimator.addVisionMeasurement(pose, timestamp);
        lastVisionPose = pose;
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
        if (pose != null) {
            System.out.println("[swerve] refusing to reset pose (no available vision pose)");
        } else {
            resetPose(getVisionPose());
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
        publishPose("Fused", lastFusedPose);
        publishPose("Odometry", lastOdometryPose);
        publishPose("Vision", lastVisionPose);
    }

    /**
     * Publish a pose to the dashboard (automatically adds the "SmartDashboard"
     * prefix so it will show up under that topic in the dashboard)
     */
    public void publishPose(String key, Pose2d val) {

        // see if a publisher already exists
        StructPublisher<Pose2d> publisher = posePublishers.get(key);

        // create it if it doesn't (we add the SmartDashboard prefix so
        // it shows up next to other values we publish)
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic(POSE_LOGGING_PREFIX, Pose2d.struct)
                    .publish();
            posePublishers.put(key, publisher);
        }

        publisher.set(val);
    }

    // ========================================================
    // DRIVING COMMANDS
    // ========================================================

    /**
     * @return a teleop command for this drive using the "standard"
     * controls (left stick controls strafing, right stick controls
     * turning, left trigger is sniper, right trigger is turbo)
     */
    public Command teleopCommand(CommandXboxController controller) {

        // pushing right or forward on the joystick results in negative values, so
        // we invert them before using them
        DoubleSupplier leftX = () -> -controller.getLeftX();
        DoubleSupplier leftY = () -> -controller.getLeftY();
        DoubleSupplier rightX = () -> -controller.getRightX();

        // triggers controller sniper/turbo behavior
        BooleanSupplier sniperTrigger = () -> controller.getLeftTriggerAxis() > 0.5;
        BooleanSupplier turboTrigger = () -> controller.getRightTriggerAxis() > 0.5;

        return new SwerveTeleopCommand(this, new SwerveTeleopSpeedSupplier(
                leftX,
                leftY,
                rightX,
                turboTrigger,
                sniperTrigger));
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
        return Commands.deferredProxy(() -> new SwerveAlignToHeadingCommand(this, angle));
    }

    /** @return a command to drive to a relative offset of the current position */
    public Command driveToOffsetCommand(Translation2d offset) {

        // we'll use a proxy command so it picks up the latest tuning
        // properties every time it runs
        return Commands.deferredProxy(() -> new SwerveAlignPidCommand(this, offset));
    }

    // ========================================================
    // POSE COMMANDS
    // ========================================================

    /** @return a command to set the pose to 0 */
    public Command zeroPoseCommand() {
        return runOnce(() -> resetPose(Util.ZERO_POSE));
    }

    /** @return a command to set the pose to the most recent vision pose */
    public Command visionPoseCommand() {
        return runOnce(() -> {
            if (lastVisionPose == null) {
                Util.log("[swerve] can't reset pose (no vision pose available)");
            } else {
                resetPose(lastVisionPose);
            }
        });
    }
}
