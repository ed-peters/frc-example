package frc.example.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.example.Dash;
import frc.example.Util;
import frc.robot.commands.swerve.DriveTeleopCommand;
import frc.robot.commands.swerve.DriveToHeadingCommand;
import frc.robot.subsystems.swerve.SwerveChassis;
import frc.robot.subsystems.swerve.TurboSniperSpeedSupplier;

import java.util.function.Supplier;

import static frc.example.swerve.SwerveConfig.kinematics;
import static frc.example.swerve.SwerveConfig.maximumWheelSpeed;

/**
 * Interface for a swerve drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    final frc.robot.subsystems.swerve.SwerveChassis chassis;
    final SwerveDriveOdometry odometry;
    final SwerveDrivePoseEstimator estimator;
    final Matrix<N3,N1> visionStdDevs;
    Pose2d lastVisionPose;
    ChassisSpeeds lastSpeeds;
    String currentCommand;

    public SwerveDriveSubsystem(SwerveChassis chassis) {
        this.currentCommand = "";
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
        this.visionStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
        this.lastVisionPose = Util.ZERO_POSE;
        this.lastSpeeds = Util.ZERO_SPEED;
        this.currentCommand = "";
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
        Dash.log("[swerve] resetting pose to %s", pose);
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
     * Tells the robot to stop
     */
    public void stop() {
        drive("stop", Util.ZERO_SPEED);
    }

    /**
     * Tells the robot to drive at the specified speeds in "robot
     * relative" coordinates
     */
    public void drive(String command, ChassisSpeeds speeds) {

        currentCommand = command;

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

        Rotation2d heading = getHeading();
        SwerveModulePosition [] positions = chassis.getModulePositions();

        // update both the odometry and the fused pose estimator
        odometry.update(heading, positions);
        estimator.update(heading, positions);

        // published to smart dashboard for humans to read
        Dash.publish("SwerveDrive/CurrentCommand", currentCommand);
        Dash.publishFeet("SwerveDrive/SpeedX", lastSpeeds.vxMetersPerSecond);
        Dash.publishFeet("SwerveDrive/SpeedY", lastSpeeds.vxMetersPerSecond);
        Dash.publishDegrees("SwerveDrive/SpeedOmega", lastSpeeds.vxMetersPerSecond);

        // published as structs to view in AdvantageScope
        Dash.publish("SwerveDrive/Structs/PoseFused", getFusedPose());
        Dash.publish("SwerveDrive/Structs/PoseOdometry", getOdometryPose());
        Dash.publish("SwerveDrive/Structs/PoseVision", getVisionPose());
    }

    /**
     * @return a teleop command for this drive using the {@link frc.robot.subsystems.swerve.TurboSniperSpeedSupplier}
     */
    public Command teleopCommand(TurboSniperSpeedSupplier supplier) {
        return new DriveTeleopCommand(this, supplier);
    }

    /**
     * @return a command to drive to a fixed heading
     */
    public Command alignToHeadingCommand(String name, Supplier<Rotation2d> headingSupplier) {
        return new DriveToHeadingCommand(name, this, headingSupplier);
    }

    /**
     * @return a command to drive to a new pose
    public Command driveToPoseCommand(Function<Pose2d,Pose2d> poseFunction) {
        return new DriveToPoseCommand(this, poseFunction);
    }
     */

    /**
     * @return a command to drive to a new pose
    public Command driveAroundObstacleCommand(Function<Pose2d,Translation2d> centerFunction, DoubleSupplier maxSpeed, DoubleSupplier input) {
        return new DriveAroundObstacleCommand(this, centerFunction, maxSpeed, input);
    }
     */

    /**
     * @return a command to zero the heading of the robot
     */
    public Command zeroHeadingCommand() {
        return runOnce(this::resetToZeroHeading);
    }

    /**
     * @return a command to zero the pose of the robot
     */
    public Command zeroPoseCommand() {
        return runOnce(this::resetToZeroPose);
    }

    /**
     * @return a command to accept the most recent vision pose as the
     * current pose of the robot
     */
    public Command resetWithVisionPoseCommand() {
        return runOnce(this::resetWithVisionPose);
    }
}
