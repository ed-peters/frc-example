package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.Util;

import static frc.robot.subsystems.swerve.SwerveConfig.kinematics;
import static frc.robot.subsystems.swerve.SwerveConfig.useFusedPose;
import static frc.robot.subsystems.swerve.SwerveConfig.useVisionHeading;

/**
 * Implements the logic for "fusing" both odometry and vision-based pose
 * estimation. Also publishes poses to the dashboard, so you can visualize
 * them using AdvantageScope, and allows you to reset the current pose. See
 * the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html">WPILib
 * docs</a> for more background on fused estimates.
 */
public class SwervePoseCalculator {

    public enum PoseType {
        ODOMETRY,
        VISION,
        FUSED
    }

    final SwerveChassis chassis;
    final SwerveDriveOdometry odometry;
    final SwerveDrivePoseEstimator estimator;
    Pose2d latestVisionPose;
    Pose2d latestOdometryPose;
    Pose2d latestFusedPose;
    double latestVisionTimestamp;

    public SwervePoseCalculator(SwerveChassis chassis, Pose2d initialPose) {

        this.chassis = chassis;
        this.odometry = new SwerveDriveOdometry(
                kinematics,
                chassis.getGyroHeading(),
                chassis.getModulePositions(),
                initialPose);
        this.estimator = new SwerveDrivePoseEstimator(
                kinematics,
                chassis.getGyroHeading(),
                chassis.getModulePositions(),
                initialPose);
        this.latestVisionPose = Util.NAN_POSE;
        this.latestOdometryPose = initialPose;
        this.latestFusedPose = initialPose;
        this.latestVisionTimestamp = Double.NaN;

    }

    /**
     * @return the latest pose estimate (this will either be odometry-only
     * or fused depending on the setting of {@link SwerveConfig#useFusedPose}
     */
    public Pose2d getPoseEstimate() {
        return getPoseEstimate(useFusedPose.getAsBoolean()
                ? PoseType.FUSED
                : PoseType.ODOMETRY);
    }

    /**
     * @return the latest pose estimate of the specified type
     */
    public Pose2d getPoseEstimate(PoseType type) {
        return switch (type) {
            case ODOMETRY -> latestOdometryPose;
            case VISION -> latestVisionPose;
            case FUSED -> latestFusedPose;
        };
    }

    /**
     * Add a vision pose to the estimator. The parameters indicate how much
     * to "trust" the vision estimate, and approximately how old it is (in
     * seconds since the robot started up).
     */
    public void addVisionEstimate(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        if (pose != null) {
            estimator.setVisionMeasurementStdDevs(stdDevs);
            estimator.addVisionMeasurement(pose, timestamp);
            latestVisionPose = pose;
            latestVisionTimestamp = timestamp;
        } else {
            latestVisionPose = Util.NAN_POSE;
            latestVisionTimestamp = Double.NaN;
        }
    }

    /**
     * Reset the pose of the robot to the specified value
     */
    public void resetPose(Pose2d newPose) {

        odometry.resetPosition(
                chassis.getGyroHeading(),
                chassis.getModulePositions(),
                newPose);
        latestOdometryPose = newPose;

        estimator.resetPosition(
                chassis.getGyroHeading(),
                chassis.getModulePositions(),
                newPose);
        latestFusedPose = newPose;
    }

    /**
     * Recalculates and publishes the latest pose estimates
     */
    public void calculateLatestPoses() {

        Rotation2d latestHeading = chassis.getGyroHeading();
        SwerveModulePosition [] latestPositions = chassis.getModulePositions();

        // update the odometry and calculate its pose estimate
        odometry.update(latestHeading, latestPositions);
        latestOdometryPose = odometry.getPoseMeters();

        // update the fused estimator and calculate its pose estimate
        estimator.update(latestHeading, latestPositions);
        latestFusedPose = estimator.getEstimatedPosition();

        // it may not be a good idea to trust the vision-based heading
        // estimate (the gyro is usually pretty accurate), so this may
        // wind up "overriding" it with the gyro heading from odometry
        if (!useVisionHeading.getAsBoolean()) {
            latestFusedPose = new Pose2d(
                    latestFusedPose.getX(),
                    latestFusedPose.getY(),
                    latestOdometryPose.getRotation());
        }

        // publish them as structs so we can see them in advantage scope
        Util.publishPose("FusedPose", latestFusedPose);
        Util.publishPose("OdometryPose", latestOdometryPose);
        Util.publishPose("VisionPose", latestVisionPose);
    }
}
