package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.Arrays;

import static frc.robot.subsystems.vision.LimelightConfig.classicPoseKey;
import static frc.robot.subsystems.vision.LimelightConfig.limelightName;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagPoseKey;

/**
 * Simulates using a Limelight. We are pulling two tricks here:
 * <ul>
 *
 *     <li>Supplying a fake pose estimate that represents what we would be
 *     getting as we drive around the field; and,</li>
 *
 *     <li>Pretending to "see" a target when we are at a specific pose on
 *     the field.</li>
 *
 * </ul>
 *
 * This should be useful for testing targeting.
 */
public class LimelightSim {

    public static final double [] NO_TAG = new double[0];

    public static final double TAG_ID = 16.0;

    /**
     * This is the position on the field of AprilTag 16 in the 2025 game (this is
     * the one on the blue algae chute)
     */
    public static final Translation2d TAG_POSITION = new Translation2d(
            Units.inchesToMeters(235.73),
            0.0);

    final DoubleArrayPublisher classicPublisher;
    final DoubleArrayPublisher megaTagPublisher;
    final DoublePublisher taPublisher;
    final DoublePublisher txPublisher;
    final DoublePublisher tidPublisher;
    final SwerveDriveSubsystem drive;
    double detectionRadius;
    double poseError;

    public LimelightSim(SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.detectionRadius = 2.0;
        this.poseError = 2.0;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        classicPublisher = table.getDoubleArrayTopic(classicPoseKey).publish();
        megaTagPublisher = table.getDoubleArrayTopic(megaTagPoseKey).publish();
        tidPublisher = table.getDoubleTopic("tid").publish();
        taPublisher = table.getDoubleTopic("ta").publish();
        txPublisher = table.getDoubleTopic("tx").publish();

        SmartDashboard.putData("LimelightSim", builder -> {
            builder.addDoubleProperty("DetectionRadius", () -> detectionRadius, val -> detectionRadius = val);
            builder.addDoubleProperty("PoseError", () -> poseError, val -> poseError = val);
        });
    }

    public void updateFakePoses() {

        Pose2d odometry = drive.getOdometryPose();

        // we'll only fake pose estimates if the robot is within a few
        // meters of the tag

        double distanceToTag = odometry
                .getTranslation()
                .getDistance(TAG_POSITION);

        if (distanceToTag > detectionRadius) {
            classicPublisher.accept(NO_TAG);
            megaTagPublisher.accept(NO_TAG);
            txPublisher.accept(0.0);
            taPublisher.accept(0.0);
            tidPublisher.accept(0.0);
            return;
        }

        double tagArea = 1.0 - (distanceToTag / detectionRadius);

        double [] classicPose = generateFakePoseInfo(
            odometry.getTranslation(), 
            odometry.getRotation(), 
            distanceToTag,
            tagArea);
        classicPublisher.accept(classicPose);

        // we will report the mega tag algorithm with the same details,
        // but its error going in the opposite direction, to differentiate
        // it from the mega tag pose

        double [] megaTagPose = Arrays.copyOf(classicPose, classicPose.length);
        megaTagPose[0] -= 2.0 * Units.inchesToMeters(poseError);
        megaTagPose[1] -= 2.0 * Units.inchesToMeters(poseError);
        megaTagPublisher.accept(megaTagPose);

        // and finally, we'll update the basic targeting info

        txPublisher.accept(odometry.getX() - TAG_POSITION.getX());
        taPublisher.accept(tagArea);
        tidPublisher.accept(TAG_ID);

    }

    /*
     * This creates fake bot pose information for the robot, in the format that
     * the Limelight would generate it.
     */
    private double [] generateFakePoseInfo(
                    Translation2d robotPosition, 
                    Rotation2d robotHeading, 
                    double distanceToTag,
                    double tagArea) {

        return new double [] {

            // first 3 = translation (X, Y, Z) in meters

            // the LL is calculating where it "thinks" the robot is by using 
            // vision recognition and AprilTag information. this is pretty good,
            // but always a little bit off; we'll simulate that by using a small
            // offset from the robot's current pose. and we'll assume the robot
            // isn't going to leave the ground.
            robotPosition.getX() + Units.inchesToMeters(poseError),
            robotPosition.getY() + Units.inchesToMeters(poseError),
            0.0,

            // next 3 = rotation in degrees (roll, pitch, yaw)
            // we don't tend to trust the LL rotation calculation, so here we'll
            // just leave it the same as reported by the robot
            0.0,
            0.0,
            robotHeading.getDegrees(),

            0.0, // total latency
            1.0, // tag count
            0.0, // tag span (we don't use this)
            distanceToTag, // average distance from camera
            tagArea, // average tag area

            TAG_ID, // tag ID
            1.0, // horizontal offset to primary pixel
            1.0, // vertical offset to primary pixel
            tagArea, // tag area
            distanceToTag, // distance to camera
            distanceToTag, // distance to robot
            0.3  // ambiguity

        };
    }

}
