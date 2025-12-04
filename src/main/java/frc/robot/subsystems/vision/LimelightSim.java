package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Util;

import java.util.Arrays;
import java.util.function.Supplier;

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
 *     the field (see {@link #TAG_POSITION})</li>
 *
 * </ul>
 *
 * This should be useful for testing targeting.
 */
public class LimelightSim {

    public static final double [] NO_TAG = new double[0];

    /**
     * This is the position on the field of AprilTag 16 in the 2025 game
     * (this is the one on the blue algae chute). If you drive the robot
     * close to that tag,
     */
    public static final Translation2d TAG_POSITION = new Translation2d(
            Units.inchesToMeters(235.73),
            0.0);

    /**
     * This is the angle (in degrees) of the cone in front of the robot that
     * will be used for detecting tags
     */
    public static final double DEFAULT_DETECTION_ANGLE = 120.0;

    /**
     * This is the maximum distance away (in feet) a tag can be from the robot
     * for us to consider it visible
     */
    public static final double DEFAULT_DETECTION_DISTANCE = 6.0;

    /**
     * We add a little "noise" to the LL pose estimates, so we can tell them
     * apart from the simulated odometry position of the robot; this is how
     * far we tx the simulated position from the robot's actual pose
     */
    public static final double DEFAULT_POSE_ERROR = Units.inchesToMeters(4.0);

    final DoubleArrayPublisher classicPublisher;
    final DoubleArrayPublisher megaTagPublisher;
    final DoublePublisher taPublisher;
    final DoublePublisher txPublisher;
    final DoublePublisher tidPublisher;
    final Supplier<Pose2d> poseSupplier;
    double detectionDistance;
    double detectionAngle;
    double poseError;

    public LimelightSim(Supplier<Pose2d> poseSupplier) {

        this.poseSupplier = poseSupplier;
        this.detectionDistance = DEFAULT_DETECTION_DISTANCE;
        this.detectionAngle = DEFAULT_DETECTION_ANGLE;
        this.poseError = DEFAULT_POSE_ERROR;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        classicPublisher = table.getDoubleArrayTopic(classicPoseKey).publish();
        megaTagPublisher = table.getDoubleArrayTopic(megaTagPoseKey).publish();
        tidPublisher = table.getDoubleTopic("tid").publish();
        taPublisher = table.getDoubleTopic("ta").publish();
        txPublisher = table.getDoubleTopic("tx").publish();

        SmartDashboard.putData("LimelightSim", builder -> {
            builder.addDoubleProperty("DetectionAngle", () -> detectionAngle, val -> detectionAngle = val);
            builder.addDoubleProperty("DetectionRadius", () -> detectionDistance, val -> detectionDistance = val);
            builder.addDoubleProperty("PoseError", () -> poseError, val -> poseError = val);
        });
    }

    /**
     * Determines whether a tag is in "view" of the robot
     */
    private boolean tagIsInView(AprilTag tag) {

        Pose2d currentPose = poseSupplier.get();
        Pose2d tagPose = tag.pose.toPose2d();;

        // if the tag is too far away, forget about it
        if (Util.feetBetween(currentPose, tagPose) > detectionDistance) {
            return false;
        }

        // if the tag falls within the cone of visibility in front of the
        // robot we'll accept it
        double halfAngle = detectionAngle / 2.0;
        double degrees = tagPose.relativeTo(currentPose)
                .getTranslation()
                .getAngle()
                .getDegrees();
        return degrees < halfAngle && degrees > -halfAngle;
    }

    /**
     * Called every frame to determine whether we can see a tag, and supply
     * targeting data for it
     */
    public void updateFakePoses() {

        Pose2d currentPose = poseSupplier.get();

        // get the closest tag and compute the distance to it
        AprilTag tag = Util.getClosestAprilTagMatching(currentPose, this::tagIsInView);
        Pose2d tagPose = tag == null
                ? Util.NAN_POSE
                : tag.pose.toPose2d();

        // if there is no tag in view, we we won't publish any of the fake
        // LL information
        double distanceToTag = tag == null
                ? Double.POSITIVE_INFINITY
                : Util.feetBetween(tagPose, currentPose);
        if (tag == null || distanceToTag > detectionDistance) {
            classicPublisher.accept(NO_TAG);
            megaTagPublisher.accept(NO_TAG);
            txPublisher.accept(0.0);
            taPublisher.accept(0.0);
            tidPublisher.accept(0.0);
            return;
        }

        Translation2d heading = tagPose
                .relativeTo(currentPose)
                .getTranslation();

//        double tagArea = 1.0 - (distanceToTag / detectionDistance);
//        double tagOffset = TAG_POSITION.getX() - currentPose.getX();

        double tagArea = 1.0 / heading.getX();
        double tagOffset = heading.getY();

        // generate a fake pose estimate for the classic algorithm

        double [] classicPose = generateFakePoseInfo(
            currentPose.getTranslation(),
            currentPose.getRotation(),
            tag,
            distanceToTag,
            tagArea);
        classicPublisher.accept(classicPose);

        // we will report the mega tag algorithm with the same details,
        // but its error going in the opposite direction, to differentiate
        // it from the mega tag pose

        double [] megaTagPose = Arrays.copyOf(classicPose, classicPose.length);
        megaTagPose[0] -= 2.0 * poseError;
        megaTagPose[1] -= 2.0 * poseError;
        megaTagPublisher.accept(megaTagPose);

        // and finally, we'll update the basic targeting info
        txPublisher.accept(tagOffset);
        taPublisher.accept(tagArea);
        tidPublisher.accept(tag.ID);

    }

    /*
     * This creates fake bot pose information for the robot, in the format that
     * the Limelight would generate it.
     */
    private double [] generateFakePoseInfo(
                    Translation2d robotPosition, 
                    Rotation2d robotHeading,
                    AprilTag tag,
                    double distanceToTag,
                    double tagArea) {

        return new double [] {

            // first 3 = translation (X, Y, Z) in meters

            // the LL is calculating where it "thinks" the robot is by using 
            // vision recognition and AprilTag information. this is pretty good,
            // but always a little bit off; we'll simulate that by using a small
            // tx from the robot's current pose. and we'll assume the robot
            // isn't going to leave the ground.
            robotPosition.getX() + poseError,
            robotPosition.getY() + poseError,
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
            tagArea, // average tag ta

            tag.ID, // tag ID
            1.0, // horizontal tx to primary pixel
            1.0, // vertical tx to primary pixel
            tagArea, // tag ta
            distanceToTag, // distance to camera
            distanceToTag, // distance to robot
            0.3  // ambiguity

        };
    }
}
