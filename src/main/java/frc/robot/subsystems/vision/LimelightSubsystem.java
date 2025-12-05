package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightPoseEstimate.Status;
import frc.robot.util.Util;

import java.util.function.Supplier;

import static frc.robot.subsystems.vision.LimelightConfig.classicMaxAmbiguity;
import static frc.robot.subsystems.vision.LimelightConfig.classicMaxDistance;
import static frc.robot.subsystems.vision.LimelightConfig.confidenceClassic;
import static frc.robot.subsystems.vision.LimelightConfig.confidenceMegaTag2;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMinArea;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMaxSpinRate;
import static frc.robot.subsystems.vision.LimelightConfig.useMegaTag2;
import static frc.robot.subsystems.vision.LimelightConfig.limelightName;
import static frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

/**
 * Subsystem for using Limelight pose estimation and targeting. Pose
 * estimation is based on sample code from vendor documentation
 * {@link <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">here</a>.
 * We can supply poses using either the classic or MegaTag2 algorithms.
 * </p>
 * 
 * Targeting is based on the in-view id's ID and camera tx/ta.
 * </p>
 *
 * This subsystem is implemented so it doesn't depend on a specific swerve
 * drive implementation.</p>
 */
public class LimelightSubsystem extends SubsystemBase {

    /**
     * Represents the targeting output of the limelight. TX and TA are
     * based on the similarly-named properties from the <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api">Limelight
     * API.</a>
     */
    public record LimelightTarget(int id, Pose2d pose, double tx, double ta) {

        public static final LimelightTarget NO_TARGET = new LimelightTarget(-1, Util.NAN_POSE, Double.NaN, Double.NaN);

    }

    final Supplier<Pose2d> poseSupplier;
    final VisionEstimateConsumer estimateConsumer;

    double previousYaw;
    double latestYaw;
    double latestYawRate;
    LimelightPoseEstimate latestPoseEstimate;
    LimelightTarget latestTarget;

    public LimelightSubsystem(Supplier<Pose2d> poseSupplier,
                              VisionEstimateConsumer estimateConsumer) {

        this.previousYaw = Double.NaN;
        this.latestYaw = poseSupplier.get().getRotation().getDegrees();
        this.latestYawRate = Double.NaN;
        this.poseSupplier = poseSupplier;
        this.estimateConsumer = estimateConsumer;
        this.latestTarget = LimelightTarget.NO_TARGET;

        SmartDashboard.putData("LimelightSubsystem", builder -> {
            builder.addStringProperty("PoseEstimate/Algorithm", () -> useMegaTag2.getAsBoolean() ? "megaTag2" : "classic", null);
            builder.addDoubleProperty("PoseEstimate/Ambiguity", () -> latestPoseEstimate.ambiguity, null);
            builder.addDoubleProperty("PoseEstimate/Area", () -> latestPoseEstimate.area, null);
            builder.addDoubleProperty("PoseEstimate/Distance", () -> latestPoseEstimate.distance, null);
            builder.addDoubleProperty("PoseEstimate/SpinRate", () -> latestYawRate, null);
            builder.addStringProperty("PoseEstimate/Status", () -> latestPoseEstimate.status.toString(), null);
            builder.addDoubleProperty("Target/tx", () -> latestTarget.tx(), null);
            builder.addDoubleProperty("Target/ta", () -> latestTarget.ta(), null);
            builder.addDoubleProperty("Target/tid", () -> latestTarget.id(), null);
        });
    }

    /**
     * @return information from the "raw" targeting API, for use in e.g.
     * visual servoing
     */
    public LimelightTarget getCurrentTarget() {
        return latestTarget.id > 0 ? latestTarget : null;
    }

    /**
     * @return the current pose, if there is a successful pose estimate;
     * null otherwise
     */
    public Pose2d getCurrentPose() {
        return latestPoseEstimate.status == Status.SUCCESS
                ? latestPoseEstimate.pose
                : null;
    }

    /**
     * @return true if the specified tag is in view
     */
    public boolean isTagInView(AprilTag tag) {
        return latestTarget.id == tag.ID;
    }

    @Override
    public void periodic() {

        // we may need to know the yaw rate (that is, how fast the robot is
        // spinning around). we will calculate that every cycle.
        latestYaw = poseSupplier.get().getRotation().getDegrees();
        if (Double.isFinite(previousYaw)) {
            latestYawRate = (latestYaw - previousYaw) / Util.DT;
        }
        previousYaw = latestYaw;

        // every cycle we update the pose estimate using the selected
        // algorithm. if our frame rate was more than 50 Hz, we could think
        // about doing this in a background thread to get more pose estimates,
        // but that's probably overkill for our level of accuracy right now
        latestPoseEstimate = useMegaTag2.getAsBoolean()
                ? readEstimateMegaTag2()
                : readEstimateClassic();

        // if we have a good estimate, we want to let the vision system know
        // about it; the "confidence" parameter says how much to trust the
        // different aspects of the estimate (X, Y and heading)
        if (latestPoseEstimate.status == Status.SUCCESS) {
            Vector<N3> confidence = useMegaTag2.getAsBoolean()
                    ? confidenceMegaTag2
                    : confidenceClassic;
            estimateConsumer.accept(
                    latestPoseEstimate.pose,
                    latestPoseEstimate.timestampSeconds,
                    confidence);
        }

        // update target information
        latestTarget = readTargetingInformation();

        // publish pose and tag position information for debugging
        Util.publishPose("LimelightPose", latestPoseEstimate.pose);
        Util.publishPose("LimelightTarget", latestTarget.pose);
    }

    /**
     * Adds the latest pose estimate from the limelight to the swerve drive,
     * using the classic targeting algorithm
     */
    private LimelightPoseEstimate readEstimateClassic() {

        // if there is no estimate, or it's not based on a single tag, we
        // will ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null
                || estimate.tagCount != 1
                || estimate.rawFiducials.length != 1) {
            return new LimelightPoseEstimate("classic", Status.NO_ESTIMATE);
        }

        // we'll also ignore the estimate if it's too ambiguous or too far
        // away from the robot; if neither of those is true, we have a good
        // estimate and we'll submit it to the estimate consumer.
        double ambiguity = estimate.rawFiducials[0].ambiguity;
        double distance = estimate.rawFiducials[0].distToCamera;
        if (ambiguity > classicMaxAmbiguity.getAsDouble()) {
            return new LimelightPoseEstimate(Status.TOO_AMBIGUOUS, ambiguity, distance);
        } else if (distance > classicMaxDistance.getAsDouble()) {
            return new LimelightPoseEstimate(Status.TOO_FAR, ambiguity, distance);
        }

        return new LimelightPoseEstimate(
                ambiguity,
                distance,
                estimate.pose,
                estimate.timestampSeconds);
    }

    /**
     * Add the latest pose estimate from the limelight to the drive using the
     * MegaTag2 algorithm
     */
    private LimelightPoseEstimate readEstimateMegaTag2() {

        // MegaTag2 wants to know our heading for its calculations; we will
        // we will assume the robot is not pitching or yawing (you may want
        // to change this if you have a top-heavy robot)
        LimelightHelpers.SetRobotOrientation(limelightName, latestYaw, latestYawRate, 0.0, 0.0, 0.0, 0.0);

        // if we're spinning around too fast, LL estimates get wacky, so we
        // will ignore them
        if (latestYawRate > megaTagMaxSpinRate.getAsDouble()) {
            return new LimelightPoseEstimate("megaTag2", Status.SPINNING);
        }

        // get an estimate; if there isn't one, or we don't have exactly
        // one item in view, or it's not a recognized AprilTag, we'll
        // ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.tagCount != 1 || estimate.rawFiducials.length != 1) {
            return new LimelightPoseEstimate("megaTag2", Status.NO_ESTIMATE);
        }

        // if the id is too small (meaning too far away) we'll ignore it
        double area = estimate.avgTagArea;
        if (area < megaTagMinArea.getAsDouble()) {
            return new LimelightPoseEstimate(area);
        }

        // otherwise, we're successful!
        return new LimelightPoseEstimate(
                area,
                estimate.pose,
                estimate.timestampSeconds);
    }

    /**
     * @return a {@link LimelightTarget} based on the latest information from
     * the "raw" Limelight API; this will always be non-null, but may be
     * {@link LimelightTarget#NO_TARGET} if there is no valid target
     */
    LimelightTarget readTargetingInformation() {

        // if there's no information about a target, we're done
        int id = (int) LimelightHelpers.getFiducialID(limelightName);
        if (id < 0) {
            return LimelightTarget.NO_TARGET;
        }

        // this would be a very weird condition - if there's a tag in view
        // but no information about it in the field layout, we'll report the
        // error
        Pose3d pose = Util.getFieldLayout().getTagPose(id).orElse(null);
        if (pose == null) {
            Util.log("[ll] !!! NO POSE FOUND FOR TAG %d !!!", id);
            return LimelightTarget.NO_TARGET;
        }

        return new LimelightTarget(
                id,
                pose.toPose2d(),
                LimelightHelpers.getTX(limelightName),
                LimelightHelpers.getTA(limelightName));
    }
}
