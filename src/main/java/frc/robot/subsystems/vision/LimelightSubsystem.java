package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    /**
     * There are various ways pose estimation could fail; this is used by the
     * methods which do the estimation to show what's going on
     */
    enum EstimateStatus {
        NO_ESTIMATE,
        NO_TAG,
        TOO_AMBIGUOUS,
        TOO_FAR,
        SPINNING,
        SUCCESS
    }

    final Supplier<Pose2d> poseSupplier;
    final VisionEstimateConsumer estimateConsumer;

    double currentHeading;
    double previousHeading;
    double latestSpinRate;
    EstimateStatus latestClassicStatus;
    double latestClassicAmbiguity;
    double latestClassicDistance;
    EstimateStatus latestMegaTagStatus;
    double latestMegaTagArea;
    Pose2d latestPose;
    double latestTimestamp;
    LimelightTarget latestTarget;

    public LimelightSubsystem(Supplier<Pose2d> poseSupplier,
                              VisionEstimateConsumer estimateConsumer) {

        this.poseSupplier = poseSupplier;
        this.estimateConsumer = estimateConsumer;
        this.latestClassicStatus = EstimateStatus.NO_ESTIMATE;
        this.latestClassicAmbiguity = Double.NaN;
        this.latestClassicDistance = Double.NaN;
        this.latestMegaTagStatus = EstimateStatus.NO_ESTIMATE;
        this.latestMegaTagArea = Double.NaN;
        this.latestTarget = LimelightTarget.NO_TARGET;
        this.latestPose = Util.NAN_POSE;

        SmartDashboard.putData("LimelightSubsystem", builder -> {
            builder.addDoubleProperty("PoseClassic/Ambiguity", () -> latestClassicAmbiguity, null);
            builder.addDoubleProperty("PoseClassic/Distance", () -> latestClassicDistance, null);
            builder.addStringProperty("PoseClassic/Status", () -> latestClassicStatus.toString(), null);
            builder.addDoubleProperty("PoseMegaTag2/Area", () -> latestMegaTagArea, null);
            builder.addDoubleProperty("PoseMegaTag2/SpinRate", () -> latestSpinRate, null);
            builder.addStringProperty("PoseMegaTag2/Status", () -> latestMegaTagStatus.toString(), null);
            builder.addDoubleProperty("Target/tx", () -> latestTarget.tx(), null);
            builder.addDoubleProperty("Target/ta", () -> latestTarget.ta(), null);
            builder.addDoubleProperty("Target/tid", () -> latestTarget.id(), null);
        });
    }

    /** @return information about the current in-view id */
    public LimelightTarget getCurrentTarget() {
        return latestTarget.id > 0 ? latestTarget : null;
    }

    /** @return the current pose (null if there isn't one) */
    public Pose2d getCurrentPose() {
        return Double.isFinite(latestPose.getX()) ? latestPose : null;
    }

    /** @return true if the specified tag is in view */
    public boolean isTagInView(AprilTag tag) {
        return latestTarget.id == tag.ID;
    }

    @Override
    public void periodic() {

        // we may need to know the yaw rate (that is, how fast the robot is
        // spinning around). we will calculate that every cycle.
        currentHeading = poseSupplier.get().getRotation().getDegrees();
        if (Double.isFinite(previousHeading)) {
            latestSpinRate = (currentHeading - previousHeading) / Util.DT;
        }
        previousHeading = currentHeading;

        // every cycle we update the pose estimate using the selected
        // algorithm. if our frame rate was more than 50 Hz, we could
        // think about doing this in a background thread to get more
        // pose estimates, but that's probably overkill for our level
        // of accuracy right now
        if (useMegaTag2.getAsBoolean()) {
            updateEstimateMegaTag2();
        } else {
            updateEstimateClassic();;
        }

        // update target information
        updateTargetingInformation();

        // publish pose and tag position information for debugging
        Util.publishPose("LimelightPose", latestPose);
        Util.publishPose("LimelightTarget", latestTarget.pose);
    }

    /**
     * Adds the latest pose estimate from the limelight to the swerve drive,
     * using the classic targeting algorithm
     */
    private void updateEstimateClassic() {

        latestClassicStatus = EstimateStatus.NO_ESTIMATE;
        latestClassicAmbiguity = Double.NaN;
        latestClassicDistance = Double.NaN;
        latestPose = Util.NAN_POSE;

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        // if there is no estimate, or it's not based on a single tag, we
        // will ignore it
        if (estimate == null
                || estimate.tagCount != 1
                || estimate.rawFiducials.length != 1) {
            latestClassicStatus = EstimateStatus.NO_ESTIMATE;
            return;
        }

        latestClassicAmbiguity = estimate.rawFiducials[0].ambiguity;
        latestClassicDistance = estimate.rawFiducials[0].distToCamera;

        // we'll also ignore the estimate if it's too ambiguous or too far
        // away from the robot
        if (latestClassicAmbiguity > classicMaxAmbiguity.getAsDouble()) {
            latestClassicStatus = EstimateStatus.TOO_AMBIGUOUS;
            return;
        } else if (latestClassicDistance > classicMaxDistance.getAsDouble()) {
            latestClassicStatus = EstimateStatus.TOO_FAR;
            return;
        }

        // otherwise, we're successful!
        latestClassicStatus = EstimateStatus.SUCCESS;
        latestPose = estimate.pose;
        latestTimestamp = estimate.timestampSeconds;
        estimateConsumer.accept(
                estimate.pose,
                estimate.timestampSeconds,
                confidenceClassic);
    }

    /**
     * Add the latest pose estimate from the limelight to the drive using the
     * MegaTag2 algorithm
     */
    private void updateEstimateMegaTag2() {

        latestMegaTagStatus = EstimateStatus.NO_ESTIMATE;
        latestMegaTagArea = Double.NaN;
        latestPose = Util.NAN_POSE;

        // if we're spinning around too fast, LL estimates get wacky
        if (latestSpinRate > megaTagMaxSpinRate.getAsDouble()) {
            latestMegaTagStatus = EstimateStatus.SPINNING;
            return;
        }

        // MegaTag2 wants to know our heading for its calculations
        LimelightHelpers.SetRobotOrientation(limelightName, latestSpinRate, 0.0, 0.0, 0.0, 0.0, 0.0);

        // get an estimate; if there isn't one, or we don't have exactly
        // one item in view, or it's not a recognized AprilTag, we'll
        // ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.tagCount != 1 || estimate.rawFiducials.length != 1) {
            latestMegaTagStatus = EstimateStatus.NO_ESTIMATE;
            return;
        }

        // if the id is too small (meaning too far away) we'll ignore it
        latestMegaTagArea = estimate.avgTagArea;
        if (latestMegaTagArea < megaTagMinArea.getAsDouble()) {
            latestMegaTagStatus = EstimateStatus.TOO_FAR;
            return;
        }

        // otherwise, we're successful!
        latestMegaTagStatus = EstimateStatus.SUCCESS;
        latestPose = estimate.pose;
        latestTimestamp = estimate.timestampSeconds;
        estimateConsumer.accept(
                estimate.pose,
                estimate.timestampSeconds,
                confidenceMegaTag2);
    }

    /**
     * Uses the Limelight "raw" tag API to calculate a target position
     */
    private void updateTargetingInformation() {
        int id = (int) LimelightHelpers.getFiducialID(limelightName);
        if (id > 0) {
            Pose2d pose = Util.getAprilTagPose(id);
            double offset = LimelightHelpers.getTX(limelightName);
            double area = LimelightHelpers.getTA(limelightName);
            latestTarget = new LimelightTarget(id, pose, offset, area);
        } else {
            latestTarget = LimelightTarget.NO_TARGET;
        }
    }

    /**
     * The Limelight docs lie, and the target pose is returned with the
     * axes wrong. It actually comes back in "camera space" (+Z is straight
     * ahead, +X is right and +Y is down)
     */
    @SuppressWarnings("unused")
    private Pose2d getCorrectedTargetPoseInRobotSpace() {

        Pose3d tagPose3 = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

        // if it doesn't have a tag it will pretend there's one at (0,0,0);
        // let's try to ignore that if possible
        if (tagPose3.getTranslation().getNorm() == 0.0) {
            return null;
        }

        // change the coordinate systems for the lying liars
        return new Pose2d(
                tagPose3.getZ(),
                -tagPose3.getX(),
                Rotation2d.fromRadians(tagPose3.getRotation().getY()));
    }
}
