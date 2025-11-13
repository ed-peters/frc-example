package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;

import static frc.robot.subsystems.vision.LimelightConfig.classicMaxAmbiguity;
import static frc.robot.subsystems.vision.LimelightConfig.classicMaxDistance;
import static frc.robot.subsystems.vision.LimelightConfig.confidenceClassic;
import static frc.robot.subsystems.vision.LimelightConfig.confidenceMegaTag2;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMinArea;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMaxYawRate;
import static frc.robot.subsystems.vision.LimelightConfig.useMegaTag2;
import static frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import static frc.robot.subsystems.vision.LimelightConfig.limelightName;

/**
 * Helper methods for using Limelight pose estimation and targeting.</p>
 *
 * Pose estimation is based on sample code from vendor documentation
 * {@link <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">here</a>.
 * We can supply poses using either the classic or MegaTag2 algorithms.
 * </p>
 * 
 * Targeting is based on the in-view id's ID and camera offset/area.
 * </p>
 */
public class LimelightSubsystem extends SubsystemBase {

    enum Status {
        NO_ESTIMATE,
        NO_TAG,
        TOO_AMBIGUOUS,
        TOO_FAR,
        SPINNING,
        SUCCESS
    }

    final SwerveDriveSubsystem drive;

    Status lastClassicStatus;
    double lastClassicAmbiguity;
    double lastClassicDistance;

    Status lastMegaTagStatus;
    double lastMegaTagYaw;
    double lastMegaTagYawRate;
    double lastMegaTagArea;

    LimelightTarget lastTarget;
    Pose2d lastPose;

    public LimelightSubsystem(SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.lastClassicStatus = Status.NO_ESTIMATE;
        this.lastClassicAmbiguity = Double.NaN;
        this.lastClassicDistance = Double.NaN;
        this.lastMegaTagStatus = Status.NO_ESTIMATE;
        this.lastMegaTagArea = Double.NaN;
        this.lastTarget = LimelightTarget.NO_TARGET;
        this.lastPose = Util.NAN_POSE;

        SmartDashboard.putData("LimelightEstimator", builder -> {
            builder.addDoubleProperty("PoseClassic/Ambiguity", () -> lastClassicAmbiguity, null);
            builder.addDoubleProperty("PoseClassic/Distance", () -> lastClassicDistance, null);
            builder.addStringProperty("PoseClassic/Status", () -> lastClassicStatus.toString(), null);
            builder.addDoubleProperty("PoseMegaTag2/Area", () -> lastMegaTagArea, null);
            builder.addDoubleProperty("PoseMegaTag2/Yaw", () -> lastMegaTagYaw, null);
            builder.addDoubleProperty("PoseMegaTag2/YawRate", () -> lastMegaTagYawRate, null);
            builder.addStringProperty("PoseMegaTag2/Status", () -> lastMegaTagStatus.toString(), null);
            builder.addDoubleProperty("Target/tx", () -> lastTarget.offset(), null);
            builder.addDoubleProperty("Target/ta", () -> lastTarget.area(), null);
            builder.addDoubleProperty("Target/tid", () -> lastTarget.id(), null);
        });
    }

    /** @return information about the current in-view id */
    public LimelightTarget getCurrentTarget() {
        return lastTarget;
    }

    /** @return the current pose (null if there isn't one) */
    public Pose2d getCurrentPose() {
        return lastPose;
    }

    @Override
    public void periodic() {

        // every cycle we update the pose estimate using the selected
        // algorithm. if our frame rate was more than 50 Hz, we could
        // think about doing this in a background thread to get more
        // pose estimates, but that's probably overkill for our level
        // of accuracy right now
        if (useMegaTag2.getAsBoolean()) {
            updateEstimateMegaTag2(drive.getHeading().getDegrees(), drive.getYawRate());
        } else {
            updateEstimateClassic();;
        }

        // we publish the target pose if we have one; if we don't, the
        // last published pose will remain - this helps debugging when
        // we lose contact with targets
        if (lastPose != null) {
            Util.publishPose("LimelightPose", lastPose);
        }

        // see if there's a target in view
        int id = (int) LimelightHelpers.getFiducialID(limelightName);
        if (id > 0) {
            Pose2d pose = Util.getAprilTagPose(id);
            double offset = LimelightHelpers.getTX(limelightName);
            double area = LimelightHelpers.getTA(limelightName);
            lastTarget = new LimelightTarget(id, pose, offset, area);
            Util.publishPose("LimelightTarget", pose);
        } else {
            lastTarget = LimelightTarget.NO_TARGET;
            Util.publishPose("LimelightTarget", Util.NAN_POSE);
        }
    }

    /**
     * Adds the latest pose estimate from the limelight to the swerve drive,
     * using the classic targeting algorithm
     */
    private void updateEstimateClassic() {

        lastClassicStatus = Status.NO_ESTIMATE;
        lastClassicAmbiguity = Double.NaN;
        lastClassicDistance = Double.NaN;
        lastPose = Util.NAN_POSE;

        // get an estimate; if there isn't one, or we don't have exactly
        // one item in view, or it's not a recognized AprilTag, we'll
        // ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null || estimate.tagCount != 1 || estimate.rawFiducials.length != 1) {
            return;
        }

        lastClassicAmbiguity = estimate.rawFiducials[0].ambiguity;
        lastClassicDistance = estimate.rawFiducials[0].distToCamera;

        // if the id is ambiguous, we'll ignore it
        if (lastClassicAmbiguity > classicMaxAmbiguity.getAsDouble()) {
            lastClassicStatus = Status.TOO_AMBIGUOUS;
            return;
        }

        // if the id is too far away, we'll ignore it
        if (lastClassicDistance > classicMaxDistance.getAsDouble()) {
            lastClassicStatus = Status.TOO_FAR;
            return;
        }

        lastClassicStatus = Status.SUCCESS;
        lastPose = estimate.pose;

        drive.addVisionPose(
                estimate.pose,
                estimate.timestampSeconds,
                confidenceClassic);
    }

    /**
     * Add the latest pose estimate from the limelight to the drive using the
     * MegaTag2 algorithm
     */
    private void updateEstimateMegaTag2(
            double yawDegrees,
            double yawRateDegreesPerSecond) {

        lastMegaTagStatus = Status.NO_ESTIMATE;
        lastMegaTagYaw = yawDegrees;
        lastMegaTagYawRate = yawRateDegreesPerSecond;
        lastMegaTagArea = Double.NaN;
        lastPose = Util.NAN_POSE;

        // if we're spinning around too fast, LL estimates get wacky
        if (yawRateDegreesPerSecond > megaTagMaxYawRate.getAsDouble()) {
            lastMegaTagStatus = Status.SPINNING;
            return;
        }

        // MegaTag2 wants to know our heading for its calculations
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0.0, 0.0, 0.0, 0.0, 0.0);

        // get an estimate; if there isn't one, or we don't have exactly
        // one item in view, or it's not a recognized AprilTag, we'll
        // ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.tagCount != 1 || estimate.rawFiducials.length != 1) {
            lastMegaTagStatus = Status.NO_ESTIMATE;
            return;
        }

        // if the id is too small (meaning too far away) we'll ignore it
        lastMegaTagArea = estimate.avgTagArea;
        if (lastMegaTagArea < megaTagMinArea.getAsDouble()) {
            lastMegaTagStatus = Status.TOO_FAR;
            return;
        }

        lastMegaTagStatus = Status.SUCCESS;
        lastPose = estimate.pose;

        drive.addVisionPose(
                estimate.pose,
                estimate.timestampSeconds,
                confidenceMegaTag2);
    }
}
