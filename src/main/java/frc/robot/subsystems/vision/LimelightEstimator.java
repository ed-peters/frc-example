package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Util;

import static frc.robot.subsystems.vision.LimelightConfig.classicMaxAmbiguity;
import static frc.robot.subsystems.vision.LimelightConfig.classicMaxDistance;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMinArea;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMaxYawRate;
import static frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import static frc.robot.subsystems.vision.LimelightConfig.limelightName;

/**
 * Helper methods for using Limelight pose estimation and targeting.</p>
 *
 * Pose estimation is based on sample code from vendor documentation
 * {@link <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">here</a>.
 * We can supply poses using either the classic or MegaTag2 algorithms.
 * It's meant to be used with {@link frc.robot.subsystems.swerve.SwerveDriveSubsystem#addVisionPose(Pose2d, Matrix, double)}
 * </p>
 * 
 * Targeting is based on the in-view tag's ID and camera offset/area.
 * It's meant to be used with {@link frc.robot.commands.swerve.SwerveTargetAprilTagCommand}
 * </p>
 */
public class LimelightEstimator {

    enum Status {
        NO_STATUS,
        NULL_ESTIMATE,
        NO_TAG,
        TOO_AMBIGUOUS,
        TOO_FAR,
        SPINNING,
        SUCCESS
    }

    Status lastClassicStatus;
    double lastClassicAmbiguity;
    double lastClassicDistance;

    Status lastMegaTagStatus;
    double lastMegaTagYaw;
    double lastMegaTagYawRate;
    double lastMegaTagArea;

    public LimelightEstimator() {

        this.lastClassicStatus = Status.NO_STATUS;
        this.lastClassicAmbiguity = Double.NaN;
        this.lastClassicDistance = Double.NaN;

        this.lastMegaTagStatus = Status.NO_STATUS;
        this.lastMegaTagArea = Double.NaN;

        SmartDashboard.putData("LimelightEstimator", builder -> {
            builder.addDoubleProperty("PoseClassic/Ambiguity", () -> lastClassicAmbiguity, null);
            builder.addDoubleProperty("PoseClassic/Distance", () -> lastClassicDistance, null);
            builder.addStringProperty("PoseClassic/Status", () -> lastClassicStatus.toString(), null);
            builder.addDoubleProperty("PoseMegaTag2/Area", () -> lastMegaTagArea, null);
            builder.addDoubleProperty("PoseMegaTag2/Yaw", () -> lastMegaTagYaw, null);
            builder.addDoubleProperty("PoseMegaTag2/YawRate", () -> lastMegaTagYawRate, null);
            builder.addStringProperty("PoseMegaTag2/Status", () -> lastMegaTagStatus.toString(), null);
            builder.addDoubleProperty("Target/tx", () -> LimelightHelpers.getTX(limelightName), null);
            builder.addDoubleProperty("Target/ta", () -> LimelightHelpers.getTA(limelightName), null);
            builder.addDoubleProperty("Target/tid", () -> LimelightHelpers.getFiducialID(limelightName), null);
        });
    }

    /**
     * @return the latest pose estimate from the limelight using its
     * classic targeting algorithm
     */
    public PoseEstimate updateEstimateClassic() {

        lastClassicStatus = Status.NO_STATUS;
        lastClassicAmbiguity = Double.NaN;
        lastClassicDistance = Double.NaN;

        // Get an estimate; if there's no estimate, or no tag in view, we're done
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null) {
            lastClassicStatus = Status.NULL_ESTIMATE;
            return null;
        }
        else if (estimate.tagCount == 0){
            lastClassicStatus = Status.NO_TAG;
            return null;
        }

        // if there's a single tag, check the ambiguity and distance thresholds
        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {

            lastClassicAmbiguity = estimate.rawFiducials[0].ambiguity;
            lastClassicDistance = estimate.rawFiducials[0].distToCamera;

            if (lastClassicAmbiguity > classicMaxAmbiguity.getAsDouble()) {
                lastClassicStatus = Status.TOO_AMBIGUOUS;
                return null;
            }

            if (lastClassicDistance > classicMaxDistance.getAsDouble()) {
                lastClassicStatus = Status.TOO_FAR;
                return null;
            }

        }

        lastClassicStatus = Status.SUCCESS;

        Util.publishPose("LimelightClassic", estimate.pose);
        return estimate;
    }

    /**
     * @return the latest pose estimate from the limelight using its
     * MegaTag2 algorithm
     */
    public PoseEstimate updateEstimateMegaTag2(
            double yawDegrees,
            double yawRateDegreesPerSecond) {

        lastMegaTagStatus = Status.NO_STATUS;
        lastMegaTagYaw = yawDegrees;
        lastMegaTagYawRate = yawRateDegreesPerSecond;
        lastMegaTagArea = Double.NaN;

        // if we're spinning around too fast, LL estimates get wacky
        if (yawRateDegreesPerSecond > megaTagMaxYawRate.getAsDouble()) {
            lastMegaTagStatus = Status.TOO_FAR;
            return null;
        }

        // MegaTag2 wants to know our heading for its calculations
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get an estimate; if there's no estimate, or no tag in view, we're done
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null || estimate.tagCount == 0) {
            lastMegaTagStatus = Status.NO_TAG;
            return null;
        }

        lastMegaTagArea = estimate.avgTagArea;
        if (lastMegaTagArea < megaTagMinArea.getAsDouble()) {
            lastMegaTagStatus = Status.TOO_FAR;
            return null;
        }

        lastMegaTagStatus = Status.SUCCESS;

        Util.publishPose("LimelightMegaTag2", estimate.pose);
        return estimate;
    }

    /**
     * @return information about the current in-view tag (null if there
     * is no target in view)
     */
    public LimelightTarget getCurrentTarget() {

        int id = (int) LimelightHelpers.getFiducialID(limelightName);
        if (id < 0) {
            return null;
        }

        Pose2d pose = Util.getAprilTagPose(id);
        double offset = LimelightHelpers.getTX(limelightName);
        double area = LimelightHelpers.getTA(limelightName);
        return new LimelightTarget(id, pose, offset, area);
    }
}
