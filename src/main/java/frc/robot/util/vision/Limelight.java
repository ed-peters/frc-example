package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Util;
import frc.robot.util.vision.LimelightEstimate.Status;
import frc.robot.util.vision.LimelightHelpers.PoseEstimate;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static frc.robot.util.vision.LimelightEstimate.Status.SPINNING;
import static frc.robot.util.vision.LimelightEstimate.Status.SUCCESS;
import static frc.robot.util.vision.LimelightEstimate.Status.TOO_AMBIGUOUS;
import static frc.robot.util.vision.LimelightEstimate.Status.TOO_FAR;

/**
 * Wrapper for a Limelight camera. Provides access to targeting and pose
 * estimation data, as well as a way to change pipelines.
 */
public class Limelight {

    /** Represents one of Limelights different LED modes */
    public enum LEDMode {

        /** Use whichever mode is configured in the pipeline on the limelight */
        PIPELINE_CONTROL,

        /** Override the pipeline setting and turn the LED off */
        FORCE_OFF,

        /** Override the pipeline setting and make the LED blink */
        FORCE_BLINK,

        /** Override the pipeline setting and turn the LED on */
        FORCE_ON,

        /**
         * Indicates an error at runtime where we couldn't understand the
         * value for LED mode returned from the Limelight
         */
        UNKNOWN
    }

    final String limelightName;
    double previousYaw;

    /**
     * Creates a {@link Limelight}
     * @param limelightName the name of the limelight in NetworkTables
     */
    public Limelight(String limelightName) {
        this.limelightName = limelightName;
        this.previousYaw = Double.NaN;
    }

    /**
     * @return the current pipeline index (0-9)
     */
    public int getCurrentPipelineIndex() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    /**
     * @return the current pipeline type
     */
    public String getCurrentPipelineType() {
        return LimelightHelpers.getCurrentPipelineType(limelightName);
    }

    /**
     * Set the current pipeline
     * @param pipelineIndex the desired pipeline (0-9)
     */
    public void switchToPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    /**
     * @return the current LED mode
     */
    public LEDMode getLedMode() {
        int p = (int) LimelightHelpers.getLEDMode(limelightName);
        for (LEDMode mode : LEDMode.values()) {
            if (mode.ordinal() == p) {
                return mode;
            }
        }
        return LEDMode.UNKNOWN;
    }

    /**
     * Set the current LED mode for the limelight
     * @param mode the target LED mode (if null or {@link LEDMode#UNKNOWN},
     *             the command does nothing)
     */
    public void setLedMode(LEDMode mode) {
        if (mode != null && mode != LEDMode.UNKNOWN) {
            LimelightHelpers.setLEDMode(limelightName, mode.ordinal());
        }
    }

    /**
     * @param id an AprilTag ID
     * @return true if the specified target is in view of the Limelight
     */
    public boolean isTagInView(int id) {
        int tid = (int) LimelightHelpers.getFiducialID(limelightName);
        return tid == id;
    }

    /**
     * @return information about the currently-recognized target (note that, if
     * this represents a valid target, it may not be an AprilTag depending on
     * the selected pipeline)
     */
    public LimelightTarget getTarget() {
        if (LimelightHelpers.getTV(limelightName) != 1.0) {
            return LimelightTarget.NO_TARGET;
        }
        return LimelightTarget.fromRaw(
                (int) LimelightHelpers.getFiducialID(limelightName),
                LimelightHelpers.getTA(limelightName),
                LimelightHelpers.getTX(limelightName));
    }

    /**
     * @param maxAmbiguity supplies the maximum allowable ambiguity
     * @param maxDistance supplies the maximum allowable distance
     * @return the robot pose estimate using the classic algorithm (this will
     * never be null, but might not be a valid estimate; consult
     * {@link LimelightEstimate#isValid()}
     */
    public LimelightEstimate getEstimateClassic(DoubleSupplier maxAmbiguity,
                                                DoubleSupplier maxDistance) {

        // get an estimate; if there isn't one, or we don't have exactly one
        // item in view, or it's not a recognized AprilTag, we'll ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null
                || estimate.tagCount != 1
                || estimate.rawFiducials.length != 1) {
            return LimelightEstimate.NO_TAG;
        }

        double ambiguity = estimate.rawFiducials[0].ambiguity;
        double distance = estimate.rawFiducials[0].distToCamera;
        Status status = SUCCESS;

        // we'll also ignore the estimate if it's too ambiguous or too far
        // away from the robot; if neither of those is true, we have a good
        // estimate and we'll submit it to the estimate consumer.
        if (ambiguity > maxAmbiguity.getAsDouble()) {
            status = TOO_AMBIGUOUS;
        } else if (distance > maxDistance.getAsDouble()) {
            status = TOO_FAR;
        }

        return new LimelightEstimate(status,
                ambiguity,
                distance,
                estimate.avgTagArea,
                Double.NaN,
                estimate);
    }

    /**
     * @param minArea minimum allowable target area
     * @param maxYawRate maximum allowable robot yaw rate
     * @param currentHeading the current robot heading
     * @return the robot pose estimate using the MegaTag2 algorithm (this will
     * never be null, but might not be a valid estimate; consult
     * {@link LimelightEstimate#isValid()}
     */
    public LimelightEstimate getEstimateMegaTag2(DoubleSupplier minArea,
                                                 DoubleSupplier maxYawRate,
                                                 Rotation2d currentHeading) {

        double currentYaw = currentHeading.getDegrees();

        // if this is the very first time we're running, we don't have a
        // previous yaw and can't calculate the yaw rate; in this case, we will
        // just pretend there's no tag for one scheduler cycle
        if (Double.isNaN(previousYaw)) {
            previousYaw = currentYaw;
            return LimelightEstimate.NO_TAG;
        }

        double currentYawRate = (currentYaw - previousYaw) / Util.DT;

        // MegaTag2 wants to know our heading for its calculations; we will
        // assume the robot is not pitching or yawing (you may want to change
        // this if you have a top-heavy robot)
        LimelightHelpers.SetRobotOrientation(limelightName, currentYaw, currentYawRate, 0.0, 0.0, 0.0, 0.0);

        // get an estimate; if there isn't one, or we don't have exactly one
        // item in view, or it's not a recognized AprilTag, we'll ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null
                || estimate.tagCount != 1
                || estimate.rawFiducials.length != 1) {
            return LimelightEstimate.NO_TAG;
        }

        double area = estimate.avgTagArea;
        Status status = SUCCESS;

        // if we're spinning around too fast, or the tag is too small (meaning
        // it's too far away), we will ignore it; otherwise we're successful
        if (currentYawRate > maxYawRate.getAsDouble()) {
            status = SPINNING;
        } else if (area < minArea.getAsDouble()) {
            status = TOO_FAR;
        }

        return new LimelightEstimate(status,
                estimate.rawFiducials[0].ambiguity,
                estimate.rawFiducials[0].distToCamera,
                estimate.avgTagArea,
                currentYawRate,
                estimate);
    }
}
