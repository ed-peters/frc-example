package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Util;
import frc.robot.util.vision.LimelightHelpers.PoseEstimate;

import java.util.function.DoubleSupplier;

import static frc.robot.util.vision.LimelightEstimate.Status.NO_TAG;
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
     * @return information about the currently-recognized target (null if there
     * is no target in view); note that this may or may not be an AprilTag
     * depending on the selected pipeline
     */
    public LimelightTarget getTarget() {
        if (LimelightHelpers.getTV(limelightName) != 1.0) {
            return null;
        }
        return new LimelightTarget(
                (int) LimelightHelpers.getFiducialID(limelightName),
                LimelightHelpers.getTA(limelightName),
                LimelightHelpers.getTX(limelightName));
    }

    /**
     * @param maxAmbiguity supplies the maximum allowable ambiguity
     * @param maxDistance supplies the maximum allowable distance
     * @return the robot pose estimate using the classic algorithm (this will
     * never be null, but might not be a valid estimate; consult
     * {@link LimelightEstimate#isFailure()}
     */
    public LimelightEstimate getEstimateClassic(DoubleSupplier maxAmbiguity,
                                                DoubleSupplier maxDistance) {

        // if there is no estimate, or it's not based on a single tag, we
        // will ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (estimate == null
                || estimate.tagCount != 1
                || estimate.rawFiducials.length != 1) {
            return new LimelightEstimate(NO_TAG);
        }

        // we'll also ignore the estimate if it's too ambiguous or too far
        // away from the robot; if neither of those is true, we have a good
        // estimate and we'll submit it to the estimate consumer.
        double ambiguity = estimate.rawFiducials[0].ambiguity;
        double distance = estimate.rawFiducials[0].distToCamera;
        if (ambiguity > maxAmbiguity.getAsDouble()) {
            return new LimelightEstimate(TOO_AMBIGUOUS, estimate);
        } else if (distance > maxDistance.getAsDouble()) {
            return new LimelightEstimate(TOO_FAR, estimate);
        }

        // success!
        return new LimelightEstimate(SUCCESS, estimate);
    }

    /**
     * @param minArea minimum allowable target area
     * @param maxYawRate maximum allowable robot yaw rate
     * @param currentHeading the current robot heading
     * @return the robot pose estimate using the MegaTag2 algorithm (this will
     * never be null, but might not be a valid estimate; consult
     * {@link LimelightEstimate#isFailure()}
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
            return new LimelightEstimate(NO_TAG);
        }

        double currentYawRate = (currentYaw - previousYaw) / Util.DT;

        // MegaTag2 wants to know our heading for its calculations; we will
        // assume the robot is not pitching or yawing (you may want to change
        // this if you have a top-heavy robot)
        LimelightHelpers.SetRobotOrientation(limelightName, currentYaw, currentYawRate, 0.0, 0.0, 0.0, 0.0);

        // if we're spinning around too fast, LL estimates get wacky, so we
        // will ignore them
        if (currentYawRate > maxYawRate.getAsDouble()) {
            return new LimelightEstimate(SPINNING);
        }

        // get an estimate; if there isn't one, or we don't have exactly one
        // item in view, or it's not a recognized AprilTag, we'll ignore it
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.tagCount != 1 || estimate.rawFiducials.length != 1) {
            return new LimelightEstimate(NO_TAG);
        }

        // if the id is too small (meaning too far away) we'll ignore it
        double area = estimate.avgTagArea;
        if (area < minArea.getAsDouble()) {
            return new LimelightEstimate(TOO_FAR, estimate);
        }

        // otherwise, we're successful!
        return new LimelightEstimate(SUCCESS, estimate);
    }
}
