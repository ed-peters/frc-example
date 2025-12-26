package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.vision.LimelightHelpers.PoseEstimate;

/**
 * Used to capture results from Limelight pose estimation. Depending on the
 * status this may or may not have actual estimate information.
 *
 * @param status status of the estimate
 * @param estimate (might be null if the estimate isn't valid)
 */
public record LimelightEstimate(Status status,
                                double ambiguity,
                                double distance,
                                double area,
                                double yawRate,
                                PoseEstimate estimate) {

    public static final LimelightEstimate NO_TAG = new LimelightEstimate(
            Status.NO_TAG,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            null);

    /**
     * Represents the results of pose estimation. The exact type of failures
     * can vary depending on which algorithm is being used.
     */
    public enum Status {

        /**
         * No tag in view (either algorithm); no estimate will be provided)
         */
        NO_TAG,

        /**
         * Robot is spinning too fast (MT2 only); no estimate will be provided
         */
        SPINNING,

        /**
         * Tag is too ambiguous (classic only); the estimate will still be
         * provided, but you shouldn't use it for navigation)
         */
        TOO_AMBIGUOUS,

        /**
         * Tag is too far away (either algorithm); the estimate will still be
         * provided, but you shouldn't use it for navigation)
         */
        TOO_FAR,

        /**
         * Estimate is good (either algorithm); feel free to use it
         */
        SUCCESS
    }

    /**
     * @return is this considered a "good" estimate (if this is false, the
     * associated pose is probably null and, even if it isn't, you shouldn't
     * use it)
     */
    public boolean isValid() {
        return status == Status.SUCCESS;
    }

    /**
     * @return the area of the tag us to capture the estimate
     */
    public double getTagArea() {
        return estimate.avgTagArea;
    }

    /**
     * @return the pose of the estimate, or null if this is a failed result
     */
    public Pose2d getPose() {
        return estimate == null ? null : estimate.pose;
    }

    /**
     * @return the timestamp of the estimate, or 0 if this is a failed result
     */
    public double getTimestamp() {
        return estimate == null ? 0.0 : estimate.timestampSeconds;
    }
}
