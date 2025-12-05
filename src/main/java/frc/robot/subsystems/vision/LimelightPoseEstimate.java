package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Util;

/**
 * Captures information about a pose estimate from the Limelight
 */
public class LimelightPoseEstimate {

    public enum Status {
        NO_ESTIMATE,
        NO_TAG,
        TOO_AMBIGUOUS,
        TOO_FAR,
        SPINNING,
        SUCCESS
    }

    public final String algorithm;
    public final Status status;
    public final double ambiguity;
    public final double distance;
    public final double timestampSeconds;
    public final double area;
    public final Pose2d pose;

    /**
     * Use this when you aren't going to use any of the information about the
     * pose estimate, or there isn't one
     */
    public LimelightPoseEstimate(String algorithm, Status status) {
        this.algorithm = algorithm;
        this.status = status;
        this.ambiguity = Double.NaN;
        this.distance = Double.NaN;
        this.timestampSeconds = Double.NaN;
        this.area = Double.NaN;
        this.pose = Util.NAN_POSE;
    }

    /**
     * Use this for classic estimates that are too ambiguous or too far away
     */
    public LimelightPoseEstimate(Status status, double ambiguity, double distance) {
        this.algorithm = "classic";
        this.status = status;
        this.ambiguity = ambiguity;
        this.distance = distance;
        this.timestampSeconds = Double.NaN;
        this.area = Double.NaN;
        this.pose = Util.NAN_POSE;
    }

    /**
     * Use this for successful classic estimates
     */
    public LimelightPoseEstimate(double ambiguity, double distance, Pose2d pose, double timestampSeconds) {
        this.algorithm = "classic";
        this.status = Status.SUCCESS;
        this.ambiguity = ambiguity;
        this.distance = distance;
        this.timestampSeconds = timestampSeconds;
        this.area = Double.NaN;
        this.pose = pose;
    }

    /**
     * Use this for MegaTag2 estimates that are too far away
     */
    public LimelightPoseEstimate(double area) {
        this.algorithm = "megaTag2";
        this.status = Status.TOO_FAR;
        this.ambiguity = Double.NaN;
        this.distance = Double.NaN;
        this.timestampSeconds = Double.NaN;
        this.area = area;
        this.pose = Util.NAN_POSE;
    }

    /**
     * Use this for successful MegaTag2 estimates
     */
    public LimelightPoseEstimate(double area, Pose2d pose, double timestampSeconds) {
        this.algorithm = "megaTag2";
        this.status = Status.SUCCESS;
        this.ambiguity = Double.NaN;
        this.distance = Double.NaN;
        this.timestampSeconds = timestampSeconds;
        this.area = area;
        this.pose = pose;
    }
}
