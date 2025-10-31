package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents an in-view AprilTag
 */
public record AprilTarget(int id, Pose2d pose, double offset, double area) {

    public static final AprilTarget NO_TARGET = new AprilTarget(-1, null, Double.NaN, Double.NaN);

}
