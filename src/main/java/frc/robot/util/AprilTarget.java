package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents an in-view AprilTag
 */
public record AprilTarget(int id, Pose2d pose, double offset, double area) {

    /**
     * @return does the supplied tag represent a valid target?
     */
    public static boolean isValidTarget(AprilTarget target) {
        return target != null && target.id > 0;
    }
}
