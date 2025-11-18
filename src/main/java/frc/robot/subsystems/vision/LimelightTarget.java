package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Util;

/**
 * Represents the targeting output of the limelight
 *
 * @param id ID of the in-view tag (-1 if there isn't one)
 * @param pose pose on the field of the in-view tag (null if there isn't one)
 * @param offset TX value from Limelight
 * @param area TA value from Limelight
 */
public record LimelightTarget(int id, Pose2d pose, double offset, double area) {

    public static final LimelightTarget NO_TARGET = new LimelightTarget(-1, Util.NAN_POSE, Double.NaN, Double.NaN);

}
