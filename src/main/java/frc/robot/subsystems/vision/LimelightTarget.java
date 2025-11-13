package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Util;

public record LimelightTarget(int id, Pose2d pose, double offset, double area) {

    public static final LimelightTarget NO_TARGET = new LimelightTarget(-1, Util.NAN_POSE, Double.NaN, Double.NaN);

}
