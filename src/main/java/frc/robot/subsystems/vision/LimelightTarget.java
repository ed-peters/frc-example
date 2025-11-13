package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public record LimelightTarget(int tag, Pose2d pose, double offset, double area) {

}
