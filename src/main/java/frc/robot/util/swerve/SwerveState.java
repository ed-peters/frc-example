package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Capture the state
 * @param pose
 * @param speeds
 */
public record SwerveState(Pose2d pose, ChassisSpeeds speeds) {

}
