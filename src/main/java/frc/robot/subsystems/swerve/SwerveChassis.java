package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Interface for the hardware of the swerve drive and gyro. This lets us
 * implement the {@link SwerveDriveSubsystem} and related commands without
 * depending on specific hardware.
 */
public interface SwerveChassis {

    /** @return current roll/pitch/yaw as reported by the gyro */
    Rotation3d getGyroRotation();

    /** @return current yaw (rotation around Z) as reported by the gyro */
    default Rotation2d getGyroHeading() {
        return Rotation2d.fromRadians(getGyroRotation().getZ());
    }

    /** @return current yaw rate in degrees per second as reported by the gyro */
    double getYawRate();

    /** @return the current robot speed */
    ChassisSpeeds getCurrentSpeed();

    /** @return the current position (angle and distance travelled) of all four modules */
    SwerveModulePosition [] getModulePositions();

    /** Called every cycle to update module states */
    void setModuleStates(SwerveModuleState [] states);
}
