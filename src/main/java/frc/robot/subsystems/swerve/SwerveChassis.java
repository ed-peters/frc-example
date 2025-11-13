package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.subsystems.swerve.SwerveConfig.cosineCompensation;

/**
 * Interface for the hardware of the swerve drive and gyro. This lets us
 * implement the {@link SwerveDriveSubsystem} and related commands without
 * depending on specific hardware.
 */
public interface SwerveChassis {

    /** @return the current robot heading */
    Rotation2d getHeading();

    /** @return the rate of change of the heading (in degrees per second) */
    double getYawRate();

    /** Reset the current heading */
    void resetHeading(Rotation2d heading);

    /** @return the current position (angle and distance travelled) of all four modules */
    SwerveModulePosition [] getModulePositions();

    /** Called every cycle to update module states */
    void setModuleStates(SwerveModuleState [] states);

    /**
     * Updates the supplied module state by (a) optimizing the angle and speed
     * to minimize required turning and (b) optionally applying cosine
     * compensation to limit drift. Subclasses should call this for each wheel
     * before applying a module state.</p>
     *
     * See <a href=https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization">this page</a>
     * for information about optimization. See <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation">this page</a>
     * for information about cosine compensation.</p>
     */
    default void optimizeModuleState(SwerveModuleState state, Rotation2d currentAngle) {
        state.optimize(currentAngle);
        if (cosineCompensation.getAsBoolean()) {
            state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
        }
    }
}
