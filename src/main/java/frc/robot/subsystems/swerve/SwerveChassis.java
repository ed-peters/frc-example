package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.subsystems.swerve.SwerveConfig.*;

public interface SwerveChassis {

    /** @return the current robot heading */
    Rotation2d getHeading();

    /** Reset the current heading */
    void resetHeading(Rotation2d heading);

    /** @return the current position (angle and distance travelled) of all four modules */
    SwerveModulePosition [] getModulePositions();

    /** @return the current state (angle and speed) of all four modules */
    SwerveModuleState [] getModuleStates();

    /** Called during tuning to directly set drive volts */
    default void applyDriveVolts(double volts) {
        throw new UnsupportedOperationException();
    }

    /** Called every cycle to update module states */
    void setModuleStates(SwerveModuleState [] states);

    /**
     * Updates the supplied module state by (a) optimizing the angle and speed to
     * minimize required turning and (b) optionally applying cosine compensation
     * to limit drift
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
     */
    default void optimizeModuleState(SwerveModuleState state, Rotation2d currentAngle) {
        state.optimize(currentAngle);
        if (cosineCompensation.getAsBoolean()) {
            state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
        }
    }
}
