package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Converts {@link ChassisSpeeds} into {@link SwerveModuleState}s that will
 * move the robot in the desired manner. Implements the following features:
 * <ul>
 *
 *     <li>Desaturation scales speeds down so no single wheel is ever turning
 *     faster than the absolute maximum speed (but keep the ratio of
 *     speeds the same so we go in the desired direction)</li>
 *
 *     <li>Optimization minimizes the amount of turning each wheel has to do
 *     by reversing the wheel direction if it means less turning</li>
 *
 *     <li>Cosine compensation reduces the speed of the wheel if it's not yet
 *     pointing in the desired direction, to reduce the amount of "skew"
 *     when changing directions</li>
 *
 *     <li>Movement around a custom center of rotation (this allows your
 *     robot to "orbit" a fixed point such as a target on the field)</li>
 *
 * </ul>
 *
 * See the <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html">WPILib
 * docs</a> for more background on kinematics.
 */
public class SwerveKinematicsCalculator {

    final SwerveDriveKinematics kinematics;
    final Supplier<SwerveModulePosition[]> modulePositionGetter;
    final DoubleSupplier maximumWheelSpeed;
    final BooleanSupplier useCosineCompensation;

    /**
     * Creates a {@link SwerveKinematicsCalculator}
     * @param kinematics the chassis kinematics (required)
     * @param modulePositionGetter a getter for module positions (required)
     * @param maximumWheelSpeed a getter for maximum wheel speed
     * @param useCosineCompensation a getter for whether we should use cosine compensation
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveKinematicsCalculator(SwerveDriveKinematics kinematics,
                                      Supplier<SwerveModulePosition[]> modulePositionGetter,
                                      DoubleSupplier maximumWheelSpeed,
                                      BooleanSupplier useCosineCompensation) {

        // we need kinematics & positions; if there are no properties we'll
        // assume some reasonable defaults
        Objects.requireNonNull(kinematics);
        Objects.requireNonNull(modulePositionGetter);
        if (maximumWheelSpeed == null) {
            maximumWheelSpeed = () -> Double.POSITIVE_INFINITY;
        }
        if (useCosineCompensation == null) {
            useCosineCompensation = () -> false;
        }

        this.kinematics = kinematics;
        this.modulePositionGetter = modulePositionGetter;
        this.maximumWheelSpeed = maximumWheelSpeed;
        this.useCosineCompensation = useCosineCompensation;
    }

    /**
     * @param speeds the desired chassis speeds
     * @return the required swerve module states, calculated with the center
     * of the robot as the center of rotation
     */
    public SwerveModuleState [] calculateStates(ChassisSpeeds speeds) {
        return calculateStates(speeds, Translation2d.kZero);
    }

    /**
     * @param speeds the desired chassis speeds
     * @param centerOfRotation the center of rotation (null means center of robot)
     * @return the required swerve module states, calculated with the supplied
     * center of rotation
     */
    public SwerveModuleState [] calculateStates(ChassisSpeeds speeds,
                                                Translation2d centerOfRotation) {

        // no center of rotation? assume it's the center of the robot
        if (centerOfRotation == null) {
            centerOfRotation = Translation2d.kZero;
        }

        // normal kinematic calculations
        SwerveModuleState [] states = kinematics.toSwerveModuleStates(
                speeds,
                centerOfRotation);

        // desaturate
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                maximumWheelSpeed.getAsDouble());

        SwerveModulePosition [] positions = modulePositionGetter.get();

        // optimize
        for (int i=0; i<states.length; i++) {
            states[i].optimize(positions[i].angle);
        }

        // cosine compensation
        if (useCosineCompensation.getAsBoolean()) {
            for (int i=0; i<states.length; i++) {
                states[i].speedMetersPerSecond *= states[i].angle
                        .minus(positions[i].angle)
                        .getCos();
            }
        }

        return states;
    }
}
