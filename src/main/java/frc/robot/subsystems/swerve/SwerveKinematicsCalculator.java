package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.subsystems.swerve.SwerveConfig.cosineCompensation;
import static frc.robot.subsystems.swerve.SwerveConfig.kinematics;
import static frc.robot.subsystems.swerve.SwerveConfig.maximumWheelSpeed;

/**
 * Converts {@link ChassisSpeeds} into {@link SwerveModuleState}s that will
 * move the robot in the desired manner. Implements optimization and cosine
 * compensation, as well as being able to rotate around a custom point. See
 * the <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html">WPILib
 * docs</a> for more background on kinematics.
 */
public class SwerveKinematicsCalculator {

    final SwerveChassis chassis;

    public SwerveKinematicsCalculator(SwerveChassis chassis) {
        this.chassis = chassis;
    }

    public SwerveModuleState [] calculateStates(ChassisSpeeds speeds,
                                                Translation2d centerOfRotation) {

        // perform the normal kinematic calculations to determine the desired
        // speed and angle for each module
        SwerveModuleState [] states = kinematics.toSwerveModuleStates(
                speeds,
                centerOfRotation);

        // this will scale speeds down so no single wheel is ever turning
        // faster than the absolute maximum speed (but keep the ratio of
        // speeds the same so we go in the desired direction)
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                maximumWheelSpeed.getAsDouble());

        SwerveModulePosition [] positions = chassis.getModulePositions();

        // optimization minimizes the amount of turning each wheel has to do
        // by reversing the wheel direction if it means less turning
        for (int i=0; i<states.length; i++) {
            states[i].optimize(positions[i].angle);
        }

        // cosine compensation reduces the speed of the wheel if it's not yet
        // pointing in the desired direction, to reduce the amount of "skew"
        // when changing directions
        if (cosineCompensation.getAsBoolean()) {
            for (int i=0; i<states.length; i++) {
                states[i].speedMetersPerSecond *= states[i].angle
                        .minus(positions[i].angle)
                        .getCos();
            }
        }

        return states;
    }
}
