package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Util;

import static frc.robot.subsystems.swerve.SwerveConfig.kinematics;

/**
 * Implements the {@link SwerveChassis} interface for a pure software
 * simulation. This lets you run in simulation and test out commands.
 */
public class SwerveChassisSim implements SwerveChassis {

    double heading;
    double [] velocity;
    double [] angle;
    double [] distance;

    public SwerveChassisSim() {
        heading = 0.0;
        velocity = new double[]{ 0.0, 0.0, 0.0, 0.0 };
        angle = new double[]{ 0.0, 0.0, 0.0, 0.0 };
        distance = new double[]{ 0.0, 0.0, 0.0, 0.0 };
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromRadians(heading);
    }

    @Override
    public void resetHeading(Rotation2d newHeading) {
        heading = newHeading.getRadians();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition [] positions = new SwerveModulePosition[4];
        for (int i=0; i<4; i++) {
            positions[i] = new SwerveModulePosition(distance[i], Rotation2d.fromRadians(angle[i]));
        }
        return positions;
    }

    @Override
    public void setModuleStates(SwerveModuleState [] states) {

        for (int i=0; i<4; i++) {

            // optimize the desired state based on the wheel's current heading
            Rotation2d currentAngle = Rotation2d.fromRadians(angle[i]);
            optimizeModuleState(states[i], currentAngle);

            // record current velocity and angle
            velocity[i] = states[i].speedMetersPerSecond;
            angle[i] = states[i].angle.getRadians();

            // calculate how far we've rolled in this unit time
            distance[i] += states[i].speedMetersPerSecond * Util.DT;
        }

        // calculate how much the robot's heading has changed as a result of
        // this motion
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        heading = MathUtil.angleModulus(heading + speeds.omegaRadiansPerSecond * Util.DT);
    }
}
