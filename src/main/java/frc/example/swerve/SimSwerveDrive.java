package frc.example.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.example.swerve.SwerveConfig.kinematics;

public class SimSwerveDrive implements SwerveChassis {

    double heading;
    double [] velocity;
    double [] angle;
    double [] distance;

    public SimSwerveDrive() {
        velocity = new double[4];
        angle = new double[4];
        distance = new double[4];
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
    public SwerveModuleState [] getModuleStates() {
        SwerveModuleState [] states = new SwerveModuleState[4];
        for (int i=0; i<4; i++) {
            states[i] = new SwerveModuleState(velocity[i], Rotation2d.fromRadians(angle[i]));
        }
        return states;
    }

    @Override
    public void setModuleStates(SwerveModuleState [] states) {
        for (int i=0; i<4; i++) {
            Rotation2d currentAngle = Rotation2d.fromRadians(angle[i]);
            optimizeModuleState(states[i], currentAngle);
            velocity[i] = states[i].speedMetersPerSecond;
            angle[i] = states[i].angle.getRadians();
            distance[i] += states[i].speedMetersPerSecond * 0.02; // Assuming 50Hz update rate
        }
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        heading = MathUtil.angleModulus(heading + speeds.omegaRadiansPerSecond * 0.02);
    }
}
