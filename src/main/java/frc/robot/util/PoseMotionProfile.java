package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.MotionProfile.State;

public class PoseMotionProfile {

    public record Output(ChassisSpeeds speeds, Pose2d pose) {

    }

    MotionProfile translate;
    MotionProfile rotate;
    Pose2d startPose;
    double cos;
    double sin;

    public PoseMotionProfile() {
        translate = new MotionProfile();
        rotate = new MotionProfile();
    }

    public void resetTranslationConstraints(double maxVelocity, double maxAcceleration, double rampTime) {
        translate.resetConstraints(maxVelocity, maxAcceleration, rampTime);
    }

    public void resetRotationConstraints(double maxVelocity, double maxAcceleration, double rampTime) {
        rotate.resetConstraints(maxVelocity, maxAcceleration, rampTime);
    }

    public void resetMotion(Pose2d startPose, Pose2d finalPose) {

        double distance = startPose.getTranslation().getDistance(finalPose.getTranslation());
        translate.resetMotion(0.0, 0.0, distance);

        double degrees = finalPose.getRotation().minus(startPose.getRotation()).getDegrees();
        rotate.resetMotion(0.0, 0.0, degrees);

        Rotation2d angle = finalPose.relativeTo(startPose).getTranslation().getAngle();
        cos = angle.getCos();
        sin = angle.getSin();

        this.startPose = startPose;
    }

    public Output sample(double time) {

        State stateT = translate.sample(time);
        State stateR = rotate.sample(time);

        ChassisSpeeds speeds = new ChassisSpeeds(
                stateT.velocity() * cos,
                stateT.velocity() * sin,
                Math.toRadians(stateR.velocity()));

        Transform2d transform = new Transform2d(
                stateT.position() * cos,
                stateT.position() * sin,
                Rotation2d.fromDegrees(stateR.position()));

        return new Output(speeds, startPose.transformBy(transform));
    }




}
