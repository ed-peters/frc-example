package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Dash;
import frc.robot.util.PDController;

import java.util.List;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveConfig.*;

public class DriveTrajectoryCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<Pose2d> poseSupplier;
    final PDController pidX;
    final PDController pidY;
    final Timer timer;
    Trajectory trajectory;
    HolonomicDriveController controller;

    public DriveTrajectoryCommand(SwerveDriveSubsystem drive, Supplier<Pose2d> poseSupplier) {
        this.drive = drive;
        this.poseSupplier = poseSupplier;
        this.timer = new Timer();
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        Pose2d startPose = drive.getFusedPose();
        Pose2d finalPose = poseSupplier.get();
        trajectory = generateTrajectory(startPose, finalPose);
        if (trajectory != null) {
            return;
        }

        // create trajectory generator configuration with maximum
        // velocity, acceleration and kinematics
        double maxV = trajectoryMaxVelocity.getAsDouble();
        double maxA = maxV * trajectoryAccelerationFactor.getAsDouble();
        TrajectoryConfig config = new TrajectoryConfig(maxA, maxV);
        config.setKinematics(kinematics);

        Translation2d midpoint = new Translation2d(
                (startPose.getX() + finalPose.getX()) / 2.0,
                (startPose.getY() + finalPose.getY()) / 2.0);

        // generate a linear trajectory between the two poses
        trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(midpoint),
                finalPose,
                config);
        timer.restart();
    }

    protected Trajectory generateTrajectory(Pose2d startPose, Pose2d finalPose) {

        // if the start pose is too close to the final pose, the trajectory
        // generator will error out; let's skip that here
        double distance = startPose.getTranslation().getDistance(finalPose.getTranslation());
        if (Math.abs(distance) < trajectoryMinDistance.getAsDouble()) {
            Dash.log("[swerve] trajectory final pose %s is too close!", finalPose);
            return null;
        }

        // create trajectory generator configuration with maximum
        // velocity, acceleration and kinematics
        double maxV = trajectoryMaxVelocity.getAsDouble();
        double maxA = maxV * trajectoryAccelerationFactor.getAsDouble();
        TrajectoryConfig config = new TrajectoryConfig(maxA, maxV);
        config.setKinematics(kinematics);

        Translation2d midpoint = new Translation2d(
                (startPose.getX() + finalPose.getX()) / 2.0,
                (startPose.getY() + finalPose.getY()) / 2.0);

        // generate a linear trajectory between the two poses
        return TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(midpoint),
                finalPose,
                config);
    }

    protected HolonomicDriveController createController() {
        PIDController pidX = new PIDController(autoTranslateP.getAsDouble(), 0.0, 0.0);
        PIDController pidY = new PIDController(autoTranslateP.getAsDouble(), 0.0, 0.0);
        Constraints constraints = new Constraints(
                Units.degreesToRadians(autoRotateDps.getAsDouble()),
                Units.degreesToRadians(autoRotateAcceleration.getAsDouble()));
        ProfiledPIDController pidO = new ProfiledPIDController(
                autoRotateP.getAsDouble(), 0.0, 0.0,
                constraints);
        pidO.enableContinuousInput(-Math.PI, Math.PI);
        return new HolonomicDriveController(pidX, pidY, pidO);

    }

    @Override
    public void execute() {

        Pose2d targetPose = trajectory.sample(timer.get()).poseMeters;

    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }
}
