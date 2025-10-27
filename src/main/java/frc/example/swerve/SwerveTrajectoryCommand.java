package frc.example.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.example.Util;

import java.util.List;
import java.util.function.Supplier;

import static frc.example.swerve.SwerveTrajectoryConfig.acceleration;
import static frc.example.swerve.SwerveTrajectoryConfig.maxRotate;
import static frc.example.swerve.SwerveTrajectoryConfig.maxTranslate;
import static frc.example.swerve.SwerveTrajectoryConfig.minDistance;
import static frc.example.swerve.SwerveConfig.kinematics;
import static frc.example.swerve.SwerveTrajectoryConfig.rotateD;
import static frc.example.swerve.SwerveTrajectoryConfig.rotateP;
import static frc.example.swerve.SwerveTrajectoryConfig.translateD;
import static frc.example.swerve.SwerveTrajectoryConfig.translateP;

public class SwerveTrajectoryCommand extends Command {

    final String name;
    final SwerveDriveSubsystem drive;
    final Supplier<Pose2d> poseSupplier;
    final Timer timer;
    Trajectory trajectory;
    HolonomicDriveController controller;

     public SwerveTrajectoryCommand(String name, SwerveDriveSubsystem drive, Supplier<Pose2d> poseSupplier) {

         this.name = name;
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
         double maxV = maxTranslate.getAsDouble();
         double maxA = maxV * acceleration.getAsDouble();
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
         if (Math.abs(distance) < minDistance.getAsDouble()) {
             Util.log("[swerve] trajectory final pose %s is too close!", finalPose);
             return null;
         }

         // create trajectory generator configuration with maximum
         // velocity, acceleration and kinematics
         double maxV = maxTranslate.getAsDouble();
         double maxA = maxV * acceleration.getAsDouble();
         TrajectoryConfig config = new TrajectoryConfig(maxA, maxV);
         config.setKinematics(kinematics);

         // trajectories always want a waypoint, so we'll assume
         // we're driving in a straight line between A and B
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

         PIDController pidX = new PIDController(translateP.getAsDouble(), 0.0, translateD.getAsDouble());
         PIDController pidY = new PIDController(translateP.getAsDouble(), 0.0, translateD.getAsDouble());

         double maxR = Math.toRadians(maxRotate.getAsDouble());
         double maxA = maxR * acceleration.getAsDouble();
         ProfiledPIDController pidO = new ProfiledPIDController(
                 rotateP.getAsDouble(), 0.0, rotateD.getAsDouble(),
                 new Constraints(maxR, maxA));
         pidO.enableContinuousInput(-Math.PI, Math.PI);

         return new HolonomicDriveController(pidX, pidY, pidO);
     }

     @Override
     public void execute() {
         State nextState = trajectory.sample(timer.get());
         Pose2d currentPose = drive.getFusedPose();
         ChassisSpeeds speeds = controller.calculate(
                 currentPose,
                 nextState,
                 currentPose.getRotation());
        drive.drive(name, speeds);
     }

     @Override
     public boolean isFinished() {
         return timer.get() > trajectory.getTotalTimeSeconds();
     }
}
