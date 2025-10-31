package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Dash;
import frc.robot.util.PDController;
import frc.robot.util.ProfiledPDController;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.List;

import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryAcceleration;
import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryMaxRotate;
import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryMaxTranslate;
import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryRotateD;
import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryRotateP;
import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryTranslateD;
import static frc.robot.commands.swerve.SwerveAlignConfig.trajectoryTranslateP;

/**
 * Moves the robot a fixed distance from its starting pose based
 * on the WPILib {@link Trajectory} capability.
 */
public class SwerveAlignTrajectoryCommand extends Command {

    public static final String KEY_POSE_NEXT = "SwerveDrive/Structs/PoseTrajectoryNext";
    public static final String KEY_POSE_FINAL = "SwerveDrive/Structs/PoseTrajectoryFinal";

    final SwerveDriveSubsystem drive;
    final Translation2d offset;
    final PDController pidX;
    final PDController pidY;
    final ProfiledPDController pidOmega;
    final HolonomicDriveController controller;
    final Timer timer;
    Pose2d startPose;
    Pose2d finalPose;
    Trajectory trajectory;

    public SwerveAlignTrajectoryCommand(SwerveDriveSubsystem drive,
                                        Translation2d offset) {

        this.drive = drive;
        this.offset = offset;
        this.pidX = new PDController(trajectoryTranslateP, trajectoryTranslateD, null, null);
        this.pidY = new PDController(trajectoryTranslateP, trajectoryTranslateD, null, null);
        this.pidOmega = ProfiledPDController.rotationController(trajectoryRotateP, trajectoryRotateD, trajectoryMaxRotate, trajectoryAcceleration);
        this.controller = new HolonomicDriveController(pidX, pidY, pidOmega);
        this.timer = new Timer();

        addRequirements(drive);
    }

    /**
     * In normal operation, there are probably going to be a bunch of
     * instances of this command, so we won't clutter the dashboard with
     * them all; instead, this will let you register them under different
     * names e.g. for testing
     */
    public void addToDash(String name) {
        SmartDashboard.putData(name, builder -> {
            builder.addDoubleProperty("ErrorX", pidX::getError, null);
            builder.addDoubleProperty("ErrorY", pidY::getError, null);
            builder.addDoubleProperty("ErrorOmega", pidOmega::getPositionError, null);
            builder.addDoubleProperty("TimeCurrent", timer::get, null);
            builder.addDoubleProperty("TimeTotal", trajectory::getTotalTimeSeconds, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {

        // calculate start and end poses
        startPose = drive.getFusedPose();
        finalPose = new Pose2d(
                startPose.getTranslation().plus(offset),
                startPose.getRotation());
        Dash.publish(KEY_POSE_FINAL, finalPose);

        Util.log("[align-trajectory] calculating trajectory to %s", finalPose);

        // create constraints for the trajectory
        double maxV = trajectoryMaxTranslate.getAsDouble();
        double maxA = maxV * trajectoryAcceleration.getAsDouble();
        TrajectoryConfig config = new TrajectoryConfig(maxA, maxV);
        config.setKinematics(drive.getKinematics());

        // calculate a midpoint for the trajectory
        Translation2d midpoint = new Translation2d(
                (startPose.getX() + finalPose.getX()) / 2.0,
                (startPose.getY() + finalPose.getY()) / 2.0);

        // calculate the trajectory itself
        trajectory =  TrajectoryGenerator.generateTrajectory(
                startPose,
                List.of(midpoint),
                finalPose,
                config);

        // reset PID controllers
        pidX.reset();
        pidY.reset();
        pidOmega.reset();

        // start the clock going
        timer.restart();
    }

    @Override
    public void execute() {

        // use the trajectory to calculate the next desired pose
        State nextState = trajectory.sample(timer.get());
        Dash.publish(KEY_POSE_NEXT, nextState.poseMeters);

        // use the controller to calculate the speeds required to
        // get us there
        ChassisSpeeds speeds = controller.calculate(
                drive.getFusedPose(),
                nextState,
                finalPose.getRotation());

        // do it!
        drive.drive("align-trajectory", speeds);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {

        // warn about this, in case this command had to be interrupted
        // by a timeout because it never got to the target heading or
        // something like that; that's a sign that it's badly tuned or
        // may need a larger tolerance
        if (!controller.atReference()) {
            Util.log("[align-trajectory] !!! MISSED target pose !!!");
        }

        timer.stop();
    }
}
