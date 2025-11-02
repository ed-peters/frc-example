package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.ProfiledPDController;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.commands.swerve.SwerveAlignConfig.pidAcceleration;
import static frc.robot.commands.swerve.SwerveAlignConfig.pidD;
import static frc.robot.commands.swerve.SwerveAlignConfig.pidMaxVelocity;
import static frc.robot.commands.swerve.SwerveAlignConfig.pidP;
import static frc.robot.commands.swerve.SwerveAlignConfig.pidTolerance;

/**
 * Moves the robot a fixed distance from its starting pose using
 * a pair of profiled PID controllers
 */
public class SwerveAlignPidCommand extends Command {

    public static final String KEY_POSE = "AlignPid";

    private final SwerveDriveSubsystem drive;
    private final ProfiledPDController pidX;
    private final ProfiledPDController pidY;
    private final Translation2d offset;
    Pose2d startPose;
    Pose2d finalPose;
    double speedX;
    double speedY;
    boolean doneX;
    boolean doneY;

    public SwerveAlignPidCommand(SwerveDriveSubsystem drive, Translation2d offset) {
        this.drive = drive;
        this.pidX = ProfiledPDController.translationController(pidP, pidD, pidMaxVelocity, pidAcceleration);
        this.pidY = ProfiledPDController.translationController(pidP, pidD, pidMaxVelocity, pidAcceleration);
        this.offset = offset;
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
            builder.addDoubleProperty("ErrorX", pidX::getPositionError, null);
            builder.addDoubleProperty("ErrorY", pidY::getPositionError, null);
            builder.addDoubleProperty("SpeedX", () -> speedX, null);
            builder.addDoubleProperty("SpeedY", () -> speedY, null);
            builder.addBooleanProperty("DoneX?", () -> doneX, null);
            builder.addBooleanProperty("DoneY?", () -> doneY, null);
            builder.addBooleanProperty("Running?", this::isScheduled, null);
        });
    }

    @Override
    public void initialize() {

        // calculate start and end pose and log the end
        startPose = drive.getFusedPose();
        finalPose = new Pose2d(
                startPose.getTranslation().plus(offset),
                startPose.getRotation());

        // reset controller error and configuration
        pidX.reset();
        pidX.setTolerance(pidTolerance.getAsDouble());
        pidY.reset();
        pidY.setTolerance(pidTolerance.getAsDouble());

        // inform the controllers of what their goals are
        pidX.setGoal(finalPose.getX());
        pidY.setGoal(finalPose.getY());
    }

    @Override
    public void execute() {

        // calculate speeds based on current pose and drive there
        Pose2d currentPose = drive.getFusedPose();
        speedX = pidX.calculate(currentPose.getX());
        speedY = pidY.calculate(currentPose.getY());
        drive.drive("align-pid", new ChassisSpeeds(speedX, speedY, 0.0));

        // see if we've made it
        doneX = pidX.atGoal();
        doneY = pidY.atGoal();

        // publish where we're going
        drive.publishPose(KEY_POSE, finalPose);
    }

    @Override
    public boolean isFinished() {
        return doneX && doneY;
    }

    @Override
    public void end(boolean interrupted) {

        // warn about this, in case this command had to be interrupted
        // by a timeout because it never got to the target heading or
        // something like that; that's a sign that it's badly tuned or
        // may need a larger tolerance
        if (!isFinished()) {
            String which = "";
            if (doneX) {
                which = "Y";
            } else if (doneY) {
                which = "X";
            } else {
                which = "X and Y";
            }
            Util.log("[align-pid] !!! FAILED aligning to tag in %s !!!", which);
        }

        drive.publishPose(KEY_POSE, Util.NAN_POSE);
    }
}
