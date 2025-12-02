package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightTarget;
import frc.robot.util.Util;

import java.util.function.Consumer;

import static frc.robot.commands.vision.LimelightConfig.enableLogging;
import static frc.robot.commands.vision.LimelightConfig.servoAreaD;
import static frc.robot.commands.vision.LimelightConfig.servoAreaP;
import static frc.robot.commands.vision.LimelightConfig.servoAreaTarget;
import static frc.robot.commands.vision.LimelightConfig.servoAreaTolerance;
import static frc.robot.commands.vision.LimelightConfig.servoMaxFeedback;
import static frc.robot.commands.vision.LimelightConfig.servoOffsetD;
import static frc.robot.commands.vision.LimelightConfig.servoOffsetP;
import static frc.robot.commands.vision.LimelightConfig.servoOffsetTarget;
import static frc.robot.commands.vision.LimelightConfig.servoOffsetTolerance;

/**
 * This implements the "visual servo" approach to aligning a robot to
 * an AprilTag based on:
 * <ul>
 *
 *     <li>The tx of the id to the center of the camera frame. This is
 *     based on the Limelight TX value, which is is >0 if the tag is to the
 *     left of center in the frame.</li>
 *
 *     <li>The ta of the id in the camera frame. For the Limelight, this is
 *     TA, and gets bigger the closer we are to the id.</li>
 *
 * </ul>
 *
 * By tuning this command you will be able to arrive at a reliable position
 * in front of a tag, which is an important step in targeting.</p>
 *
 * This command is implemented so it doesn't depend on a specific swerve
 * drive implementation.</p>
 */
public class LimelightVisualServoCommand extends Command {

    final Consumer<ChassisSpeeds> speedConsumer;
    final LimelightSubsystem limelight;
    final PIDController pidArea;
    final PIDController pidOffset;
    double lastOffset;
    double lastArea;
    double lastSpeedX;
    double lastSpeedY;
    boolean achievedArea;
    boolean achievedOffset;
    boolean running;

    public LimelightVisualServoCommand(LimelightSubsystem limelight,
                                       Subsystem driveSubsystem,
                                       Consumer<ChassisSpeeds> speedConsumer) {
        this.limelight = limelight;
        this.speedConsumer = speedConsumer;
        this.pidArea = new PIDController(servoAreaP.getAsDouble(), 0.0, servoAreaD.getAsDouble());
        this.pidOffset = new PIDController(servoOffsetP.getAsDouble(), 0.0, servoOffsetD.getAsDouble());
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        achievedArea = false;
        achievedOffset = false;
        running = true;

        // always reset the PIDs when you're doing closed loop
        Util.resetPid(pidOffset, servoOffsetP, servoOffsetD, servoOffsetTolerance);
        Util.resetPid(pidArea, servoAreaP, servoAreaD, servoAreaTolerance);

        Util.log("[ll-target] aligning to id");
    }

    @Override
    public void execute() {

        LimelightTarget target = limelight.getCurrentTarget();

        // if we lose sight of the id, we can't really do anything
        // and we have to quit
        if (target == null || target.id() < 1) {
            Util.log("[ll-target] NO TAG IN VIEW !!!");
            running = false;
            return;
        }

        lastSpeedX = 0.0;
        lastSpeedY = 0.0;

        // ta is how big the tag is in the camera frame; bigger means we're
        // closer to the tag (and hence we want to move in the -X direction)
        lastArea = target.ta();

        // the limelight reports TX as positive when the tag is tx to the
        // left. when this is the case, we would want to move left (the +Y
        // direction), which is also positive. so we will negate the tx
        // for our feedback.
        lastOffset = -target.tx();

        // calculate X speed (forward-back) if the tag is either too big or
        // to small in the camera frame
        if (!achievedArea) {
            lastSpeedX = pidArea.calculate(lastArea, servoAreaTarget.getAsDouble());
            achievedArea = pidArea.atSetpoint();
        }

        // calculate the Y speed (left-right) if the tag is to the right or
        // left of the desired tx
        if (!achievedOffset) {
            lastSpeedY = pidOffset.calculate(lastOffset, servoOffsetTarget.getAsDouble());
            achievedOffset = pidOffset.atSetpoint();
        }

        // we will run until we've hit both objectives
        running = !(achievedArea && achievedOffset);

        speedConsumer.accept(new ChassisSpeeds(
                Util.applyClamp(lastSpeedX, servoMaxFeedback),
                Util.applyClamp(lastSpeedY, servoMaxFeedback),
                0.0));

        // in normal operation, we're probably going to wind up with
        // many instances of this command. instead of trying to register
        // them all under different names, we'll just have whichever one
        // is running publish the "latest" information for debugging
        if (enableLogging) {
            SmartDashboard.putNumber("LimelightVisualServoCommand/SpeedX", lastSpeedX);
            SmartDashboard.putNumber("LimelightVisualServoCommand/SpeedY", lastSpeedY);
            SmartDashboard.putNumber("LimelightVisualServoCommand/OffsetCurrent", lastOffset);
            SmartDashboard.putNumber("LimelightVisualServoCommand/OffsetError", pidOffset.getError());
            SmartDashboard.putNumber("LimelightVisualServoCommand/AreaCurrent", lastArea);
            SmartDashboard.putNumber("LimelightVisualServoCommand/AreaError", pidArea.getError());
            SmartDashboard.putBoolean("LimelightVisualServoCommand/AtX?", pidOffset.atSetpoint());
            SmartDashboard.putBoolean("LimelightVisualServoCommand/AtY?", pidArea.atSetpoint());
            SmartDashboard.putBoolean("LimelightVisualServoCommand/Running?", true);
        }
    }

    @Override
    public boolean isFinished() {
        return !running;
    }

    @Override
    public void end(boolean interrupted) {

        // this command could run forever if we can't attain the target
        // heading for some reason. rather than lose control of the
        // robot during a match we will probably put it in a timeout.
        // if we lost sight of the id, or we were interrupted, we should
        // know that so we can check tuning etc.
        String which = "";
        if (!achievedArea && achievedOffset) {
            which = "X";
        }
        if (achievedArea && !achievedOffset) {
            which = "Y";
        }
        if (!achievedArea && !achievedOffset) {
            which = "X and Y";
        }
        if (!which.isEmpty()) {
            Util.log("[align-id] !!! FAILED aligning to id in %s !!!", which);
        }

        lastArea = Double.NaN;
        lastOffset = Double.NaN;
        lastSpeedX = Double.NaN;
        lastSpeedY = Double.NaN;

        if (enableLogging) {
            SmartDashboard.putBoolean("LimelightVisualServoCommand/Running?", false);
        }
    }
}
