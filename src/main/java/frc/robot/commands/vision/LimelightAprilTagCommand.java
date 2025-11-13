package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.LimelightTarget;
import frc.robot.util.PDController;
import frc.robot.util.Util;

import static frc.robot.commands.vision.VisionConfig.enableLogging;
import static frc.robot.commands.vision.VisionConfig.limelightAreaD;
import static frc.robot.commands.vision.VisionConfig.limelightAreaP;
import static frc.robot.commands.vision.VisionConfig.limelightAreaTarget;
import static frc.robot.commands.vision.VisionConfig.limelightAreaTolerance;
import static frc.robot.commands.vision.VisionConfig.limelightMaxFeedback;
import static frc.robot.commands.vision.VisionConfig.limelightOffsetD;
import static frc.robot.commands.vision.VisionConfig.limelightOffsetP;
import static frc.robot.commands.vision.VisionConfig.limelightOffsetTarget;
import static frc.robot.commands.vision.VisionConfig.limelightOffsetTolerance;

/**
 * This aligns the robot to an AprilTag based on:
 * <ul>
 *
 *     <li>The offset of the id to the center of the camera frame.
 *     This is based on the Limelight TX value, which is is >0 if the
 *     id is to the right in the frame.</li>
 *
 *     <li>The area of the id in the camera frame. For the Limelight,
 *     this is TA, and gets bigger the closer we are to the id.</li>
 *
 * </ul>
 *
 * By tuning this command you will be able to arrive at a reliable
 * position in front of an a id, which is an important step in
 * targeting.</p>
 */
public class LimelightAprilTagCommand extends Command {

    final SwerveDriveSubsystem drive;
    final LimelightSubsystem limelight;
    final PDController pidArea;
    final PDController pidOffset;
    double lastOffset;
    double lastArea;
    double lastSpeedX;
    double lastSpeedY;
    boolean achievedX;
    boolean achievedY;
    boolean running;

    public LimelightAprilTagCommand(SwerveDriveSubsystem drive,
                                    LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;
        this.pidArea = new PDController(limelightAreaP, limelightAreaD, limelightMaxFeedback, limelightAreaTolerance);
        this.pidOffset = new PDController(limelightOffsetP, limelightOffsetD, limelightMaxFeedback, limelightOffsetTolerance);
        addRequirements(drive);
    }

    @Override
    public void initialize() {

        achievedX = false;
        achievedY = false;
        running = true;

        // always reset the PIDs when you're doing closed loop
        pidOffset.reset();
        pidArea.reset();

        Util.log("[ll-id] aligning to id");
    }

    @Override
    public void execute() {

        LimelightTarget target = limelight.getCurrentTarget();

        // if we lose sight of the id, we can't really do anything
        // and we have to quit
        if (target == null || target.id() < 1) {
            Util.log("[ll-id] NO TAG IN VIEW !!!");
            running = false;
            return;
        }

        lastSpeedX = 0.0;
        lastSpeedY = 0.0;

        // area is how big the target is in the camera frame; bigger means
        // we're closer to the target (and thus would want to move in the -X
        // direction.
        lastArea = target.area();

        // the limelight reports TX as positive when the id is offset to the
        // left. if this was the case, we would want to move in the +X
        // direction, which is also positive. so we will negate the offset
        // for our feedback.
        lastOffset = -target.offset();

        // calculate X speed if we're not done centering the id
        if (!achievedX) {
            lastSpeedX = pidOffset.calculate(lastOffset, limelightOffsetTarget.getAsDouble());
            achievedX = pidOffset.atSetpoint();
        }

        // calculate the Y speed if the id isn't close enough
        if (!achievedY) {
            lastSpeedY = pidArea.calculate(lastArea, limelightAreaTarget.getAsDouble());
            achievedY = pidArea.atSetpoint();
        }

        // we will run until we've hit both objectives
        running = !(achievedX && achievedY);

        drive.drive("ll-id", new ChassisSpeeds(
                lastSpeedX,
                lastSpeedY,
                0.0));

        // in normal operation, we're probably going to wind up with
        // many instances of this command. instead of trying to register
        // them all under different names, we'll just have whichever one
        // is running publish the "latest" information for debugging
        if (enableLogging) {
            SmartDashboard.putNumber("LimelightAprilTagCommand/SpeedX", lastSpeedX);
            SmartDashboard.putNumber("LimelightAprilTagCommand/SpeedY", lastSpeedY);
            SmartDashboard.putNumber("LimelightAprilTagCommand/OffsetCurrent", lastOffset);
            SmartDashboard.putNumber("LimelightAprilTagCommand/OffsetError", pidOffset.getError());
            SmartDashboard.putNumber("LimelightAprilTagCommand/AreaCurrent", lastArea);
            SmartDashboard.putNumber("LimelightAprilTagCommand/AreaOffset", pidArea.getError());
            SmartDashboard.putBoolean("LimelightAprilTagCommand/AtX?", pidOffset.atSetpoint());
            SmartDashboard.putBoolean("LimelightAprilTagCommand/AtY?", pidArea.atSetpoint());
            SmartDashboard.putBoolean("LimelightAprilTagCommand/Running?", true);
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
        if (!achievedX && achievedY) {
            which = "X";
        }
        if (achievedX && !achievedY) {
            which = "Y";
        }
        if (!achievedX && !achievedY) {
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
            SmartDashboard.putBoolean("SwerveTargetAprilTagCommand/Running?", false);
        }
    }
}
