package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AprilTarget;
import frc.robot.util.PDController;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.Supplier;

import static frc.robot.commands.swerve.SwerveTargetingConfig.tagAreaD;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagAreaP;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagAreaTarget;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagAreaTolerance;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagMaxFeedback;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagOffsetD;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagOffsetP;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagOffsetTarget;
import static frc.robot.commands.swerve.SwerveTargetingConfig.tagOffsetTolerance;

/**
 * This aligns the robot to an AprilTag based on:
 * <ul>
 *
 *     <li>The offset of the tag to the center of the camera frame.
 *     This is written based on the Limelight TX value, which is
 *     is >0 if the tag is to the right in the frame.</li>
 *
 *     <li>The area of the tag in the camera frame. For the Limelight,
 *     this is TA, and gets bigger the closer we are to the tag.</li>
 *
 * </ul>
 */
public class SwerveTargetAprilTagCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<AprilTarget> targetSupplier;
    final PDController pidArea;
    final PDController pidOffset;
    double lastOffset;
    double lastArea;
    double lastSpeedX;
    double lastSpeedY;
    boolean achievedX;
    boolean achievedY;
    boolean running;

    public SwerveTargetAprilTagCommand(SwerveDriveSubsystem drive,
                                       Supplier<AprilTarget> targetSupplier) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        this.pidArea = new PDController(tagAreaP, tagAreaD, tagMaxFeedback, tagAreaTolerance);
        this.pidOffset = new PDController(tagOffsetP, tagOffsetD, tagMaxFeedback, tagOffsetTolerance);
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
            builder.addDoubleProperty("SpeedX", () -> lastSpeedX, null);
            builder.addDoubleProperty("SpeedY", () -> lastSpeedY, null);
            builder.addDoubleProperty("OffsetCurrent", () -> lastOffset, null);
            builder.addDoubleProperty("OffsetError", pidOffset::getError, null);
            builder.addDoubleProperty("AreaCurrent", () -> lastArea, null);
            builder.addDoubleProperty("AreaOffset", pidArea::getError, null);
            builder.addBooleanProperty("AtX?", pidOffset::atSetpoint, null);
            builder.addBooleanProperty("AtY?", pidArea::atSetpoint, null);
        });
    }

    @Override
    public void initialize() {

        achievedX = false;
        achievedY = false;
        running = true;

        // always reset the PIDs when you're doing closed loop
        pidOffset.reset();
        pidArea.reset();

        Util.log("[align-tag] aligning to tag");
    }

    @Override
    public void execute() {

        AprilTarget target = targetSupplier.get();

        // if we lose sight of the tag, we can't really do anything
        // and we have to quit
        if (target == null || target.id() < 0) {
            Util.log("[align-tag] TAG DROPPED OUT OF VIEW !!!");
            running = false;
            return;
        }

        // the limelight reports TX as positive when the tag is offset to the
        // left. to fix this, we want to move in the +X direction, which is
        // also positive. so we will negate the offset for our feedback.
        lastSpeedX = 0.0;
        lastSpeedY = 0.0;
        lastOffset = -target.offset();
        lastArea = target.area();

        // calculate X speed if we're not done centering the tag
        if (!achievedX) {
            lastSpeedX = pidOffset.calculate(lastOffset, tagOffsetTarget.getAsDouble());
            achievedX = pidOffset.atSetpoint();
        }

        // calculate the Y speed if the tag isn't close enough
        if (!achievedY) {
            lastSpeedY = pidArea.calculate(lastArea, tagAreaTarget.getAsDouble());
            achievedY = pidArea.atSetpoint();
        }

        // we will run until we've hit both objectives
        running = !(achievedX && achievedY);

        drive.drive("align-tag", new ChassisSpeeds(
                lastSpeedX,
                lastSpeedY,
                0.0));
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
        // if we lost sight of the tag, or we were interrupted, we should
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
            Util.log("[align-tag] !!! FAILED aligning to tag in %s !!!", which);
        }

        lastArea = Double.NaN;
        lastOffset = Double.NaN;
        lastSpeedX = Double.NaN;
        lastSpeedY = Double.NaN;
    }
}
