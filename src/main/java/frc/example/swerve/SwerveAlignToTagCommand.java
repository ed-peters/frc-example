package frc.example.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.example.AprilTarget;
import frc.example.PDController;
import frc.example.Util;

import java.util.function.Supplier;

import static frc.example.swerve.SwerveAlignConfig.tagAreaD;
import static frc.example.swerve.SwerveAlignConfig.tagAreaP;
import static frc.example.swerve.SwerveAlignConfig.tagAreaTarget;
import static frc.example.swerve.SwerveAlignConfig.tagAreaTolerance;
import static frc.example.swerve.SwerveAlignConfig.tagMaxFeedback;
import static frc.example.swerve.SwerveAlignConfig.tagOffsetD;
import static frc.example.swerve.SwerveAlignConfig.tagOffsetP;
import static frc.example.swerve.SwerveAlignConfig.tagOffsetTarget;
import static frc.example.swerve.SwerveAlignConfig.tagOffsetTolerance;

/**
 * Aligns the robot to an AprilTag based on:
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
public class SwerveAlignToTagCommand extends Command {

    final SwerveDriveSubsystem drive;
    final Supplier<AprilTarget> targetSupplier;
    final PDController pidArea;
    final PDController pidOffset;
    double lastOffset;
    double lastArea;
    double lastSpeedX;
    double lastSpeedY;
    boolean doneX;
    boolean doneY;

    public SwerveAlignToTagCommand(SwerveDriveSubsystem drive,
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
        pidOffset.reset();
        pidArea.reset();
        doneX = false;
        doneY = false;
        Util.log("[align-tag] aligning to tag");
    }

    @Override
    public void execute() {

        AprilTarget target = targetSupplier.get();

        // if we lose sight of the tag, we can't really do anything
        // and we have to quit
        if (target == null || target.id() < 0) {
            Util.log("[align-tag] MISSING TAG!!!");
            doneX = true;
            doneY = true;
            return;
        }

        // TX is positive when the tag is offset to the left.
        // To fix this, we want to move in the +X direction, which
        // is also positive. So we will negate the offset for our
        // feedback calculations
        lastSpeedX = 0.0;
        lastSpeedY = 0.0;
        lastOffset = -target.offset();
        lastArea = target.area();

        if (!doneX) {
            lastSpeedX = pidOffset.calculate(lastOffset, tagOffsetTarget.getAsDouble());
            doneX = pidOffset.atSetpoint();
        }

        if (!doneY) {
            lastSpeedY = pidArea.calculate(lastArea, tagAreaTarget.getAsDouble());
            doneY = pidArea.atSetpoint();
        }

        drive.drive("align-tag", new ChassisSpeeds(
                lastSpeedX,
                lastSpeedY,
                0.0));
    }

    @Override
    public boolean isFinished() {
        return doneX && doneY;
    }

    @Override
    public void end(boolean interrupted) {

        if (doneX && doneY) {
            Util.log("[align-tag] succeeded aligning to tag");
        }

        // warn about this, in case this command had to be interrupted
        // by a timeout because it never got to the target alignment or
        // something like that; that's a sign that it's badly tuned or
        // may need a larger tolerance
        else {
            String which = "";
            if (doneX) {
                which = "Y";
            } else if (doneY) {
                which = "X";
            } else {
                which = "X and Y";
            }
            Util.log("[align-tag] !!! FAILED aligning to tag in %s !!!", which);
        }

        lastArea = Double.NaN;
        lastOffset = Double.NaN;
        lastSpeedX = Double.NaN;
        lastSpeedY = Double.NaN;
    }
}
