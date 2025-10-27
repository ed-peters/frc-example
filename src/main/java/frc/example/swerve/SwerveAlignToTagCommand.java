package frc.example.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.example.Util;

import java.util.function.DoubleSupplier;

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
    final DoubleSupplier areaSupplier;
    final DoubleSupplier offsetSupplier;
    final PIDController pidArea;
    final PIDController pidOffset;
    double speedX;
    double speedY;
    boolean doneX;
    boolean doneY;

    public SwerveAlignToTagCommand(
            SwerveDriveSubsystem drive,
            DoubleSupplier areaSupplier,
            DoubleSupplier offsetSupplier) {

        this.drive = drive;
        this.pidArea = new PIDController(tagAreaP.getAsDouble(), 0.0, tagAreaD.getAsDouble());
        this.pidOffset = new PIDController(tagOffsetP.getAsDouble(), 0.0, tagOffsetD.getAsDouble());
        this.areaSupplier = areaSupplier;
        this.offsetSupplier = offsetSupplier;

        addRequirements(drive);

        SmartDashboard.putData("AlignToAprilTag", builder -> {
            builder.addDoubleProperty("SpeedX", () -> speedX, null);
            builder.addDoubleProperty("SpeedY", () -> speedY, null);
            builder.addDoubleProperty("OffsetCurrent", offsetSupplier, null);
            builder.addDoubleProperty("OffsetError", pidOffset::getError, null);
            builder.addDoubleProperty("AreaCurrent", areaSupplier, null);
            builder.addDoubleProperty("AreaOffset", pidArea::getError, null);
            builder.addBooleanProperty("AtX?", () -> doneX, null);
            builder.addBooleanProperty("AtY?", () -> doneY, null);
        });
    }

    @Override
    public void initialize() {

        pidOffset.setP(tagOffsetP.getAsDouble());
        pidOffset.setD(tagOffsetD.getAsDouble());
        pidOffset.setTolerance(tagOffsetTolerance.getAsDouble());
        pidOffset.reset();

        pidArea.setP(tagAreaP.getAsDouble());
        pidArea.setD(tagAreaD.getAsDouble());
        pidArea.setTolerance(tagAreaTolerance.getAsDouble());
        pidArea.reset();

        doneX = false;
        doneY = false;

        Util.log("[swerve] aligning to tag");
    }

    @Override
    public void execute() {

        // until we the target offset, we need to juke left and right
        // to align to the in-view tag
        if (pidOffset.atSetpoint()) {
            doneX = true;
            speedX = 0.0;
        } else {
            doneX = false;

            // TX gets is positive when the tag is offset to the left.
            // To fix this, we want to move in the +X direction, which
            // is also positive. So we will negate the offset for our
            // feedback calculations
            double offset = -offsetSupplier.getAsDouble();
            speedX = Util.applyClamp(
                    pidOffset.calculate(offset, tagOffsetTarget.getAsDouble()),
                    tagMaxFeedback);
        }

        // until we the target area, we need to drive forward and back
        // to align to the in-view tag
        if (pidArea.atSetpoint()) {
            doneY = true;
            speedY = 0.0;
        } else {
            doneY = false;
            speedY = Util.applyClamp(
                    pidArea.calculate(areaSupplier.getAsDouble(), tagAreaTarget.getAsDouble()),
                    tagMaxFeedback);
        }

        drive.drive("align-to-tag", new ChassisSpeeds(
                speedX,
                speedY,
                0.0));
    }

    @Override
    public boolean isFinished() {
        return doneX && doneY;
    }

    @Override
    public void end(boolean interrupted) {
        if (!pidOffset.atSetpoint()) {
            Util.log("[swerve] failed aligning to tag in X");
        }
        if (!pidArea.atSetpoint()) {
            Util.log("[swerve] failed aligning to tag in Y");
        }
    }
}
