package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import frc.robot.util.vision.Limelight;
import frc.robot.util.vision.LimelightEstimate;
import frc.robot.util.vision.LimelightTarget;

import java.util.function.Supplier;

import static frc.robot.subsystems.vision.LimelightConfig.classicMaxAmbiguity;
import static frc.robot.subsystems.vision.LimelightConfig.classicMaxDistance;
import static frc.robot.subsystems.vision.LimelightConfig.confidenceClassic;
import static frc.robot.subsystems.vision.LimelightConfig.confidenceMegaTag2;
import static frc.robot.subsystems.vision.LimelightConfig.limelightName;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMaxSpinRate;
import static frc.robot.subsystems.vision.LimelightConfig.megaTagMinArea;
import static frc.robot.subsystems.vision.LimelightConfig.useMegaTag2;

/**
 * Subsystem that uses the {@link Limelight} to provide up-to-date pose
 * estimation and targeting information. This subsystem is implemented so it
 * doesn't depend on a specific swerve drive implementation for supplying
 * pose estimates.
 */
public class LimelightSubsystem extends SubsystemBase {

    final Limelight limelight;
    final Supplier<Rotation2d> gyroHeadingSupplier;
    final VisionEstimateConsumer estimateConsumer;
    LimelightEstimate latestEstimate;
    LimelightTarget latestTarget;

    /**
     * Creates a new {@link LimelightSubsystem}
     * @param gyroHeadingSupplier supplies the most recent gyro heading
     * @param estimateConsumer accepts valid pose estimates every period
     */
    public LimelightSubsystem(Supplier<Rotation2d> gyroHeadingSupplier,
                              VisionEstimateConsumer estimateConsumer) {

        this.limelight = new Limelight(limelightName);
        this.gyroHeadingSupplier = gyroHeadingSupplier;
        this.estimateConsumer = estimateConsumer;
        this.latestEstimate = LimelightEstimate.NO_TAG;
        this.latestTarget = LimelightTarget.NO_TARGET;

        SmartDashboard.putData("LimelightSubsystem", builder -> {
            builder.addStringProperty("PoseEstimate/Algorithm", () -> useMegaTag2.getAsBoolean() ? "megaTag2" : "classic", null);
            builder.addDoubleProperty("PoseEstimate/Ambiguity", () -> latestEstimate.ambiguity(), null);
            builder.addDoubleProperty("PoseEstimate/Area", () -> latestEstimate.area(), null);
            builder.addDoubleProperty("PoseEstimate/Distance", () -> latestEstimate.distance(), null);
            builder.addDoubleProperty("PoseEstimate/SpinRate", () -> latestEstimate.yawRate(), null);
            builder.addStringProperty("PoseEstimate/Status", () -> latestEstimate.status().toString(), null);
            builder.addDoubleProperty("Target/offset", () -> latestTarget.offset(), null);
            builder.addDoubleProperty("Target/area", () -> latestTarget.area(), null);
            builder.addDoubleProperty("Target/tagId", () -> latestTarget.tagId(), null);
        });
    }

    /**
     * @return the latest target, if it's valid; null otherwise
     */
    public LimelightTarget getLatestGoodTarget() {
        return latestTarget.isValid() ? latestTarget : null;
    }

    /**
     * @return the latest pose estimate, if it's valid; null otherwise
     */
    public Pose2d getLatestGoodPose() {
        return latestEstimate.isValid() ? latestEstimate.getPose() : null;
    }

    /**
     * @return true if the specified tag is in view
     */
    public boolean isTagInView(AprilTag tag) {
        return limelight.isTagInView(tag.ID);
    }

    @Override
    public void periodic() {

        // every cycle we update the pose estimate using the selected
        // algorithm. if our frame rate was more than 50 Hz, we could think
        // about doing this in a background thread to get more pose estimates,
        // but that's probably overkill for our level of accuracy right now
        if (useMegaTag2.getAsBoolean()) {
            Rotation2d currentHeading = gyroHeadingSupplier.get();
            latestEstimate = limelight.getEstimateMegaTag2(
                    megaTagMinArea,
                    megaTagMaxSpinRate,
                    currentHeading);
        } else {
            latestEstimate = limelight.getEstimateClassic(
                    classicMaxAmbiguity,
                    classicMaxDistance);
        }

        // if we have a good estimate, we want to let the vision system know
        // about it; the "confidence" parameter says how much to trust the
        // different aspects of the estimate (X, Y and heading)
        if (latestEstimate.isValid()) {
            Vector<N3> confidence = useMegaTag2.getAsBoolean()
                    ? confidenceMegaTag2
                    : confidenceClassic;
            estimateConsumer.accept(
                    latestEstimate.getPose(),
                    latestEstimate.getTimestamp(),
                    confidence);
        }

        // publish pose and tag position information for debugging
        Util.publishPose("LimelightPose", latestEstimate.getPose());

        // update target information
        latestTarget = limelight.getTarget();
    }
}
