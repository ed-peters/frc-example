package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.robot.subsystems.vision.QuestnavConfig.HEADSET_TO_ROBOT;
import static frc.robot.subsystems.vision.QuestnavConfig.ROBOT_TO_HEADSET;
import static frc.robot.subsystems.vision.QuestnavConfig.confidence;


/**
 * Subsystem that uses the {@link QuestNav} to provide up-to-date pose
 * estimation. This subsystem is implemented so it doesn't depend on a specific
 * swerve drive implementation for supplying pose estimates or resetting the
 * pose.
 */public class QuestnavSubsystem extends SubsystemBase {

    final QuestNav questnav;
    final VisionEstimateConsumer estimateConsumer;
    Pose2d estimatedPose;
    double estimatedTimestamp;

    /**
     * Creates a {@link QuestnavSubsystem}
     * @param estimateConsumer accepts valid pose estimates every period
     */
    public QuestnavSubsystem(VisionEstimateConsumer estimateConsumer) {

        this.estimateConsumer = estimateConsumer;
        this.questnav = new QuestNav();
        this.estimatedPose = null;

        SmartDashboard.putData("QuestnavSubsystem", builder -> {
            builder.addDoubleProperty("Battery", () -> questnav.getBatteryPercent().orElse(-1), null);
            builder.addDoubleProperty("Latency", questnav::getLatency, null);
            builder.addBooleanProperty("Tracking?", questnav::isTracking, null);
            builder.addBooleanProperty("Connected?", questnav::isConnected, null);
            builder.addDoubleProperty("Timestamp", () -> estimatedTimestamp, null);
        });
    }

    /**
     * TODO you MUST call this if robot's pose changes
     * Specific example - at the beginning of auto when you determine where the
     * robot is on the field, you have to tell the headset where it is
     */
    public void resetPose(Pose2d robotPose) {
        questnav.setPose(robotPose.transformBy(ROBOT_TO_HEADSET));
    }

    @Override
    public void periodic() {

        // process all the commands to and from the headset
        questnav.commandPeriodic();

        // if there are no unread pose frames, we have nothing left to do
        PoseFrame [] poseFrames = questnav.getAllUnreadPoseFrames();
        if (poseFrames == null || poseFrames.length == 0) {
            return;
        }

        // get the estimated headset pose and transform it to robot space
        PoseFrame frame = poseFrames[poseFrames.length - 1];
        estimatedPose = frame.questPose().transformBy(HEADSET_TO_ROBOT);
        estimatedTimestamp = frame.dataTimestamp();

        // publish the pose for debugging
        Util.publishPose("QuestnavPose", estimatedPose);

        estimateConsumer.accept(
                    estimatedPose,
                    estimatedTimestamp,
                    confidence);
    }
}
