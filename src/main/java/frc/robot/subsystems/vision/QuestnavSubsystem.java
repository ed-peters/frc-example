package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.robot.subsystems.vision.QuestnavConfig.HEADSET_TO_ROBOT;
import static frc.robot.subsystems.vision.QuestnavConfig.ROBOT_TO_HEADSET;
import static frc.robot.subsystems.vision.QuestnavConfig.confidence;

public class QuestnavSubsystem extends SubsystemBase {

    final QuestNav questnav;
    final SwerveDriveSubsystem drive;
    Pose2d estimatedPose;

    public QuestnavSubsystem(SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.questnav = new QuestNav();
        this.estimatedPose = null;

        if (drive != null) {
            drive.addPoseListener(this::resetPose);
        }

        SmartDashboard.putData("QuestnavSubsystem", builder -> {
            builder.addDoubleProperty("Battery", () -> questnav.getBatteryPercent().orElse(-1), null);
            builder.addDoubleProperty("Latency", questnav::getLatency, null);
            builder.addBooleanProperty("Tracking?", questnav::isTracking, null);
            builder.addBooleanProperty("Connected?", questnav::isConnected, null);
        });
    }

    /**
     * This needs to be called if the robot's pose changes (for instance,
     * at the beginning of an auto routine) - we have to tell the headset
     * where it is on the field
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
        Util.publishPose("Questnav", estimatedPose);

        // contribute that to the swerve drive if necessary
        if (drive != null) {
            drive.addVisionPose(estimatedPose, frame.dataTimestamp(), confidence);
        }
    }
}
