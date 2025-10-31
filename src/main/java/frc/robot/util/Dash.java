package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import java.util.HashMap;
import java.util.Map;

public class Dash {

    // ===========================================================
    // PUBLISHING (STRUCTS)
    // ===========================================================

    static Map<String, StructPublisher<Pose2d>> posePublishers = new HashMap<>();

    /**
     * Publish a pose to the dashboard (automatically adds the "SmartDashboard"
     * prefix so it will show up under that topic in the dashboard)
     */
    public static void publish(String key, Pose2d val) {

        // see if a publisher already exists
        StructPublisher<Pose2d> publisher = posePublishers.get(key);

        // create it if it doesn't (we add the SmartDashboard prefix so
        // it shows up next to other values we publish)
        if (publisher == null) {
            publisher = NetworkTableInstance.getDefault()
                    .getStructTopic("SmartDashboard/"+key, Pose2d.struct)
                    .publish();
            posePublishers.put(key, publisher);
        }

        publisher.set(val);
    }

}
