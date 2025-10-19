package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

public class Dash {

    // ===========================================================
    // LOGGING
    // ===========================================================

    /**
     * Formats & writes a message to stdout after formatting (adds a
     * newline to the end of the message if there isn't one)
     */
    public static void log(String message, Object... args) {
        System.out.printf(message, args);
        if (!message.endsWith("%n")) {
            System.out.println();
        }
    }

    // ===========================================================
    // PUBLISHING (SIMPLE VALUES)
    // ===========================================================

    /** Writes a value to SmartDashboard */
    public static void publish(String name, boolean value) {
        SmartDashboard.putBoolean(name, value);
    }

    /** Writes a value to SmartDashboard, truncated to 3 digits */
    public static void publish(String name, double value) {
        SmartDashboard.putNumber(name, Util.chopDigits(value));
    }

    /** Writes a value to SmartDashboard, converted to feet and truncated to 3 digits */
    public static void publishFeet(String name, double meters) {
        publish(name, Units.metersToFeet(meters));
    }

    /** Writes a value to SmartDashboard, converted to degrees and truncated to 3 digits */
    public static void publishDegrees(String name, double radians) {
        publish(name, Math.toDegrees(radians));
    }

    /** Writes a value to SmartDashboard */
    public static void publish(String name, String value) {
        SmartDashboard.putString(name, value);
    }

    // ===========================================================
    // PUBLISHING (STRUCTS)
    // ===========================================================

    static Map<String,StructPublisher<Pose2d>> posePublishers = new HashMap<>();

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

    // ===========================================================
    // TOGGLES
    // ===========================================================

    public interface Toggle {
        void check();
    }

    /**
     * Publishes a boolean to the dashboard, and then returns a {@link Toggle}
     * that you can call to see if it's changed. If it has, the supplied
     * callback will be invoked. This is helpful for implement a motor brake
     * toggle, when testing and setting the brake mode on the motor each
     * cycle is too high.
     */
    public static Toggle toggle(String name, boolean defaultValue, BooleanConsumer callback) {

        SmartDashboard.putBoolean(name, defaultValue);
        callback.accept(defaultValue);

        AtomicBoolean lastValue = new AtomicBoolean(defaultValue);
        return () -> {
            boolean currentValue = SmartDashboard.getBoolean(name, defaultValue);
            if (currentValue != lastValue.get()) {
                callback.accept(currentValue);
                lastValue.set(currentValue);
            }
        };
    }

    // ===========================================================
    // MECHANISMS
    // ===========================================================

    /**
     * Creates a {@link Mechanism2d} representing an elevator, and returns
     * a {@link DoubleConsumer} that can be used to publish its height.
     * The height should be supplied a ratio of the maximum height (from
     * 0.0 to 1.0)
     */
    public static DoubleConsumer createElevatorWidget(String name) {
        Mechanism2d mech = new Mechanism2d(4, 4);
        MechanismRoot2d root = mech.getRoot(name, 0.5, 0.5);
        MechanismLigament2d handle = root.append(new MechanismLigament2d("riser", 0.5, 90.0, 1, new Color8Bit(Color.kBlack)));
        handle.append(new MechanismLigament2d("platform", 3.0, -90, 1, new Color8Bit(Color.kWhite)));
        SmartDashboard.putData(name, mech);
        return ratio -> {
            handle.setLength(MathUtil.clamp(ratio, 0.0, 1.0) * 3.0);
        };
    }

    /**
     * Creates {@link Mechanism2d} representing an arm, and returns
     * a {@link DoubleConsumer} that can be used to publish its angle.
     * The angle should be supplied as a number of degrees. This only
     * handles degrees from 0.0 to 90.0
     */
    public static DoubleConsumer createArmWidget(String name) {
        Mechanism2d mech = new Mechanism2d(4, 4);
        MechanismRoot2d root = mech.getRoot(name, 2.0, 2.0);
        MechanismLigament2d ligament = root.append(
                new MechanismLigament2d("arm", 1.5, 0.0, 1, new Color8Bit(Color.kWhite)));
        SmartDashboard.putData(name, mech);
        return ligament::setAngle;
    }
}
