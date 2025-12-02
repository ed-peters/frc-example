package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

public class LimelightConfig {

    public static final boolean enableLogging = true;

    // ==================================================================
    // LIMELIGHT VISUAL SERVO
    // ==================================================================

    /**
     * These properties control how we align in the X direction (forward/back)
     * to achieve the proper tag image size within the camera frame
     */
    public static DoubleSupplier servoAreaP = pref("Limelight/Servo/AreaKP", 0.5);
    public static DoubleSupplier servoAreaD = pref("Limelight/Servo/AreaKD", 0.0);
    public static DoubleSupplier servoAreaTarget = pref("Limelight/Servo/AreaTarget", 4.0);
    public static DoubleSupplier servoAreaTolerance = pref("Limelight/Servo/AreaTolerance", 0.5);

    /**
     * These properties control how we align in the Y direction (left/right)
     * to achieve the proper tag offset within the camera frame
     */
    public static DoubleSupplier servoOffsetP = pref("Limelight/Servo/OffsetKP", 0.1);
    public static DoubleSupplier servoOffsetD = pref("Limelight/Servo/OffsetKD", 0.0);
    public static DoubleSupplier servoOffsetTarget = pref("Limelight/Servo/OffsetTarget", 0.0);
    public static DoubleSupplier servoOffsetTolerance = pref("Limelight/Servo/OffsetTolerance", 3.0);

    /**
     * This puts a cap on the maximum speed (in feet per second) we will
     * translate in either direction
     */
    public static DoubleSupplier servoMaxFeedback = pref("Limelight/Servo/MaxFeedback", 10.0);

    // ==================================================================
    // LIMELIGHT TARGETING
    // ==================================================================

    /**
     * How close (in feet) do we have to be to a target before we will
     * automatically drive there?
     */
    public static DoubleSupplier tagMaxDistance = pref("Limelight/Targeting/MaxDistance", 10.0);

    /**
     * How far back from the tag (in feet) will we position ourself when we
     * automatically drive there?
     */
    public static DoubleSupplier tagStartingOffset = pref("Limelight/Targeting/StartingOffset", 5.0);

}
