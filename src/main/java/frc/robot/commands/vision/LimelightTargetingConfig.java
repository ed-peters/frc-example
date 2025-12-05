package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration for Limelight-related targeting commands
 */
public class LimelightTargetingConfig {

    // ==================================================================
    // LIMELIGHT VISUAL SERVO
    // ==================================================================

    /**
     * These properties control how we align in the X direction (forward/back)
     * to achieve the proper tag image size within the camera frame
     */
    public static DoubleSupplier servoAreaP = pref("LimelightServo/AreaKP", 2.0);
    public static DoubleSupplier servoAreaD = pref("LimelightServo/AreaKD", 0.1);
    public static DoubleSupplier servoAreaTarget = pref("LimelightServo/AreaTarget", 1.8);
    public static DoubleSupplier servoAreaTolerance = pref("LimelightServo/AreaTolerance", 0.2);

    /**
     * These properties control how we align in the Y direction (left/right)
     * to achieve the proper tag offset within the camera frame
     */
    public static DoubleSupplier servoOffsetP = pref("LimelightServo/OffsetKP", 2.0);
    public static DoubleSupplier servoOffsetD = pref("LimelightServo/OffsetKD", 0.1);
    public static DoubleSupplier servoOffsetTarget = pref("LimelightServo/OffsetTarget", 0.0);
    public static DoubleSupplier servoOffsetTolerance = pref("LimelightServo/OffsetTolerance", 3.0);

    /**
     * This puts a cap on the maximum speed (in feet per second) we will
     * translate in either direction
     */
    public static DoubleSupplier servoMaxFeedback = pref("LimelightServo/MaxFeedback", 10.0);

    // ==================================================================
    // LIMELIGHT TARGETING
    // ==================================================================

    /**
     * How far back from the tag (in feet) will we position ourself when we
     * automatically drive there?
     */
    public static DoubleSupplier tagStartingOffset = pref("LimelightTargeting/StartingOffset", 5.0);

}
