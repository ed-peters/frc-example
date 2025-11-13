package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

public class VisionConfig {

    public static final boolean enableLogging = true;

    /** Properties for aligning to an AprilTag (x direction) */
    public static DoubleSupplier limelightOffsetP = pref("LimelightCommand/OffsetP", 0.0);
    public static DoubleSupplier limelightOffsetD = pref("LimelightCommand/OffsetD", 0.0);
    public static DoubleSupplier limelightOffsetTarget = pref("LimelightCommand/OffsetTarget", 0.0);
    public static DoubleSupplier limelightOffsetTolerance = pref("LimelightCommand/OffsetTolerance", 0.0);

    /** Properties for aligning to an AprilTag (y direction) */
    public static DoubleSupplier limelightAreaP = pref("LimelightCommand/AreaP", 0.0);
    public static DoubleSupplier limelightAreaD = pref("LimelightCommand/AreaD", 0.0);
    public static DoubleSupplier limelightAreaTarget = pref("LimelightCommand/AreaTarget", 0.0);
    public static DoubleSupplier limelightAreaTolerance = pref("LimelightCommand/AreaTolerance", 0.0);

    /** Properties for aligning to an AprilTag (both directions) */
    public static DoubleSupplier limelightMaxFeedback = pref("LimelightCommand/MaxFeedback", 0.0);

}
