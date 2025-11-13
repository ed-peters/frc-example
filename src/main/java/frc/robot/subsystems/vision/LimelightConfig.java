package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Configuration parameters for limelight april id targeting (all default
 * values were supplied in the manufacturer's docs)
 */
public class LimelightConfig {

    /** Name of the limelight in SmartDashboard */
    public static final String limelightName = "limelight";

    /**
     * Confidence value to supply with the vision estimate when using the
     * classic algorithm
     */
    public static final Vector<N3> confidenceClassic = VecBuilder.fill(.5,.5,9999999);

    /**
     * Confidence value to supply with the vision estimate when using the
     * MegaTag2 algorithm
     */
    public static final Vector<N3> confidenceMegaTag2 = VecBuilder.fill(.7,.7,9999999);

    /** Which algorithm to use */
    public static final BooleanSupplier useMegaTag2 = pref("LimelightPose/MegaTag2?", true);

    /** Thresholds for classic algorithm */
    public static final DoubleSupplier classicMaxAmbiguity = pref("LimelightPose/ClassicMaxAmbiguity", 0.7);
    public static final DoubleSupplier classicMaxDistance = pref("LimelightPose/ClassicMaxDistance", 3.0);

    /** Thresholds for MegaTag2 algorithm */
    public static final DoubleSupplier megaTagMinArea = pref("LimelightPose/MegaTagMinArea", 0.5);
    public static final DoubleSupplier megaTagMaxYawRate = pref("LimelightPose/MegaTagMaxYawRate", 720.0);

}
