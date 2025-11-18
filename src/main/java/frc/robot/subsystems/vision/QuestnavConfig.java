package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class QuestnavConfig {

    /**
     * This is the orientation of the headset, relative to the center
     * of the robot
     */
    public static final Transform2d ROBOT_TO_HEADSET = new Transform2d(
            Units.inchesToMeters(12.0),
            Units.inchesToMeters(12.0),
            Rotation2d.fromDegrees(0.0)
    );

    /**
     * This is just the opposite of the above; no need to change this
     */
    public static final Transform2d HEADSET_TO_ROBOT = ROBOT_TO_HEADSET.inverse();

    /**
     * Confidence for position estimates
     */
    public static final Vector<N3> confidence = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );
}
