package frc.robot.util.vision;

/**
 * <p>This represents information about an in-view Limelight target. It is
 * meant to be used for "visual servo" targeting (explained in depth in the <a
 * href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing">Limelight
 * documentation</a>.</p>
 *
 * @param tagId if the target is an AprilTag, this will be the tag ID
 * @param area area of the target in the camera frame (if you are directly
 *             facing the target, driving the robot in the +X direction will
 *             make this value increase, and -X will make it decrease)
 * @param offset how far the target is to the right of the center of the camera
 *               frame (if you are directly facing the target, driving the robot
 *               in the +Y direction will make this value increase)
 */
public record LimelightTarget(int tagId, double area, double offset) {

    /**
     * Creates a {@link LimelightEstimate} from raw values reported by
     * the Limelight.     *
     * @param tid the fiducial ID reported by the Limelight
     * @param ta the TA value reported by the Limelight
     * @param tx the TX value reported by the Limelight
     * @return the target
     */
    public static LimelightTarget fromRaw(int tid, double ta, double tx) {

        // the limelight reports TX as positive when the tag is tx to the
        // left. when this is the case, we would want to move left (the +Y
        // direction), which is also positive. so we will negate the tx
        return new LimelightTarget(tid, ta, -tx);
    }
}
