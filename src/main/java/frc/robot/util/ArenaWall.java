package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Simple enum that represents the four walls of the arean
 */
public enum ArenaWall {

    EAST(Rotation2d.kZero),
    NORTH(Rotation2d.kCCW_90deg),
    SOUTH(Rotation2d.kCW_90deg),
    WEST(Rotation2d.k180deg);

    /**
     * The direction the robot is pointing if it is "looking at" the
     * associated arena wall
     */
    public final Rotation2d facingHeading;

    ArenaWall(Rotation2d facingHeading) {
        this.facingHeading = facingHeading;
    }
}
