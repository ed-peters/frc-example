package frc.robot.util;

/**
 * This interface lets us write subsystems and commands without depending
 * on a specific vendor hardware library
 */
public interface Motor {

    /** @return is the motor brake enabled? */
    boolean isBrakeEnabled();

    /** @return motor position in rotations */
    double getPosition();

    /** @return motor velocity in rotations per second */
    double getVelocity();

    /** @return motor output current in amps */
    double getCurrent();

    /** Applies voltage to the motor */
    void applyVolts(double volts);

    /** Enables/disables the motor brake */
    void applyBrake(boolean brake);
}
