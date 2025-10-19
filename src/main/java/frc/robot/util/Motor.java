package frc.robot.util;

/**
 * Generic interface for a motor so we can write and test subsystems without depending
 * on specific vendor libraries
 */
public interface Motor {
    
    /** @return position of the motor in rotations */
    double getPosition();

    /** @return velocity of the motor in rotations per second */
    double getVelocity();

    /** @return output current of the motor in amps */
    double getAmps();

    /** Applies voltage to the motor */
    void applyVolts(double volts);

    /** Sets the "brake mode" of the motor */
    void applyBrakes(boolean brake);
}
