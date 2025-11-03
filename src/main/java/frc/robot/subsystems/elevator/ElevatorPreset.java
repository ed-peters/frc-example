package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.presetL1;
import static frc.robot.subsystems.elevator.ElevatorConfig.presetL2;
import static frc.robot.subsystems.elevator.ElevatorConfig.presetL3;

/**
 * We can use enumerations to define preset heights of interest (for
 * instance in 2025 the scoring heights on the "reef").
 */
public enum ElevatorPreset {

    L1,
    L2,
    L3;

    /** @return a preference used to get the associated height */
    public DoubleSupplier getHeight() {
        return switch (this) {
            case L1 -> presetL1;
            case L2 -> presetL2;
            case L3 -> presetL3;
        };
    }
}
