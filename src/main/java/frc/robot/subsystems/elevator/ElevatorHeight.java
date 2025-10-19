package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConfig.heightLevel1;
import static frc.robot.subsystems.elevator.ElevatorConfig.heightLevel2;
import static frc.robot.subsystems.elevator.ElevatorConfig.heightLevel3;

/**
 * We usually use enums to represent preset heights. Each of these will
 * correspond to a property in {@link ElevatorConfig} that controls the
 * actual associated height.
 */
public enum ElevatorHeight {

    LEVEL1(heightLevel1),
    LEVEL2(heightLevel2),
    LEVEL3(heightLevel3);

    public final DoubleSupplier height;

    ElevatorHeight(DoubleSupplier height) {
        this.height = height;
    }
}
