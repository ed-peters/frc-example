package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConfig.ejectSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.feedSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.pickupSpeed;

public enum IntakePreset {

    PICKUP(pickupSpeed),
    EJECT(ejectSpeed),
    FEED(feedSpeed);

    public final DoubleSupplier speed;

    IntakePreset(DoubleSupplier speed) {
        this.speed = speed;
    }
}
