package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConfig.collectSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.ejectSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.feedSpeed;
import static frc.robot.subsystems.intake.IntakeConfig.repositionSpeed;

public enum IntakePreset {

    COLLECT,
    REPOSITION,
    FEED,
    EJECT;

    public DoubleSupplier getSpeed() {

        return switch (this) {

            case COLLECT -> collectSpeed;
            case FEED -> feedSpeed;
            case REPOSITION -> repositionSpeed;

            // when we have a configuration property that specifies a "negative
            // speed", we want to be careful that we always treat it as negative,
            // even if someone forgot to add a minus sign
            case EJECT -> () -> -Math.abs(ejectSpeed.getAsDouble());
        };
    }
}
