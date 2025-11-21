package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.shooter.ShooterConfig.speed1;
import static frc.robot.subsystems.shooter.ShooterConfig.speed2;
import static frc.robot.subsystems.shooter.ShooterConfig.speed3;

public enum ShooterPreset {

    SPEED_1,
    SPEED_2,
    SPEED_3;

    public DoubleSupplier getSpeed() {
        return switch (this) {
            case SPEED_1 -> speed1;
            case SPEED_2 -> speed2;
            case SPEED_3 -> speed3;
        };
    }
}
