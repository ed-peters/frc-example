package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.Motor;
import frc.robot.util.Util;

import static frc.robot.subsystems.intake.IntakeConfig.gearRatio;
import static frc.robot.subsystems.intake.IntakeConfig.wheelDiameterInches;
import static frc.robot.subsystems.intake.IntakeConfig.wheelMassLbs;

/**
 * Implements the {@link Motor} interface along with a physics-based
 * simulation of an attached flywheel.
 */
public class IntakeMotorSim implements Motor {

    /**
     * Physical properties of the wheel
     */
    public static final double wheelMassKg = Units.lbsToKilograms(wheelMassLbs);
    public static final double wheelRadiusM = Units.inchesToMeters(wheelDiameterInches / 2.0);
    public static final double wheelMoi = 0.5 * wheelMassKg * wheelRadiusM * wheelRadiusM;

    FlywheelSim sim;
    double position;

    public IntakeMotorSim() {
        DCMotor motor = DCMotor.getFalcon500(1);
        sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motor, wheelMoi, gearRatio),
                motor);
    }

    @Override
    public void applyBrake(boolean brake) {
        Util.log("[intake-sim] no support for brakes");
    }

    @Override
    public boolean isBrakeEnabled() {
        return false;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public double getVelocity() {
        return sim.getAngularVelocityRPM() / 60.0;
    }

    @Override
    public double getCurrent() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void applyVolts(double volts) {
        sim.setInputVoltage(volts);
        sim.update(Util.DT);
        position += getVelocity() * Util.DT;
    }
}
