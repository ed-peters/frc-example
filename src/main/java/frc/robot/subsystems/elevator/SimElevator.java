package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Dash;
import frc.robot.util.Motor;

import java.util.function.DoubleConsumer;

import static frc.robot.subsystems.elevator.ElevatorConfig.*;

/**
 * Implements the {@link Motor} interface as if it were attached to a
 * real elevator using physics-based simulation. Includes a visual
 * representation of the elevator's position which you can view in
 * the sim GUI.
 */
public class SimElevator implements Motor {

    // g = 1.3028
    // v = 0.0991

    // Physical elevator description
    public static final int NUM_MOTORS = 2;
    public static final double MASS_LBS = 25.0;
    public static final double DRUM_RADIUS_IN = 1.0;
    public static final double GEAR_RATIO = 5.0;

    ElevatorSim sim;
    DoubleConsumer display;
    boolean brakeEnabled;

    public SimElevator() {
        this.sim = makeSim(true);
        this.display = createWidget();
        this.brakeEnabled = true;
        applyVolts(0.0);
        SmartDashboard.putData("SimElevator", builder -> {
            builder.addDoubleProperty("SimHeight", this::getHeight, this::setHeight);
        });
    }

    public double getHeight() {
        return Units.metersToInches(sim.getPositionMeters());
    }

    public void setHeight(double inches) {
        sim.setState(Units.inchesToMeters(inches), 0.0);
    }

    protected ElevatorSim makeSim(boolean brakeEnabled) {
        return new ElevatorSim(
                DCMotor.getNEO(NUM_MOTORS),
                GEAR_RATIO,
                Units.lbsToKilograms(MASS_LBS),
                Units.inchesToMeters(DRUM_RADIUS_IN),
                Units.inchesToMeters(minHeight.getAsDouble()),
                Units.inchesToMeters(maxHeight.getAsDouble()),
                !brakeEnabled,
                Units.inchesToMeters(minHeight.getAsDouble()));
    }

    private DoubleConsumer createWidget() {
        DoubleConsumer widget = Dash.createElevatorWidget("ElevatorSim");
        return value -> {
            double num = (value - minHeight.getAsDouble());
            double den = (maxHeight.getAsDouble() - minHeight.getAsDouble());
            widget.accept(num / den);
        };
    }

    @Override
    public double getPosition() {
        return Units.metersToInches(sim.getPositionMeters()) / inchesPerRotation;
    }

    @Override
    public double getVelocity() {
        return Units.metersToInches(sim.getVelocityMetersPerSecond()) / inchesPerRotation;
    }

    @Override
    public double getAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void applyVolts(double volts) {
        sim.setInputVoltage(volts);
        sim.update(0.02);
        display.accept(Units.metersToInches(sim.getPositionMeters()));
    }

    @Override
    public void applyBrakes(boolean brake) {
        if (brake != brakeEnabled) {
            System.err.println("[sim] switching brake");
            ElevatorSim newSim = makeSim(brake);
            newSim.setState(sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
            sim = newSim;
            brakeEnabled = brake;
        }
    }
}