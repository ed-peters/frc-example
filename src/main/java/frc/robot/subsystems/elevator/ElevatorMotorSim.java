package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.Motor;

import java.util.function.DoubleConsumer;

import static frc.robot.subsystems.elevator.ElevatorConfig.inchesPerRotation;
import static frc.robot.subsystems.elevator.ElevatorConfig.maxHeight;
import static frc.robot.subsystems.elevator.ElevatorConfig.minHeight;
import static frc.robot.util.Util.DT;
import static frc.robot.util.Util.log;

/**
 * Implements the {@link Motor} interface. It shows how to:
 *
 * <ul>
 *
 *     <li>Use a {@link ElevatorMotorSim} to do a real physics-based
 *     simulation of motors powering an elevator</li>
 *
 *     <li>Reverse the calculations of the sim to determine motor
 *     position / velocity based on the physical mechanism</li>
 *
 *     <li>Use a {@link Mechanism2d} to draw a widget on the dashboard
 *     for visualizing the state of a physical mechanism</li>
 *
 * </ul>
 */
public class ElevatorMotorSim implements Motor {

    // g = 1.3028
    // v = 0.0992

    // Physical elevator description
    public static final int NUM_MOTORS = 2;
    public static final double MASS_LBS = 25.0;
    public static final double DRUM_RADIUS_IN = 1.0;
    public static final double GEAR_RATIO = 5.0;

    DoubleConsumer display;
    ElevatorSim sim;
    double lastHeight;
    double thisHeight;
    double calculatedVelocity;

    public ElevatorMotorSim() {

        this.display = createWidget();
        this.sim = new ElevatorSim(
            DCMotor.getNEO(NUM_MOTORS),
            GEAR_RATIO,
            Units.lbsToKilograms(MASS_LBS),
            Units.inchesToMeters(DRUM_RADIUS_IN),
            Units.inchesToMeters(minHeight.getAsDouble()),
            Units.inchesToMeters(maxHeight.getAsDouble()),
            true,
            Units.inchesToMeters(minHeight.getAsDouble()));
        this.lastHeight = Double.NaN;
        this.thisHeight = Double.NaN;
        this.calculatedVelocity = Double.NaN;

        applyVolts(0.0);

        SmartDashboard.putData("SimElevator", builder -> {
            builder.addDoubleProperty("SimHeight", this::getSimHeight, this::setHeight);
            builder.addDoubleProperty("CalculatedVelocity", () -> calculatedVelocity, null);
        });
    }

    /**
     * Creates the dashboard widget, and returns a {@link DoubleConsumer} that
     * can be used to set its height. The consumer accepts a height in inches,
     * scales that to the height of the canvas, and adjusts the widget
     * accordingly.
     */
    private DoubleConsumer createWidget() {

        // creates a horizontal line widget in the dashboard
        Mechanism2d mech = new Mechanism2d(4, 4);
        MechanismRoot2d root = mech.getRoot("ElevatorSim", 0.5, 0.5);
        MechanismLigament2d handle = root.append(new MechanismLigament2d("riser", 0.5, 90.0, 1, new Color8Bit(Color.kBlack)));
        handle.append(new MechanismLigament2d("platform", 3.0, -90, 1, new Color8Bit(Color.kWhite)));

        // add it to the dashboard
        SmartDashboard.putData("ElevatorSim", mech);

        // returns a function that will set the line's height
        return value -> {
            double num = (value - minHeight.getAsDouble());
            double den = (maxHeight.getAsDouble() - minHeight.getAsDouble());
            handle.setLength(MathUtil.clamp(num / den, 0.0, 1.0) * 3.0);
        };
    }

    /**
     * @return current height of the elevator in inches, as determined by
     * the simulation logic
     */
    public double getSimHeight() {
        return thisHeight;
    }

    /**
     * @return the motor position, calculated from the simulation's
     * estimate of elevator height
     */
    @Override
    public double getPosition() {
        return getSimHeight() / inchesPerRotation;
    }

    /**
     * @return the motor velocity, calculated from the simulation's
     * estimate of elevator velocity
     */
    @Override
    public double getVelocity() {
        return calculatedVelocity / inchesPerRotation;
    }

    /**
     * Set the current height of the simulation in inches
     */
    public void setHeight(double inches) {
        log("[elevator-sim] setting height to %.2f", inches);
        sim.setState(Units.inchesToMeters(inches), 0.0);
        sim.update(DT);
    }

    /**
     * @return the motor's output current, as determined by the simulation
     * logic
     */
    @Override
    public double getCurrent() {
        return sim.getCurrentDrawAmps();
    }

    /**
     * @return true (the sim always acts as if it has the brake on)
     */
    @Override
    public boolean isBrakeEnabled() {
        return true;
    }

    /**
     * Toggles the motor brake
     */
    @Override
    public void applyBrake(boolean brake) {
        log("[elevator-sim] the sim does not model brake toggling");
    }

    /**
     * Apply motor voltage to the simulation
     */
    @Override
    public void applyVolts(double volts) {

        sim.setInputVoltage(volts);
        sim.update(DT);

        thisHeight = Units.metersToInches(sim.getPositionMeters());

        // for some reason the sim calculates the wrong velocity under
        // certain circumstances, so we'll just calculate it ourselves
        // based on the observed height of the elevator
        if (Double.isFinite(lastHeight)) {
            calculatedVelocity = (thisHeight - lastHeight) / DT;            
        }
        lastHeight = thisHeight;

        display.accept(thisHeight);
    }
}