package frc.robot.testbots;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorPreset;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorMotorSim;
import frc.robot.util.Util;

/**
 * Implementation of {@link TimedRobot} that can be used to test
 * elevator-related commands or practice tuning in the simulator.
 */
public class ElevatorSimRobot extends TimedRobot {

    CommandXboxController controller;
    ElevatorSubsystem elevator;
    ElevatorMotorSim sim;

    public ElevatorSimRobot() {

        sim = new ElevatorMotorSim();
        elevator = new ElevatorSubsystem(sim);
        controller = new CommandXboxController(0);

        // pick one of these depending on what you're doing
        mapCommandsForTuning();
//        mapCommandsForPresets();

        // when using the simulator to practice tuning, you may want a way
        // to set the virtual elevator to testing height
        controller.y().onTrue(elevator.runOnce(() -> {
            Util.log("setting sim height");
            sim.setHeight(70.0);
        }));
    }

    /**
     * This maps the controls so that you can easily tune the mechanism
     * (see {@link frc.robot.commands.elevator.ElevatorTuningCommand}
     * for comments on the tuning procedure.
     */
    private void mapCommandsForTuning() {

        // default behavior - apply 0 volts (make sure someone is ready
        // to catch the mechanism when it falls)
        elevator.setDefaultCommand(elevator.releaseCommand());

        // this applies the tuning velocity; you will run this a bunch
        // of times with new values of your tuning constants until you
        // get the behavior you want
        controller.a().whileTrue(elevator.tuningCommand());

        // once you've tuned G, this command should work to hold the
        // elevator still (test it to be sure)
        controller.b().whileTrue(elevator.holdCommand());
    }

    /**
     * This maps the controls so you can use them to tune max velocity
     * and acceleration, and preset heights.
     */
    private void mapCommandsForPresets() {

        // default behavior is to hold still (this will only work
        // once you've tuned G)
        elevator.setDefaultCommand(elevator.holdCommand());

        // moves the elevator between different preset heights; you can
        // tweak the heights and the max velocity and acceleration to see
        // how fast / accurate your tuning is
        controller.a().onTrue(elevator.moveToHeightCommand(ElevatorPreset.L1));
        controller.b().onTrue(elevator.moveToHeightCommand(ElevatorPreset.L3));

        // makes it easy to drop the elevator to the bottom if you need
        // to during testing
        controller.x().whileTrue(elevator.releaseCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
