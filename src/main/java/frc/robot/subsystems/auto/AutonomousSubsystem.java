package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Util;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import static frc.robot.subsystems.auto.AutonomousConfig.d;
import static frc.robot.subsystems.auto.AutonomousConfig.p;

public class AutonomousSubsystem extends SubsystemBase {

    /*
     * This is the list of all the autonomous programs we have to choose
     * from.
     *
     * There should be one line for each different program. The value on
     * the left is a "short name" (which gets displayed on the DigitBoard)
     * and the value on the right is the actual filename of the program
     * that you edited using PathPlanner.
     *
     * These options will appear in the order they are shown below.
     */
    public static Map<String,String> programs = new LinkedHashMap<>();
    static {
        programs.put("LL", "Leave Left");
        programs.put("LM", "Leave Middle");
        programs.put("LR", "Leave Right");
    }

    final SwerveDriveSubsystem drive;
    final Supplier<String> programPicker;
    String selected;
    Command command;
    boolean error;

    public AutonomousSubsystem(SwerveDriveSubsystem drive) {

        this.drive = drive;

        // we decide which type of picker to use depending on whether
        // we're in simulation or not; either way, our picker will
        // return the long name of the currently-selected program

        this.programPicker = RobotBase.isSimulation()
                ? new DashboardPicker("AutonomousProgram", programs)
                : new DigitBoardPicker(programs);

        this.selected = programPicker.get();
        this.error = false;

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Program", () -> selected, null);
            builder.addBooleanProperty("Running?", () -> command != null && command.isScheduled(), null);
            builder.addBooleanProperty("Error?", () -> error, null);
        });
    }

    /**
     * This will be invoked to generate the actual command. You should do
     * all the work of loading the program and configuring PathPlanner
     * here, so the operator doesn't have to wait for it to happen.
     */
    public Command generateCommand() {

        if (selected == null) {
            Util.log("[auto] NO SELECTED PROGRAM!!!");
            return Commands.none();
        }

        // there's a lot happening under the covers here, and PathPlanner
        // might generate an error if there's a problem with config files,
        // or calculating a path, or something like that.

        // we don't want the entire robot to crash if there's a problem
        // with autonomous, so we wrap this whole thing with an exception
        // handler. if there is an error, we will log it and the robot
        // will just sit still during autonomous.

        // TODO how should we set the starting position if an error happens?

        try {

            registerNamedCommands();
            configureAutoBuilder();

            Util.log("[auto] loading program %s", selected);

            // this loads the actual program description and links it
            // up with all the other configurations
            command = new PathPlannerAuto(selected);

            // in years past we might mess with the program once it was
            // loaded - for instance, prepending some initialization code
            // or adding some timeouts or other such hackery

        } catch (Exception e) {
            Util.log("[auto] ERROR LOADING PROGRAM!!!");
            e.printStackTrace();
            command = Commands.none();
        }

        return command;
    }

    /**
     * This is where you register all the "named commands" that are used
     * by your different autonomous programs
     */
    private void registerNamedCommands() {

        Util.log("[auto] Registering named commands");

        // TODO these should obviously be replaced with real commands that do stuff
        NamedCommands.registerCommand("RaiseElevatorL1", Commands.none());
        NamedCommands.registerCommand("RaiseElevatorL2", Commands.none());
        NamedCommands.registerCommand("RaiseElevatorL3", Commands.none());
        NamedCommands.registerCommand("RaiseElevatorL4", Commands.none());
        NamedCommands.registerCommand("IntakeCoral", Commands.none());
        NamedCommands.registerCommand("ScoreNWL", Commands.none());
        NamedCommands.registerCommand("ScoreNWR", Commands.none());
        NamedCommands.registerCommand("Eject", Commands.none());
    }

    /**
     * Configure PathPlanner's command builder
     */
    private void configureAutoBuilder() throws IOException, ParseException {

        // this loads the settings for the robot, which you edited and
        // saved through the GUI
        Util.log("[auto] loading robot configuration");
        RobotConfig config = RobotConfig.fromGUISettings();

        // this is what accepts calculated speeds from the path engine
        // and actually applies them to drive the robot around; if the
        // robot isn't moving at all, start debugging here
        BiConsumer<ChassisSpeeds,DriveFeedforwards> output = (s, f) -> {
            SmartDashboard.putNumber("PP_X", s.vxMetersPerSecond);
            SmartDashboard.putNumber("PP_Y", s.vyMetersPerSecond);
            drive.drive("auto", s);
        };

        // this is used to provide feedback to keep the robot on the
        // supplied course; if the robot isn't moving accurately, start
        // debugging here
        PathFollowingController controller = new PPHolonomicDriveController(
                new PIDConstants(p.getAsDouble(), 0.0, d.getAsDouble()),
                new PIDConstants(p.getAsDouble(), 0.0, d.getAsDouble())
        );

        Util.log("[auto] configuring AutoBuilder");
        AutoBuilder.configure(
                drive::getFusedPose,
                drive::resetPose,
                drive::getCurrentSpeed,
                output,
                controller,
                config,
                () -> !Util.isBlueAlliance(),
                this);
    }

    @Override
    public void periodic() {
        selected = programPicker.get();
    }
}
