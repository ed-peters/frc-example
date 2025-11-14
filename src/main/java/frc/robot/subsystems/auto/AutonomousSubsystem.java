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

    public AutonomousSubsystem(SwerveDriveSubsystem drive) {

        this.drive = drive;

        // in 2025 we loaded all the different programs up front. this
        // got slower and slower as we added more complicated options.
        // eventually the operator would have to stand on the field
        // waiting for a minute or more before they could make their
        // program selection. they got yelled at a lot.

        // don't let this happen to you! only create the picker of
        // names here; wait to load the command until the game is
        // getting underway.

        this.programPicker = RobotBase.isSimulation()
                ? new DashboardPicker("AutonomousProgram", programs.keySet())
                : new DigitBoardPicker(programs.keySet());

        this.selected = programPicker.get();

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Program", () -> selected, null);
            builder.addBooleanProperty("Running?", () -> command != null && command.isScheduled(), null);
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

        registerNamedCommands();
        configureAutoBuilder();

        Util.log("[auto] loading program %s", selected);

        // this loads the actual program description and links it
        // up with all the other configurations
        command = new PathPlannerAuto(programs.get(selected));

        // in years past we might mess with the program once it was
        // loaded - for instance, prepending some initialization code
        // or adding some timeouts or other such hackery
        
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
    private void configureAutoBuilder() {

        // this loads the settings for the robot, which you edited and
        // saved through the GUI
        RobotConfig config = null;
        try {
            Util.log("[auto] loading robot configuration");
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

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
