package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
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
                ? new DashboardPicker("AutonomousProgram", getProgramNames())
                : new DigitBoardPicker(getProgramNames());

        this.selected = programPicker.get();
        this.error = false;

        SmartDashboard.putData(getName(), builder -> {
            builder.addStringProperty("Program", () -> selected, null);
            builder.addBooleanProperty("Running?", () -> command != null && command.isScheduled(), null);
            builder.addBooleanProperty("Error?", () -> error, null);
        });
    }

    // ==========================================================
    // STANDARD STUFF
    // ==========================================================

    /**
     * Call this from {@link TimedRobot#autonomousInit()} to create the
     * actual command. You should do all the work of loading the program
     * and configuring PathPlanner here, so the field operator doesn't
     * have to wait for it to happen.
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
            command = decorateAutoCommand(command);

        } catch (Exception e) {

            // this handles logging
            Util.log("[auto] ERROR LOADING PROGRAM!!!");
            e.printStackTrace();

            // this sets the "emergency start pose" and program
            drive.resetPose(createEmergencyStartPose());
            command = createEmergencyCommand();
        }

        return command;
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

    // ==========================================================
    // THINGS FOR YOU TO DO
    // ==========================================================

    /*
     * TODO identify all your programs
     *
     * This is the list of all the autonomous programs we have to choose
     * from.
     *
     * There should be one entry for each different program. The value on
     * the left is a "short name" (which gets displayed on the DigitBoard)
     * and the value on the right is the actual filename of the program
     * that you edited using PathPlanner.
     */
    private Map<String,String> getProgramNames() {

        // we use a LinkedHashMap so the programs will be shown in the
        // same order as below
        Map<String,String> programs = new LinkedHashMap<>();
        programs.put("EX01", "Example1");
        programs.put("EX02", "Example2");
        programs.put("EX03", "Example3");
        return programs;
    }

    /*
     * TODO create all your named commands
     *
     * This is where you register all the "named commands" that are used
     * by your different autonomous programs
     */
    private void registerNamedCommands() {

        Util.log("[auto] Registering named commands");

        // you will have to create commands that do real things like
        // shooting and scoring if you want to use them in your autos
        NamedCommands.registerCommand("DoSomething1", Commands.print("*** thing 1 ***"));
        NamedCommands.registerCommand("DoSomething2", Commands.print("*** thing 2 ***"));
        NamedCommands.registerCommand("DoSomething3", Commands.print("*** thing 3 ***"));
    }

    /*
     * TODO do you need to "decorate" your auto program?
     *
     * In previous years we've had to do some "mandatory" startup tasks,
     * like lowering an arm or moving the position of a held gamepiece.
     * We did this by taking the Command created by PathPlanner and adding
     * stuff to the beginning or end.
     */
    private Command decorateAutoCommand(Command autoCommand) {
        Command before = Commands.print("[auto] starting auto");
        Command after = Commands.print("[auto] ending auto");
        return before.andThen(autoCommand).andThen(after);
    }

    /*
     * TODO create an emergency auto program
     *
     * If the autonomous routine gets buggered up, what do you want the
     * robot to do? Sitting still might be one option, but you might also
     * have some of that "mandatory" stuff to do. Or you might want to try
     * some minimal driving to score points.
     */
    private Command createEmergencyCommand() {
        return Commands.print("[auto] oh, dang, something went wrong!");
    }

    /*
     * TODO create an "emergency" start pose
     *
     * The robot gets its starting pose from the autonomous path. If there
     * isn't one, what should you do? You probably at least want to consider
     * guessing the heading of the robot, so the driver has some little bit
     * of control and can e.g. get to an AprilTag and reset position from
     * there using the Limelight.
     */
    private Pose2d createEmergencyStartPose() {

        // this assumes we always start facing away from whichever driver
        // station we're at
        return Util.isBlueAlliance()
                ? new Pose2d(0.0, 0.0, Rotation2d.k180deg)
                : Util.ZERO_POSE;
    }
}
