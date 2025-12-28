package frc.robot.testbots;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.swerve.SwerveAutoConfig;
import frc.robot.util.Util;
import frc.robot.util.motion.Motion;
import frc.robot.util.motion.Motions;

public class MotionProfileRobot extends TimedRobot {

    XboxController controller;
    Motion<State> profile;
    double maxPosition;
    double maxVelocity;
    double maxAcceleration;
    Timer timer;
    State state;

    public MotionProfileRobot() {

        controller = new XboxController(0);
        maxPosition = Math.PI;
        maxVelocity = Math.toRadians(SwerveAutoConfig.rotateMaxVelocity.getAsDouble());
        maxAcceleration =  Math.toRadians(SwerveAutoConfig.rotateMaxAcceleration.getAsDouble());
        timer = new Timer();
        state = Util.ZERO_STATE;

        SmartDashboard.putData("Motion", builder -> {
            builder.addDoubleProperty("Config/MaxPosition", () -> maxPosition, val -> maxPosition = val);
            builder.addDoubleProperty("Config/MaxVelocity", () -> maxVelocity, val -> maxVelocity = val);
            builder.addDoubleProperty("Config/MaxAcceleration", () -> maxAcceleration, val -> maxAcceleration = val);
        });

    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("Motion/Output/Position", state.position);
        SmartDashboard.putNumber("Motion/Output/Velocity", state.velocity);
    }

    @Override
    public void teleopPeriodic() {

        /*
         * If button A was pressed:
         *   - if a profile is running, stop it
         *   - otherwise,
         */
        if (controller.getAButtonReleased()) {
            if (timer.isRunning()) {
                timer.stop();
                state = Util.ZERO_STATE;
            } else {
                profile = Motions.trapezoid(
                        () -> maxVelocity,
                        () -> maxAcceleration,
                        0.0,
                        maxPosition);
                timer.restart();
            }
        }

        if (timer.isRunning()) {
            state = profile.sample(timer.get());
        }
    }
}
