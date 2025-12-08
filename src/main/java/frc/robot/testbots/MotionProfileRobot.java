package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MotionProfile;
import frc.robot.util.MotionProfile.State;

public class MotionProfileRobot extends TimedRobot {

    static final State NO_STATE = new State(0.0, 0.0, 0.0);

    XboxController controller;
    MotionProfile profile;
    double maxPosition;
    double maxVelocity;
    double maxAcceleration;
    double rampTime;
    Timer timer;
    State state;

    public MotionProfileRobot() {

        controller = new XboxController(0);
        profile = new MotionProfile();
        maxPosition = 50.0;
        maxVelocity = 10.0;
        maxAcceleration = 5.0;
        rampTime = 0.2;
        timer = new Timer();
        state = NO_STATE;

        SmartDashboard.putData("Motion", builder -> {
            builder.addDoubleProperty("Config/MaxPosition", () -> maxPosition, val -> maxPosition = val);
            builder.addDoubleProperty("Config/MaxVelocity", () -> maxVelocity, val -> maxVelocity = val);
            builder.addDoubleProperty("Config/MaxAcceleration", () -> maxAcceleration, val -> maxAcceleration = val);
            builder.addDoubleProperty("Config/RampTime", () -> rampTime, val -> rampTime = val);
    });

    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("Motion/Output/Position", state.position());
        SmartDashboard.putNumber("Motion/Output/Velocity", state.velocity());
        SmartDashboard.putNumber("Motion/Output/Acceleration", state.acceleration());
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
                state = NO_STATE;
            } else {
                profile.resetConstraints(maxVelocity, maxAcceleration, rampTime);
                profile.resetMotion(0.0, maxPosition);
                timer.restart();
            }
        }

        if (timer.isRunning()) {
            state = profile.sample(timer.get());
        }
    }





}
