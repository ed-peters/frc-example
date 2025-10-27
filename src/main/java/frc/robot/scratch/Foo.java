package frc.robot.scratch;

import edu.wpi.first.math.controller.ProfiledPIDController;

public class Foo {

    ProfiledPIDController pid;

    public void foo() {
        pid.calculate(1.0, null, null);
    }

}
