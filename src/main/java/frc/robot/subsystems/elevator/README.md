# Elevator Subsystem

This is a subsystem for an elevator. A few learnings that are embedded here ...

**Elevators and arms are essentially identical**. Both implement position-based control using feedforward and feedback, and benefit from using trapezoidal motion profiles to move to preset positions. The only major differences are (a) terminology, (b) the feedforward equation, and (c) the potential "wraparound" of an angle encoder. You can use this code as the basis for an arm.

**Holding still is complex**. See the comments in `frc.robot.commands.elevator` for more information. This implementation can "remember" a goal height to use in handoff between a trapezoidal motion command and a "holding still" command.

**Mousetrapping is awkward**. In 2025 our arm and elevator would drift downwards when disabled because the motor brakes couldn't fully hold them in place. But the subsystem would "remember" their setpoints. So when you re-enabled them, the "hold still" logic would apply a high P correction and "snap" them back into place. This code implements a cap on feedback to help capture that.

**Simulation is hella useful**. It's very useful to be able to run in the simulation to test command logic. This is much faster than waiting for the robot to be available and going through continuous deploy-test-fix cycles. The `ElevatorMotorSim` shows how to use WPILib's simulation libraries for this.
