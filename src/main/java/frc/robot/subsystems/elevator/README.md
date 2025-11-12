# Elevator Subsystem

This is a subsystem for an elevator. A few learnings that are embedded here ...

**Knowing what your subsystems think they're doing is useful**. The subsystem includes a text field called `currentCommand` that is set whenever you're running a command. That gets displayed on the dashboard, and *poof!* you can easily tell which command is running for debugging.

**Running at max voltage can be dangerous**. The robot will have hard limits a the top and bottom; running at max voltage will quickly run into them and damage the robot or hurt people. We include multiple safety limits:
* Not setting the height below minimum or above maximum
* Not allowing velocity to carry us below minimum or above maximum
* Capping teleop input at a maximum value
* Capping feedback at a maximum value (see below)

**Mousetrapping is awkward**. In 2025 our arm and elevator would drift downwards when disabled because the motor brakes couldn't fully hold them in place. But the subsystem would "remember" their setpoints. So when you re-enabled them, the "hold still" logic would apply a high P correction and "snap" them back into place. This code implements a cap on feedback to help capture that.

**Holding still is complex**. See the comments in `frc.robot.commands.elevator` for more information. This implementation can "remember" a goal height to use in handoff between a trapezoidal motion command and a "holding still" command.

**Simulation is hella useful**. It's very useful to be able to run in the simulation to test command logic. This is much faster than waiting for the robot to be available and going through continuous deploy-test-fix cycles. The `ElevatorMotorSim` shows how to use WPILib's simulation libraries for this.

**Elevators and arms are essentially identical**. Both implement position-based control using feedforward and feedback, and benefit from using trapezoidal motion profiles to move to preset positions. The only major differences are (a) terminology, (b) the feedforward equation, and (c) the potential "wraparound" of an angle encoder. 

You can use this code as the basis for an arm, but it's probably best to have a separate copy. This is helpful when you're debugging or tweaking things - you don't want a temporary change/fix to arm code to accidentally break a working elevator. 