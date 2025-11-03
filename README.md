# FRC Examples

This is heavily-commented example code for implementing different common subsystems for FRC robots. It embeds some hard-won experience gathered over a few years of competition.

It includes:

* **Utils** - some super-useful utility code that we wind up implementing year after year. See
  * `frc.robot.util`


* **Strap** - a very simple subsystem for driving open-loop behavior, with an implementation of "stall detection" that we found useful one year. See
  * `frc.robot.subsystems.strap`


* **Intake** - a flywheel based subsystem, with support for closed loop based on rotation speed or linear speed, and some examples of using sensors
  * `frc.robot.subsystems.intake`
  * `frc.robot.commands.intake`


* **Elevator** - a position based subsystem, with support for tuning and driving quickly and smoothly to preset positions using motion profiles
  * `frc.robot.subsystems.elevator`
  * `frc.robot.commands.elevator`


* **Swerve** - a swerve subsystem with commands for teleop mode (with a bunch of options we use every year) and various automatic targeting actions
  * `frc.robot.subsystems.swerve`
  * `frc.robot.commands.swerve`
