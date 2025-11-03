# FRC Examples

This is heavily-commented example code for implementing different common subsystems for FRC robots. It embeds some hard-won experience gathered over a few years of competition.

It includes:

* **Strap** - a very simple subsystem for driving open-loop behavior, with an implementation of "stall detection" that we found useful one year.


* **Intake** - a flywheel based subsystem, with support for closed loop based on rotation speed or linear speed, and some examples of using sensors


* **Elevator** - a position based subsystem, with support for tuning and driving quickly and smoothly to preset positions using motion profiles


* **Swerve** - a swerve subsystem with commands for teleop mode (with a bunch of options we use every year) and various automatic targeting actions