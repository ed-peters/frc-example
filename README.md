# FRC Examples

This is heavily-commented example code for implementing different common subsystems for FRC robots. It embeds some hard-won experience gathered over a few years of competition.

It includes:

* **Utils** - some super-useful utility code that we wind up implementing year after year. See
  * [`frc.robot.util`](src/main/java/frc/robot/util)


* **Strap** - a very simple subsystem for driving open-loop behavior, with an implementation of "stall detection" that we found useful one year. See
  * [`frc.robot.subsystems.strap`](src/main/java/frc/robot/subsystems/strap)


* **Intake** - a flywheel based subsystem, with support for closed loop based on desiredRotation speed or linear speed, and some examples of using sensors
  * [`frc.robot.subsystems.intake`](src/main/java/frc/robot/subsystems/intake)
  * [`frc.robot.commands.intake`](src/main/java/frc/robot/commands/intake)


* **Shooter** - another flywheel based subsystem which uses hardware PID instead of software
  * [`frc.robot.subsystems.shooter`](src/main/java/frc/robot/subsystems/shooter)
  * [`frc.robot.commands.shooter`](src/main/java/frc/robot/commands/shooter)


* **Elevator** - a position based subsystem, with support for tuning and driving quickly and smoothly to preset positions using motion profiles
  * [`frc.robot.subsystems.elevator`](src/main/java/frc/robot/subsystems/elevator)
  * [`frc.robot.commands.elevator`](src/main/java/frc/robot/commands/elevator)


* **Swerve** - a swerve subsystem with commands for teleop mode (with a bunch of options we use every year) and various automatic movement actions
  * [`frc.robot.subsystems.swerve`](src/main/java/frc/robot/subsystems/swerve)
  * [`frc.robot.commands.swerve`](src/main/java/frc/robot/commands/swerve)


* **Vision** - vision subsystems for Limelight and QuestNav with commands for resetting the robot's pose and tag-based targeting 
  * [`frc.robot.subsystems.vision`](src/main/java/frc/robot/subsystems/vision)
  * [`frc.robot.commands.vision`](src/main/java/frc/robot/commands/vision)


* **Auto** - an autonomous "subsystem" which shows how we pick and generate routines with a dedicated hardware component
  * [`frc.robot.subsystems.auto`](src/main/java/frc/robot/subsystems/auto)


