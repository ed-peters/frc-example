# Elevator Commands

These are commands that work with the `ElevatorSubsystem` to accomplish super-common tasks.
A few learnings that are embedded here ...

**You don't want to do teleop in a competition**. It's just not accurate enough when you're stressed out. You're much better off tuning your elevator and using a `TrapezoidProfile` to drive the elevator to a goal height.

**Holding still is a tad tricky**. When you drive to a target height, you will come close. If you then switch to holding at "the current position", you may be holding steady at a slightly wrong height. You need your hold command to "remember" the target height from your trapezoid command, and use that.

**Tuning is pretty easy**. We got very accurate using only P, D, G and V constants, and doing a slightly abbreviated version of the recommended tuning approach in WPILib docs. See the `ElevatorTuningCommand` for more information.

**D can be important**. We had an issue with our elevator slightly overshooting its target. D helps you come in for a "smooth landing". See `ElevatorTrapezoidCommand` for some comments.

**Arms are really similar**. This is true for the subsystem (see the README for `ElevatorSubsystem`), but even more so for the commands - they are basically identical. Just like for the subsystem, though, it's probably worth having separate versions of the commands for the arm.