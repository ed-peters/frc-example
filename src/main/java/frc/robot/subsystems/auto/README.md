# Autonomous Subsystem

This is a subsystem for managing the selection of autonomous programs.
A few learnings that are embedded here ...

*Don't use the Dashboard in competitions*. WPILib code samples will show you
how to use a `SendableChooser` to pick programs from the dashboard. The problem
with this is that, if the dashboard app crashes before the match, you might not
be able to do that. (This was a real problem for us one year.)

*The DigitBoard is awesome*. Our solution was to use a [REV Digit Board](https://www.revrobotics.com/rev-11-1113/) to allow an on-field operator to
pick which program to run. The code in here uses the `DashboardPicker` during simulation, and the `DigitBoardPicker` for the "real" robot.

*Don't load all your available routines at once*. We used to create a 
`Command` object for each available routine, and then pick which one to use at the beginning of autonomous. This works OK if you have just a few programs but creating those things take a fair amount of time. When we got up to >3 programs, it can slow down the startup time of the robot a lot. The on-field operator will get yelled at by the judges to hurry up and it's awkward. So this code waits to create the actual `Command` until it's needed.


