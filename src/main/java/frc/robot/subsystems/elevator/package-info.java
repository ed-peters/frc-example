/**
 * Implementation of the elevator on the 2025 (Reefscape) bot. It
 * moved between various preset height for scoring.
 *
 * We learned a few things from this:
 * <ul>
 *
 *     <li>Trapezoidal motion control is really the bomb - it lets
 *     you go quickly but smoothly between two positions. This is
 *     important for heavy things like an elevator, to avoid jerking
 *     them (which can cause damage and be inaccurate)</li>
 *
 *     <li>In this situation, your "setpoint" in the subsystem is
 *     different than the "goal" applied by the command. It's important
 *     to be able to tell when you're at a "goal" height. See
 *     {@link frc.robot.subsystems.elevator.ElevatorSubsystem#isAt(double)}</li>
 *
 *     <li>When you do switch from moving to a preset height, to holding
 *     still at the "current" height, you can wind up with a little "droop".
 *     This is because the elevator sags for a split second as the command
 *     processing loop does its thing. You solve this by "remembering" the
 *     target height - see
 *     {@link frc.robot.commands.elevator.ElevatorHoldCommand#initialize()}
 *     to see how this works</li>
 *
 *     <li>If you only use P for feedback, you can wind up overshooting
 *     your target position a little bit. You want to use D so you apply
 *     feedback based on how fast you're moving.</li>
 *
 *     <li>It's really useful for debugging to be know what the subsystem
 *     "thinks" it's doing. This is why we capture a <code>currentCommand</code>
 *     when we're operating in both open- and closed-loop, and display
 *     it on the dashboard.</li>
 *
 * </ul>
 */
package frc.robot.subsystems.elevator;