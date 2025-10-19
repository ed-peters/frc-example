/**
 * Implementation of the intake on the 2024 (Crescendo) bot.
 * It ran in one of three modes:
 * <ul>
 *
 *     <li>Pickup mode - running constantly at low speed to
 *     pick up a gamepiece (and would stop when a photo sensor
 *     told us we had one)</li>
 *
 *     <li>Feed mode - running at high speed for a short period
 *     of time to feed the gamepiece into a shooter</li>
 *
 *     <li>Eject mode - running backwards for a brief period
 *     of time to get rid of a piece stuck in the intake</li>
 *
 * </ul>
 *
 * We learned a few things from this:
 *
 * <ul>
 *
 *     <li>There are three separate speeds at play for a flywheel - the
 *     motor speed (rotations per second), the wheel speed (based on motor
 *     speed but influenced by the gear ratio) and the liner velocity
 *     of the contact point on the wheel (based on wheel speed but
 *     influenced by wheel size)</li>
 *
 *     <li>You will hit a point where the build team is looking at you
 *     and asking "can this thing go any faster?" To answer this, it's
 *     useful to know (a) how many volts you're applying to the motor and
 *     (b) how fast the motor is spinning. Make sure to log those.</li>
 *
 *     <li>For this robot, the intake had to feed a gamepiece to a shooter,
 *     and we wanted to sync up their speeds to avoid damaging them. This
 *     meant synchronizing the linear speeds for two different wheels with
 *     different sizes and gear ratios. So we also wanted to log those and
 *     pay attention to them.</li>
 *
 *     <li>Closed loop control is really best; when you don't have an exact
 *     target speed, it can be tempting to just apply a voltage and do it all
 *     by trial-and-error, but the mechanism will slow down when it hits
 *     resistance and it's hard to synchronize speed.</li>
 *
 *     <li>Tuning closed loop for a flywheel is really easy. Set it to a
 *     constant velocity and see how close it gets. Tune the V parameter
 *     for feedforward first - you should be able to get to ~1% of the
 *     setpoint with only that. Then use P and D to help you get there
 *     faster and close the last bit of error.</li>
 *
 * </ul>
 */
package frc.robot.subsystems.intake;