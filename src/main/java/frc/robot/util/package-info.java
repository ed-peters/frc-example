/**
 * Includes various utility functions that come in handy in certain
 * situations. Key learnings that are encapsulated here:
 *
 * <ul>
 *
 *     <li>Preferences are the bomb; they allow you to tweak values
 *     like PID constants from the dashboard, and have them saved
 *     across robot restarts; see
 *     {@link frc.robot.util.Util#pref(java.lang.String, double)}</li>
 *     for an example</li>
 *
 *     <li>It's useful to have some constants defined for stuff
 *     like "a pose with 0 values", so you don't keep creating new
 *     objects all the time; see {@link frc.robot.util.Util#ZERO_POSE}
 *     for an example</li>
 *
 *     <li>It's useful to be able to view your robot's pose in
 *     AdvantageScope; doing that requires publishing them as structs
 *     which takes a little work; see
 *     {@link frc.robot.util.Dash#publish(java.lang.String, edu.wpi.first.math.geometry.Pose2d)}
 *     for the code</li>
 *     
 *     <li>Values like motor position are reported with a LOT of digits
 *     of precision, and change really quickly beyond about three digits;
 *     that's visual clutter on the dashboard and it's useful to get
 *     rid of it; see {@link frc.robot.util.Util#chopDigits(double)}</li>
 *
 *     <li>You can display little "widgets" on the dashboard to visualize
 *     the state of a mechanism when you're operating in simulation mode;
 *     see {@link frc.robot.util.Dash#createElevatorWidget(java.lang.String)}
 *     for an example</li>
 *
 * </ul>
 */
package frc.robot.util;