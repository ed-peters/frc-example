# Intake Subsystem

This is a subsystem for an intake. A few learnings that are embedded here ...

**Knowing what your subsystems think they're doing is useful**. The subsystem includes a text field called `currentCommand` that is set whenever you're running a command. That gets displayed on the dashboard, and *poof!* you can easily tell which command is running for debugging.

**Spinning wheels are all pretty similar**. This is a simple flywheel subsystem; it could probably be used as the basis for e.g. a shooter as well. The key is velocity-based tuning.

**Max voltage is probably safe**. Systems like elevators, arms or climbers are usually constrained by hard stops. Running them at maximum voltage may cause damage to the robot when they smack into their limits. On the other hand, spinning wheels are meant to spin freely, so running them at top speed usually isn't directly damaging to the robot. You'll see less "safety limit" code in here for that reason. 

**There are multiple "speeds"**. The motor speed determines the wheel speed, dependent on the gear ratio. The wheel speed determines how "fast" a piece moves through the system, dependent on the radius of the attached wheel. In 2024, we had to match the speed of two different-sized subsystems in feet per second to avoid damaging a gamepiece. This subsystem allows control using any of the three speeds.


