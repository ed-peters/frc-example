# Shooter Subsystem

This is a subsystem for a high-speed shooter. A few learnings that are embedded here ...

**Shooters are like intakes**. Both are velocity-controller spinning mechanisms. And both have different relevant velocities as determined by gear ratios and wheel size.

**But ...** Shooters move at much higher speed, and feedback is usually based on the motor's relative encoder. This means it makes more sense to use hardware PID - it can react much more quickly (usually 1KHz instead of 50Hz) and be more accurate.
