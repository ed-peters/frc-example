# Swerve Commands

These are commands that work with the `SwerveDriveSubsystem` to accomplish super-common tasks. A few learnings that are embedded here ...

**We do teleop the same every year**. Field relative movement, turbo mode, sniper mode, etc. All this logic configuration and control mappings are embedded in the various `Teleop` classes.

**Drift correction might not be necessary**. We implemented it a few years ago when our swerve drive construction wasn't great, and the robot rotated a lot under motion. It's not entirely clear that it really impacts our accuracy in teleop. There's a flag to disable it.

**Slew limiting might be useful**. In 2025 we had a very tall and top-heavy robot. We never actually tipped it over, but fast acceleration could cause instability. We added slew limiting to deal with that.

**Rotating is hella useful**. Being able to rotate quickly to a fixed heading is
pretty useful. The most common case is aligning to some field fixture, like an
arena wall, a scoring or feeding piece, or the like.

**There are many ways to drive to an offset**. We switched between using the WPILib trajectory support, and a very basic PID-based approach. This implementation tries to strike a balance and uses a `TrapezoidProfile` to drive along a straight line to a target position. See `SwerveTargetPoseOffsetCommand`.