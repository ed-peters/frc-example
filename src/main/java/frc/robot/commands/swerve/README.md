# Swerve Commands

These are commands that work with the `SwerveDriveSubsystem` to accomplish super-common tasks.
A few learnings that are embedded here ...

## On teleop ...

**We do it the same every year**. Field relative movement, turbo mode, sniper mode, etc. All this logic configuration and control mappings are embedded in the various `Teleop` classes.

**Drift correction might not be necessary**. We implemented it a few years ago when our swerve drive construction wasn't great, and the robot rotated a lot under motion. It's not entirely clear that it really impacts our accuracy in teleop. There's a flag to disable it.

**Slew limiting might be useful**. In 2025 we had a very tall and top-heavy robot. We never actually tipped it over, but fast acceleration could cause instability. We added slew limiting to deal with that.

## On targeting ...

**We got mileage from a three-step process**. 2025 was our best year for automated targeting. We did it in three steps: (a) turn the robot to face an AprilTag, (b) align the AprilTag within the camera frame, and (c) drive to a small offset of the aligned position. See `SwerveFullTargetingSequenceCommand`.

**There are many ways to drive to an offset**. We switched between using the WPILib trajectory support, and a very basic PID-based approach. This implementation tries to strike a balance and uses a `TrapezoidProfile` to drive along a straight line to a target position. See `SwerveTargetPoseOffsetCommand`.