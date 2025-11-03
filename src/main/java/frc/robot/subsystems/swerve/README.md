# Swerve Drive Subsystem

This is a subsystem for a swerve drive. A few learnings that are embedded here ...

**We implement the same stuff every year**. Kinematics, module state optimization, resetting poses, blah blah blah. It depends on the details of the hardware as configuration only. So it's easy to abstract out.

**Visualizing poses is critical for debugging**. This implementation includes the ability to publish arbitrary poses that you can then view in AdvantageScope. It's used inside `SwerveDriveSubsystem` and in some of the targeting commands in `frc.robot.subsystems.swerve`.

**Simulation is hella useful**. It's very useful to be able to run in the simulation to test command logic. This is much faster than waiting for the robot to be available and going through continuous deploy-test-fix cycles. The `SwerveChassisSim` does this for the drive.

**Rotating around objects is easy**. We've never actually used this but it's included here just because it's pretty easy to implement. See `SwerveDriveSubsystem::rotateAround`.