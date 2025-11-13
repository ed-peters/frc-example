# Vision Commands

These are commands that work with vision implementations and the `SwerveDriveSubsystem` to accomplish super-common tasks. A few learnings that are embedded here ...

**Resetting the drive pose is insanely useful**. When you're setting up on a practice field, you need a way to tell the robot what it's pose is on the field. Doing this from an AprilTag is super-useful.

**Limelight wasn't accurate enough for fine-grained targeting**. For 2025 we needed to be within a couple of inches of a target position. We tried using the Limelight's estimate of the target's 3D pose (specifically, `targetpose_robotspace`) to figure out where to drive for scoring. It's always off a little bit and, unfortunately, not in a predictable way so it's very hard to correct for. We MIGHT have been able to fix this with some kind of tuning, but ...

**Basic camera targeting is pretty accurate**. By just using info about the target's position in the camera frame (X offset and pixel area) we were able to drive to a very predictable position in front of a tag. This was the basis for a good targeting algorithm.

**We got mileage from a three-step process**. 2025 was our best year for automated targeting. We did it in three steps:

* First you turn the robot to face the AprilTag. You do this by identifying the tag by number, looking up the pose of that tag on the field, and rotating to that heading. This can be done with the `SwerveRotateCommand`.


* Second, you align to the tag within the camera view. For our purpose, we wanted it centered (X offset = 0) and at a predictable distance (based on area). This is what's implemented in the `LimelightAprilTagCommand`.


* Finally, your "scoring position" may actually be out of view of the tag. So you add a little offset to the mix. This can be done with the `SwerveTranslateCommand`.
 
This full algorithm is implemented in the `ThreeStageTargetingCommand`.
