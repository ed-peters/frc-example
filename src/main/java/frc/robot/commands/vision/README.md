# Vision Commands

These are commands that work with vision implementations and a swerve drive to accomplish super-common tasks. A few learnings that are embedded here ...

**Odometry alone wasn't accurate enough for fine-grained targeting**. For 2025 we needed to be within a couple of inches of a target position. We tried using the Limelight's estimate of the target's 3D pose (specifically, `targetpose_robotspace`) to figure out where to drive for scoring. It's always off a little bit and, unfortunately, not in a predictable way so it's very hard to correct for. We MIGHT have been able to fix this with some kind of tuning, but ...

**Basic camera targeting is pretty accurate**. By just using info about the target's position in the camera frame (X offset and pixel area) we were able to drive to a very predictable position in front of a tag. This was the basis for a good targeting algorithm. Limelight calls this approach [visual servoing](https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing).

**We got mileage from a three-step process**. 2025 was our best year for automated targeting. We did it in three steps:

* First turn the robot to face the AprilTag. You do this by identifying the tag by number, looking up the pose of that tag on the field, and rotating to that heading.


* Second, you align to the tag within the camera view. For our purpose, we wanted it centered (X offset = 0) and at a predictable distance (based on area).


* Finally, your "scoring position" may actually be out of view of the tag. So you add a little offset to the mix.
 
This full algorithm is implemented in [`TargetingCommandBuilder.java`](TargetingCommandBuilder.java) But ...
 
**You can do just a bit better**. If you aren't directly in front of the tag, when you turn, you could lose sight of it. A slightly better approach is the following three steps:

* Drive to a position in front of the tag and facing it; you can get the position pretty close, and the heading very close;


* Make your alignment exact via visual servoing; and then,


* Scoot to the offset.

This "swanky" algorithm is implemented in [`TargetingCommandBuilder.java`](TargetingCommandBuilder.java)

**It doesn't hurt to be cautious**. This logic will stop driving if it loses sight of the tag. The Limelight folks recommend implementing some kind of "search" algorithm, but we aren't using that.
 