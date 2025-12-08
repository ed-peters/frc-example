# Vision Subsystem

Subsystems for vision stuff with both the Limelight and the Quest headset. A few learnings that are embedded here ...

*Limelight and Questnav do different stuff*. Limelight "trues up" your position periodically with AprilTags, and provides realtime targeting for specific positioning tasks. Questnav keeps track of where you have moved from your starting position (it's like much better odometry).

*They are both pretty simple*. They publish data via NetworkTables that you can use for different stuff. The Questnav needs to know when the swerve drive pose gets reset, which is a little quirk.

*Limited simulation is doable*. Since the interface to the vision hardware is through NT, it's possible to mock up some basic targeting features for testing in the simulator. See the [`LimelightSim.java`](LimelightSim).

*Stable camera mounts are important*. Accurate targeting depends on knowing exactly where the camera is relative to the center of the robot. The angle is particularly important - in previous years ours would bend with impacts, which introduced error.

*Limelight coordinate systems are awkward*. The [Limelight coordinate systems](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems), don't match up at all with the WPILib systems.