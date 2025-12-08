package frc.robot.util;

/**
 * Profiles a one-dimensional "motion profile" - calculates position and
 * velocity over time respecting constraints on velocity, acceleration and
 * ramp time.</p>
 *
 * In the degenerate case where the ramp time is 0, this is essentially the
 * same as the {@link edu.wpi.first.math.trajectory.TrapezoidProfile}. When
 * a ramp time is specified, motion follows an "S-Curve" trajectory with up
 * to 7 phases of motion.
 */
public class MotionProfile {

    /**
     * Captures the expected position, velocity and acceleration at a moment
     * in time
     */
    public record State(double position, double velocity, double acceleration) { }

    // Constraints on motion
    private double maxVelocity = Double.NaN;
    private double maxAcceleration = Double.NaN;
    private double rampTime = Double.NaN;

    // Parameters for an individual motion
    private double startPosition = Double.NaN;
    private double startVelocity = Double.NaN;
    private double finalPosition = Double.NaN;

    // Calculated profile timing
    private double totalTime = Double.NaN;
    private double actualMaxVelocity = Double.NaN;
    private boolean isReversed = false;

    // Phase timing (for S-curve profile with up to 7 phases)
    private double t1;  // End of initial jerk phase (acceleration ramp up)
    private double t2;  // End of constant acceleration phase
    private double t3;  // End of jerk phase (acceleration ramp down)
    private double t4;  // End of cruise phase
    private double t5;  // End of jerk phase (deceleration ramp up)
    private double t6;  // End of constant deceleration phase
    private double t7;  // End of jerk phase (deceleration ramp down) - total time

    // =============================================================
    // CONSTRAINTS
    // =============================================================

    /**
     * Resets the constraints for this profile. This will default the ramp
     * time to 0 seconds, which implies a trapezoidal motion profile.
     *
     * @param maxVelocity maximum velocity in units per second
     * @param maxAcceleration maximum acceleration in units per second squared
     * @throws IllegalArgumentException if velocity is <= 0
     * @throws IllegalArgumentException if acceleration is <= 0
     */
    public void resetConstraints(double maxVelocity, double maxAcceleration) {
        resetConstraints(maxVelocity, maxAcceleration, 0.0);
    }

    /**
     * Resets the constraints for this profile.
     *
     * @param maxVelocity maximum velocity in units per second
     * @param maxAcceleration maximum acceleration in units per second squared
     * @param rampTime time in seconds to reach maximum acceleration
     * @throws IllegalArgumentException if velocity is <= 0
     * @throws IllegalArgumentException if acceleration is <= 0
     * @throws IllegalArgumentException if ramp time is < 0
     */
    public void resetConstraints(double maxVelocity, double maxAcceleration, double rampTime) {

        if (maxVelocity <= 0) {
            throw new IllegalArgumentException("maxVelocity must be positive");
        }
        if (maxAcceleration <= 0) {
            throw new IllegalArgumentException("maxAcceleration must be positive");
        }
        if (rampTime < 0) {
            throw new IllegalArgumentException("rampTime must be non-negative");
        }

        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.rampTime = rampTime;
    }

    // =============================================================
    // PARAMETERS & CALCULATION
    // =============================================================

    /**
     * Resets the motion parameters. This assumes the starting velocity is 0,
     * and the final velocity is 0.
     *
     * @param startPosition starting position in units
     * @param finalPosition final position in units
     */
    public void resetMotion(double startPosition, double finalPosition) {
        resetMotion(startPosition, 0.0, finalPosition);
    }

    /**
     * Resets the motion parameters. This assumes the final velocity is 0.
     *
     * @param startPosition starting position in units
     * @param startVelocity starting velocity in units per second
     * @param finalPosition final position in units
     */
    public void resetMotion(double startPosition, double startVelocity, double finalPosition) {

        this.startPosition = startPosition;
        this.startVelocity = startVelocity;
        this.finalPosition = finalPosition;

        double distance = finalPosition - startPosition;
        isReversed = distance < 0;
        if (isReversed) {
            distance = -distance;
        }

        double initialVelocity = isReversed ? -startVelocity : startVelocity;
        if (initialVelocity < 0) {
            initialVelocity = 0; // Can't move backwards from start
        }

        if (rampTime == 0) {
            calculateTrapezoidalProfile(distance, initialVelocity);
        } else {
            calculateSCurveProfile(distance, initialVelocity);
        }
    }

    /*
     * Calculates the phase timings and actual maximum velocity for the case
     * when ramp time is 0 (this is a standard trapezoid profile)
     */
    private void calculateTrapezoidalProfile(double distance, double initialVelocity) {

        // Trapezoidal profile: constant acceleration phases
        // Phase 1: accelerate to max velocity (or less if distance is short)
        // Phase 2: cruise at max velocity
        // Phase 3: decelerate to final position

        // Distance needed to decelerate from current velocity to 0
        double decelerationDistance = initialVelocity * initialVelocity / (2 * maxAcceleration);

        // Distance needed to accelerate to max velocity and decelerate
        double triangleDistance = maxVelocity * maxVelocity / (2 * maxAcceleration)
                - initialVelocity * initialVelocity / (2 * maxAcceleration)
                + maxVelocity * maxVelocity / (2 * maxAcceleration);

        // Can't even maintain current velocity, must decelerate immediately
        if (distance < decelerationDistance) {
            actualMaxVelocity = Math.sqrt(2 * maxAcceleration * distance + initialVelocity * initialVelocity);
            t1 = 0;
            t2 = 0;
            t3 = 0;
            t4 = 0;
            t5 = 0;
            t6 = 0;
            t7 = (actualMaxVelocity - initialVelocity) / maxAcceleration;
        }

        // Triangle profile: accelerate then decelerate, no cruise
        else if (distance < triangleDistance) {
            actualMaxVelocity = Math.sqrt(maxAcceleration * distance + initialVelocity * initialVelocity / 2);
            t1 = 0;
            t2 = (actualMaxVelocity - initialVelocity) / maxAcceleration;
            t3 = t2;
            t4 = t2;
            t5 = t2;
            t6 = t2 + actualMaxVelocity / maxAcceleration;
            t7 = t6;
        }

        // Full trapezoidal profile: accelerate, cruise, decelerate
        else {
            actualMaxVelocity = maxVelocity;
            double accelTime = (maxVelocity - initialVelocity) / maxAcceleration;
            double decelTime = maxVelocity / maxAcceleration;
            double accelDistance = (maxVelocity + initialVelocity) / 2 * accelTime;
            double decelDistance = maxVelocity / 2 * decelTime;
            double cruiseDistance = distance - accelDistance - decelDistance;
            double cruiseTime = cruiseDistance / maxVelocity;

            t1 = 0;
            t2 = accelTime;
            t3 = t2;
            t4 = t2 + cruiseTime;
            t5 = t4;
            t6 = t4 + decelTime;
            t7 = t6;
        }

        totalTime = t7;
    }

    /*
     * Calculates the phase timings and actual maximum velocity for the case
     * when ramp time is 0 (this is full S-Curve profile)
     */
    private void calculateSCurveProfile(double distance, double initialVelocity) {

        // S-curve profile with jerk limiting
        // The jerk (rate of change of acceleration) is constant during ramp phases
        double jerk = maxAcceleration / rampTime;

        // Minimum distance needed with full S-curve (7 phases)

        // Distance during acceleration ramp-up
        double accelDistance = getAccelDistance(initialVelocity, jerk);

        // Distance for deceleration phase (symmetric)
        double decelDistance = maxVelocity * maxVelocity / (2 * maxAcceleration) + maxAcceleration * rampTime * rampTime / 3;

        // If we don't have enough distance for the full profile, use a
        // simpler calculation; for now we'll fall back to the trapezoidal
        // motion profile
        if (distance < accelDistance + decelDistance) {
            calculateTrapezoidalProfile(distance, initialVelocity);
            return;
        }

        // Full S-curve profile
        actualMaxVelocity = maxVelocity;

        // Phase 1: Jerk phase (acceleration ramp up)
        t1 = rampTime;

        // Phase 2: Constant acceleration
        double v1 = initialVelocity + jerk * rampTime * rampTime / 2;
        double velocityGainNeeded = maxVelocity - v1 - maxAcceleration * rampTime / 2;
        t2 = t1 + (velocityGainNeeded > 0 ? velocityGainNeeded / maxAcceleration : 0);

        // Phase 3: Jerk phase (acceleration ramp down)
        t3 = t2 + rampTime;

        // Calculate distances covered during acceleration
        double d1 = initialVelocity * rampTime + jerk * rampTime * rampTime * rampTime / 6;
        double d2 = v1 * (t2 - t1) + maxAcceleration * (t2 - t1) * (t2 - t1) / 2;
        double v2 = v1 + maxAcceleration * (t2 - t1);
        double d3 = v2 * rampTime + maxAcceleration * rampTime * rampTime / 2 - jerk * rampTime * rampTime * rampTime / 6;

        // Calculate deceleration distances
        double d5 = maxVelocity * rampTime - jerk * rampTime * rampTime * rampTime / 6;
        double v5 = maxVelocity - jerk * rampTime * rampTime / 2;
        double velocityLossNeeded = v5 - maxAcceleration * rampTime / 2;
        double decelConstantTime = velocityLossNeeded > 0 ? velocityLossNeeded / maxAcceleration : 0;
        double d6 = v5 * decelConstantTime - maxAcceleration * decelConstantTime * decelConstantTime / 2;
        double v6 = v5 - maxAcceleration * decelConstantTime;
        double d7 = v6 * rampTime - maxAcceleration * rampTime * rampTime / 2 + jerk * rampTime * rampTime * rampTime / 6;

        // Phase 4: Cruise
        double cruiseDistance = distance - (d1 + d2 + d3) - (d5 + d6 + d7);
        double cruiseTime = cruiseDistance / maxVelocity;
        t4 = t3 + cruiseTime;

        // Phase 5: Jerk phase (deceleration ramp up)
        t5 = t4 + rampTime;

        // Phase 6: Constant deceleration
        t6 = t5 + decelConstantTime;

        // Phase 7: Jerk phase (deceleration ramp down)
        t7 = t6 + rampTime;

        totalTime = t7;
    }

    /*
     * Calculates the distance required for the full acceleration phase
     */
    private double getAccelDistance(double initialVelocity, double jerk) {

        // Distance during acceleration ramp-up
        double rampUpDistance = (initialVelocity * rampTime + jerk * rampTime * rampTime * rampTime / 6);

        // Distance during the constant acceleration phase
        double constantAccelVelocityChange = maxVelocity - initialVelocity - jerk * rampTime * rampTime / 2;
        double constantAccelDistance = 0;
        double constantAccelTime = 0;
        if (constantAccelVelocityChange > 0) {
            constantAccelTime = constantAccelVelocityChange / maxAcceleration;
            constantAccelDistance = initialVelocity * constantAccelTime
                    + jerk * rampTime * rampTime / 2 * constantAccelTime
                    + maxAcceleration * constantAccelTime * constantAccelTime / 2;
        }

        // Distance during acceleration ramp-down
        double rampDownDistance = maxVelocity * rampTime - maxAcceleration * rampTime * rampTime / 2 + jerk * rampTime * rampTime * rampTime / 6;

        return rampUpDistance + constantAccelDistance + rampDownDistance;
    }

    // =============================================================
    // SAMPLING
    // =============================================================

    /**
     * Samples the motion profile at the specified time.
     *
     * @param time the time in seconds since motion started
     * @return the calculated position and velocity at that time
     * @throws IllegalStateException if no constraints have been set
     * @throws IllegalStateException if no parameters have been set
     */
    public State sample(double time) {

        if (Double.isNaN(maxVelocity)) {
            throw new IllegalStateException("Constraints have not been set");
        }
        if (Double.isNaN(startPosition)) {
            throw new IllegalStateException("Parameters have not been set");
        }

        if (time <= 0) {
            return new State(
                    startPosition,
                    startVelocity,
                    0.0
            );
        }

        if (time >= totalTime) {
            return new State(
                    finalPosition,
                    0.0,
                    0.0
            );
        }

        double position, velocity, acceleration;

        if (rampTime == 0) {
            State s = sampleTrapezoidal(time);
            position = s.position();
            velocity = s.velocity();
            acceleration = s.acceleration();
        } else {
            State s = sampleSCurve(time);
            position = s.position();
            velocity = s.velocity();
            acceleration = s.acceleration();
        }

        // Handle reversed direction
        if (isReversed) {
            position = startPosition - position;
            velocity = -velocity;
            acceleration = -acceleration;
        } else {
            position = startPosition + position;
        }

        return new State(position, velocity, acceleration);
    }

    /**
     * @return based on the most recently-set parameters, will the final
     * position be reached by the specified time?
     * @throws IllegalStateException if no constraints have been set
     * @throws IllegalStateException if no parameters have been set
     */

    private State sampleTrapezoidal(double time) {
        double position = 0;
        double velocity = isReversed ? -startVelocity : startVelocity;
        double acceleration = 0;

        if (velocity < 0) velocity = 0;

        if (time <= t2) {
            // Acceleration phase
            acceleration = maxAcceleration;
            velocity = velocity + acceleration * time;
            position = velocity * time - acceleration * time * time / 2;
        } else if (time <= t4) {
            // Cruise phase
            double accelTime = t2;
            double cruiseTime = time - t4;
            acceleration = 0;
            velocity = actualMaxVelocity;
            position = (velocity + (isReversed ? -startVelocity : startVelocity)) / 2 * accelTime
                    + velocity * (time - accelTime);
        } else {
            // Deceleration phase
            double accelTime = t2;
            double cruiseTime = t4 - t2;
            double decelTime = time - t4;
            acceleration = -maxAcceleration;
            velocity = actualMaxVelocity + acceleration * decelTime;
            double initialVel = isReversed ? -startVelocity : startVelocity;
            if (initialVel < 0) initialVel = 0;
            double accelDistance = (actualMaxVelocity + initialVel) / 2 * accelTime;
            double cruiseDistance = actualMaxVelocity * cruiseTime;
            position = accelDistance + cruiseDistance
                    + actualMaxVelocity * decelTime + acceleration * decelTime * decelTime / 2;
        }

        return new State(position, velocity, acceleration);
    }

    private State sampleSCurve(double time) {
        double initialVel = isReversed ? -startVelocity : startVelocity;
        if (initialVel < 0) initialVel = 0;

        double jerk = maxAcceleration / rampTime;
        double position = 0;
        double velocity = initialVel;
        double acceleration = 0;

        if (time <= t1) {
            // Phase 1: Acceleration ramp up
            acceleration = jerk * time;
            velocity = initialVel + jerk * time * time / 2;
            position = initialVel * time + jerk * time * time * time / 6;
        } else if (time <= t2) {
            // Phase 2: Constant acceleration
            double dt = time - t1;
            double v1 = initialVel + jerk * rampTime * rampTime / 2;
            double d1 = initialVel * rampTime + jerk * rampTime * rampTime * rampTime / 6;
            acceleration = maxAcceleration;
            velocity = v1 + acceleration * dt;
            position = d1 + v1 * dt + acceleration * dt * dt / 2;
        } else if (time <= t3) {
            // Phase 3: Acceleration ramp down
            double dt = time - t2;
            double v1 = initialVel + jerk * rampTime * rampTime / 2;
            double d1 = initialVel * rampTime + jerk * rampTime * rampTime * rampTime / 6;
            double v2 = v1 + maxAcceleration * (t2 - t1);
            double d2 = v1 * (t2 - t1) + maxAcceleration * (t2 - t1) * (t2 - t1) / 2;
            acceleration = maxAcceleration - jerk * dt;
            velocity = v2 + maxAcceleration * dt - jerk * dt * dt / 2;
            position = d1 + d2 + v2 * dt + maxAcceleration * dt * dt / 2 - jerk * dt * dt * dt / 6;
        } else if (time <= t4) {
            // Phase 4: Cruise
            double dt = time - t3;
            double v1 = initialVel + jerk * rampTime * rampTime / 2;
            double d1 = initialVel * rampTime + jerk * rampTime * rampTime * rampTime / 6;
            double v2 = v1 + maxAcceleration * (t2 - t1);
            double d2 = v1 * (t2 - t1) + maxAcceleration * (t2 - t1) * (t2 - t1) / 2;
            double d3 = v2 * rampTime + maxAcceleration * rampTime * rampTime / 2 - jerk * rampTime * rampTime * rampTime / 6;
            acceleration = 0;
            velocity = actualMaxVelocity;
            position = d1 + d2 + d3 + velocity * dt;
        } else if (time <= t5) {
            // Phase 5: Deceleration ramp up
            double dt = time - t4;
            double v1 = initialVel + jerk * rampTime * rampTime / 2;
            double d1 = initialVel * rampTime + jerk * rampTime * rampTime * rampTime / 6;
            double v2 = v1 + maxAcceleration * (t2 - t1);
            double d2 = v1 * (t2 - t1) + maxAcceleration * (t2 - t1) * (t2 - t1) / 2;
            double d3 = v2 * rampTime + maxAcceleration * rampTime * rampTime / 2 - jerk * rampTime * rampTime * rampTime / 6;
            double d4 = actualMaxVelocity * (t4 - t3);
            acceleration = -jerk * dt;
            velocity = actualMaxVelocity - jerk * dt * dt / 2;
            position = d1 + d2 + d3 + d4 + actualMaxVelocity * dt - jerk * dt * dt * dt / 6;
        } else if (time <= t6) {
            // Phase 6: Constant deceleration
            double dt = time - t5;
            double v1 = initialVel + jerk * rampTime * rampTime / 2;
            double d1 = initialVel * rampTime + jerk * rampTime * rampTime * rampTime / 6;
            double v2 = v1 + maxAcceleration * (t2 - t1);
            double d2 = v1 * (t2 - t1) + maxAcceleration * (t2 - t1) * (t2 - t1) / 2;
            double d3 = v2 * rampTime + maxAcceleration * rampTime * rampTime / 2 - jerk * rampTime * rampTime * rampTime / 6;
            double d4 = actualMaxVelocity * (t4 - t3);
            double v5 = actualMaxVelocity - jerk * rampTime * rampTime / 2;
            double d5 = actualMaxVelocity * rampTime - jerk * rampTime * rampTime * rampTime / 6;
            acceleration = -maxAcceleration;
            velocity = v5 + acceleration * dt;
            position = d1 + d2 + d3 + d4 + d5 + v5 * dt + acceleration * dt * dt / 2;
        } else {
            // Phase 7: Deceleration ramp down
            double dt = time - t6;
            double v1 = initialVel + jerk * rampTime * rampTime / 2;
            double d1 = initialVel * rampTime + jerk * rampTime * rampTime * rampTime / 6;
            double v2 = v1 + maxAcceleration * (t2 - t1);
            double d2 = v1 * (t2 - t1) + maxAcceleration * (t2 - t1) * (t2 - t1) / 2;
            double d3 = v2 * rampTime + maxAcceleration * rampTime * rampTime / 2 - jerk * rampTime * rampTime * rampTime / 6;
            double d4 = actualMaxVelocity * (t4 - t3);
            double v5 = actualMaxVelocity - jerk * rampTime * rampTime / 2;
            double d5 = actualMaxVelocity * rampTime - jerk * rampTime * rampTime * rampTime / 6;
            double v6 = v5 - maxAcceleration * (t6 - t5);
            double d6 = v5 * (t6 - t5) - maxAcceleration * (t6 - t5) * (t6 - t5) / 2;
            acceleration = -maxAcceleration + jerk * dt;
            velocity = v6 - maxAcceleration * dt + jerk * dt * dt / 2;
            position = d1 + d2 + d3 + d4 + d5 + d6 + v6 * dt - maxAcceleration * dt * dt / 2 + jerk * dt * dt * dt / 6;
        }

        return new State(position, velocity, acceleration);
    }

    // =============================================================
    // SAMPLING
    // =============================================================

    /**
     * @return will the motion described by the most recent parameters be
     * completed by the supplied time?
     */
    public boolean isFinishedAt(double time) {
        return Double.isNaN(totalTime) || time >= totalTime;
    }
}
