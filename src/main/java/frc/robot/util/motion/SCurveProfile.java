package frc.robot.util.motion;

import edu.wpi.first.math.util.Units;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * <p>Provides a one-dimensional "motion profile" - calculates position and
 * velocity over time respecting constraints on velocity, acceleration and
 * ramp time. Units don't matter as long as they are consistent between
 * position and velocity.</p>
 *
 * <p>In the degenerate case where the ramp time is 0, this is essentially the
 * same as the {@link edu.wpi.first.math.trajectory.TrapezoidProfile}. When
 * a ramp time is specified, motion follows an "S-Curve" trajectory with up
 * to 7 phases of motion.</p>
 *
 * <p>This is meant to be a more or less drop-in replacement for a
 * {@link edu.wpi.first.math.trajectory.TrapezoidProfile}</p>
 */
public class SCurveProfile {

    /**
     * Represents constraints on motion. The motion profile doesn't care what
     * units are used for velocity / acceleration, but they have to agree with
     * one another.
     *
     * @param maxVelocity supplier for max velocity in units per second
     * @param maxAcceleration supplier for max acceleration in units per second squared
     * @param rampTime supplier for ramp time in seconds
     */
    public record Constraints(DoubleSupplier maxVelocity,
                              DoubleSupplier maxAcceleration,
                              DoubleSupplier rampTime) {

        /**
         * Converts supplied constraints to meters
         * @param maxVelocity supplier for max velocity in feet per second
         * @param maxAcceleration supplier for max acceleration in feet per second squared
         * @param rampTime supplier for ramp time in seconds
         * @return new constraints expressed in meters and seconds
         */
        public static Constraints feetToMeters(
                DoubleSupplier maxVelocity,
                DoubleSupplier maxAcceleration,
                DoubleSupplier rampTime) {
            return new Constraints(
                    () -> Units.feetToMeters(maxVelocity.getAsDouble()),
                    () -> Units.feetToMeters(maxAcceleration.getAsDouble()),
                    rampTime);
        }

        /**
         * Converts supplied constraints to radians
         * @param maxVelocity supplier for max velocity in degrees per second
         * @param maxAcceleration supplier for max acceleration in degrees per second squared
         * @param rampTime supplier for ramp time in seconds
         * @return new constraints expressed in radians and seconds
         */
        public static Constraints degreesToRadians(
                DoubleSupplier maxVelocity,
                DoubleSupplier maxAcceleration,
                DoubleSupplier rampTime) {
            return new Constraints(
                    () -> Units.degreesToRadians(maxVelocity.getAsDouble()),
                    () -> Units.degreesToRadians(maxAcceleration.getAsDouble()),
                    rampTime);
        }
    }

    /**
     * Captures the expected position, velocity and acceleration at a moment
     * in time.
     *
     * @param position position in units
     * @param velocity velocity in units per second
     * @param acceleration acceleration in units per second squared
     */
    public record State(double position, double velocity, double acceleration) {

    }

    // Constraints on motion (either stored as suppliers or static values)
    private final DoubleSupplier maxVelocitySupplier;
    private final DoubleSupplier maxAccelerationSupplier;
    private final DoubleSupplier rampTimeSupplier;

    // Current constraint values (read from suppliers in reset())
    double maxVelocity;
    double maxAcceleration;
    double rampTime;

    // Motion parameters
    double startPosition;
    double startVelocity;
    double finalPosition;

    // Calculated profile timing
    double totalTime = Double.NaN;
    double actualMaxVelocity = Double.NaN;
    boolean isReversed = false;

    // Phase timing (for S-curve profile with up to 7 phases)
    private double t1;  // End of initial jerk phase (acceleration ramp up)
    private double t2;  // End of constant acceleration phase
    private double t3;  // End of jerk phase (acceleration ramp down)
    private double t4;  // End of cruise phase
    private double t5;  // End of jerk phase (deceleration ramp up)
    private double t6;  // End of constant deceleration phase
    private double t7;  // End of jerk phase (deceleration ramp down) - total time

    /**
     * Creates a new {@link SCurveProfile}
     * @param constraints motion constraints (required)
     * @throws NullPointerException if required parameters are null
     */
    public SCurveProfile(Constraints constraints) {
        this(constraints.maxVelocity, constraints.maxAcceleration, constraints.rampTime);
    }

    /**
     * Creates a new {@link SCurveProfile}
     * @param maxVelocity supplier for max velocity in units per second (required)
     * @param maxAcceleration supplier for max acceleration in units per second squared (required)
     * @param rampTime supplier for ramp time in seconds (required)
     * @throws NullPointerException if required parameters are null
     */
    public SCurveProfile(DoubleSupplier maxVelocity, DoubleSupplier maxAcceleration, DoubleSupplier rampTime) {
        Objects.requireNonNull(maxVelocity);
        Objects.requireNonNull(maxAcceleration);
        Objects.requireNonNull(rampTime);
        this.maxVelocitySupplier = maxVelocity;
        this.maxAccelerationSupplier = maxAcceleration;
        this.rampTimeSupplier = rampTime;
    }

    // =============================================================
    // RESET & CALCULATION
    // =============================================================

    /**
     * Called when initiating a new motion. This will calculate the motion
     * profile, phase points and overall timing. We assume that, although we
     * might be moving when starting, the goal is to end up stationary at the
     * final position.
     *
     * @param startPosition the start position of the motion
     * @param startVelocity the start velocity of the motion
     * @param finalPosition the final position of the motion
     */
    public void reset(double startPosition, double startVelocity, double finalPosition) {

        // Re-read constraint values from suppliers
        this.maxVelocity = maxVelocitySupplier.getAsDouble();
        this.maxAcceleration = maxAccelerationSupplier.getAsDouble();
        this.rampTime = rampTimeSupplier.getAsDouble();

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
     * @return the calculated position, velocity and acceleration at that time
     * @throws IllegalStateException if called before {@link #reset(double, double, double)}
     */
    public State sample(double time) {

        if (Double.isNaN(totalTime)) {
            throw new IllegalStateException("you need to call reset() first");
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

    /*
     * Calculates a sample in the simple case where we're following a
     * trapezoid profile
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

    /*
     * Calculates a sample in the case where we're following an S-Curve
     * profile
     */
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
    // TIMING
    // =============================================================

    /**
     * @return how long in seconds this profile should run for
     */
    public double totalTime() {
        return totalTime;
    }
}
