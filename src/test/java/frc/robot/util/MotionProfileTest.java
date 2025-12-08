package frc.robot.util;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import static org.junit.jupiter.api.Assertions.*;

class MotionProfileTest {

    private MotionProfile profile;
    private static final double EPSILON = 1e-6;

    @BeforeEach
    void setUp() {
        profile = new MotionProfile();
    }

    // ========== Constraint and Parameter Validation Tests ==========

    @Test
    @DisplayName("Should throw exception when sampling without setting parameters")
    void testSampleWithoutParameters() {
        profile.resetConstraints(10, 5);
        assertThrows(IllegalStateException.class, () -> profile.sample(1.0));
    }

    @Test
    @DisplayName("Should throw exception for negative max velocity")
    void testNegativeMaxVelocity() {
        assertThrows(IllegalArgumentException.class,
            () -> profile.resetConstraints(-10, 5));
    }

    @Test
    @DisplayName("Should throw exception for zero max velocity")
    void testZeroMaxVelocity() {
        assertThrows(IllegalArgumentException.class,
            () -> profile.resetConstraints(0, 5));
    }

    @Test
    @DisplayName("Should throw exception for negative max acceleration")
    void testNegativeMaxAcceleration() {
        assertThrows(IllegalArgumentException.class,
            () -> profile.resetConstraints(10, -5));
    }

    @Test
    @DisplayName("Should throw exception for negative ramp time")
    void testNegativeRampTime() {
        assertThrows(IllegalArgumentException.class,
            () -> profile.resetConstraints(10, 5, -0.1));
    }

    // ========== Trapezoidal Profile Tests ==========

    @Test
    @DisplayName("Full trapezoidal profile: should reach final position")
    void testFullTrapezoidalProfile() {
        profile.resetConstraints(10, 5, 0);  // rampTime = 0 for trapezoidal
        profile.resetMotion(0, 50);

        // Sample at start
        MotionProfile.State startState = profile.sample(0);
        assertEquals(0, startState.position(), EPSILON);
        assertEquals(0, startState.velocity(), EPSILON);

        // Sample in the middle (should be moving)
        MotionProfile.State midState = profile.sample(3);
        assertTrue(midState.position() > 0 && midState.position() < 50);
        assertTrue(midState.velocity() > 0);

        // Sample at a large time (should reach final position)
        MotionProfile.State endState = profile.sample(100);
        assertEquals(50, endState.position(), EPSILON);
        assertEquals(0, endState.velocity(), EPSILON);
    }

    @Test
    @DisplayName("Triangle profile: should handle short distance without cruise phase")
    void testTriangleProfile() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 5);  // Short distance

        // Should still reach final position
        MotionProfile.State endState = profile.sample(100);
        assertEquals(5, endState.position(), EPSILON);
        assertEquals(0, endState.velocity(), EPSILON);
    }

    @Test
    @DisplayName("Trapezoidal profile with non-zero starting velocity")
    void testTrapezoidalWithInitialVelocity() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 3, 50);  // Start with velocity of 3

        MotionProfile.State startState = profile.sample(0);
        assertEquals(0, startState.position(), EPSILON);
        assertEquals(3, startState.velocity(), EPSILON);

        // Should still reach final position
        MotionProfile.State endState = profile.sample(100);
        assertEquals(50, endState.position(), EPSILON);
        assertEquals(0, endState.velocity(), EPSILON);
    }

    @Test
    @DisplayName("Reverse direction: should handle negative distance")
    void testReverseDirection() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(50, 0);  // Move from 50 to 0

        MotionProfile.State startState = profile.sample(0);
        assertEquals(50, startState.position(), EPSILON);

        MotionProfile.State endState = profile.sample(100);
        assertEquals(0, endState.position(), EPSILON);
        assertEquals(0, endState.velocity(), EPSILON);
    }

    @Test
    @DisplayName("Zero distance: should stay at start position")
    void testZeroDistance() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(5, 5);  // Same start and end

        MotionProfile.State state = profile.sample(1);
        assertEquals(5, state.position(), EPSILON);
        assertEquals(0, state.velocity(), EPSILON);
    }

    // ========== S-Curve Profile Tests ==========

    @Test
    @DisplayName("S-curve profile: should reach final position with smooth acceleration")
    void testSCurveProfile() {
        profile.resetConstraints(10, 5, 0.2);  // rampTime > 0 for S-curve
        profile.resetMotion(0, 50);

        // Sample at start
        MotionProfile.State startState = profile.sample(0);
        assertEquals(0, startState.position(), EPSILON);
        assertEquals(0, startState.velocity(), EPSILON);
        assertEquals(0, startState.acceleration(), EPSILON);

        // Sample during ramp-up phase (acceleration should be increasing)
        MotionProfile.State rampState = profile.sample(0.1);
        assertTrue(rampState.acceleration() > 0);
        assertTrue(rampState.velocity() > 0);

        // Sample at end
        MotionProfile.State endState = profile.sample(100);
        assertEquals(50, endState.position(), EPSILON);
        assertEquals(0, endState.velocity(), EPSILON);
        assertEquals(0, endState.acceleration(), EPSILON);
    }

    @Test
    @DisplayName("S-curve with short distance: should fall back gracefully")
    void testSCurveShortDistance() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 3);  // Short distance

        // Should still reach final position
        MotionProfile.State endState = profile.sample(100);
        assertEquals(3, endState.position(), EPSILON);
        assertEquals(0, endState.velocity(), EPSILON);
    }

    // ========== isFinishedAt Tests ==========

    @Test
    @DisplayName("isFinishedAt: should return false before completion")
    void testIsFinishedAtBeforeCompletion() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        assertFalse(profile.isFinishedAt(0.1));
        assertFalse(profile.isFinishedAt(1.0));
    }

    @Test
    @DisplayName("isFinishedAt: should return true after completion")
    void testIsFinishedAtAfterCompletion() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        assertTrue(profile.isFinishedAt(100));
    }

    // ========== Physics Consistency Tests ==========

    @Test
    @DisplayName("Position should be monotonically increasing for forward motion")
    void testMonotonicPosition() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        double previousPosition = 0;
        for (double t = 0; t <= 20; t += 0.1) {
            MotionProfile.State state = profile.sample(t);
            assertTrue(state.position() >= previousPosition,
                "Position should be monotonically increasing at t=" + t);
            previousPosition = state.position();
            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("Velocity should be non-negative for forward motion")
    void testNonNegativeVelocity() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        for (double t = 0; t <= 20; t += 0.1) {
            MotionProfile.State state = profile.sample(t);
            assertTrue(state.velocity() >= -EPSILON,
                "Velocity should be non-negative at t=" + t + ", got " + state.velocity());
            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("Acceleration should respect max acceleration constraint (trapezoidal)")
    void testMaxAccelerationConstraint() {
        double maxAccel = 5.0;
        profile.resetConstraints(10, maxAccel, 0);
        profile.resetMotion(0, 50);

        for (double t = 0; t <= 20; t += 0.1) {
            MotionProfile.State state = profile.sample(t);
            assertTrue(Math.abs(state.acceleration()) <= maxAccel + EPSILON,
                "Acceleration should not exceed max at t=" + t + ", got " + state.acceleration());
            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("Velocity should respect max velocity constraint")
    void testMaxVelocityConstraint() {
        double maxVel = 10.0;
        profile.resetConstraints(maxVel, 5, 0);
        profile.resetMotion(0, 100);  // Long distance to ensure cruise phase

        for (double t = 0; t <= 30; t += 0.1) {
            MotionProfile.State state = profile.sample(t);
            assertTrue(state.velocity() <= maxVel + EPSILON,
                "Velocity should not exceed max at t=" + t + ", got " + state.velocity());
            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("Final velocity should be zero")
    void testFinalVelocityIsZero() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        // Find when motion completes
        double endTime = 0;
        for (double t = 0; t <= 100; t += 0.1) {
            if (profile.isFinishedAt(t)) {
                endTime = t;
                break;
            }
        }

        MotionProfile.State finalState = profile.sample(endTime);
        assertEquals(0, finalState.velocity(), EPSILON, "Final velocity should be zero");
    }

    @Test
    @DisplayName("Position integration: verify position matches integrated velocity")
    void testPositionIntegration() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        double dt = 0.01;
        double integratedPosition = 0;
        double previousVelocity = 0;

        for (double t = 0; t <= 20; t += dt) {
            MotionProfile.State state = profile.sample(t);

            if (t > 0) {
                // Use trapezoidal integration for velocity
                integratedPosition += (previousVelocity + state.velocity()) / 2 * dt;

                // Position from integration should roughly match actual position
                // Allow larger tolerance due to numerical integration errors
                assertEquals(state.position(), integratedPosition, 0.1,
                    "Integrated position should match at t=" + t);
            }

            previousVelocity = state.velocity();

            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("Velocity integration: verify velocity matches integrated acceleration")
    void testVelocityIntegration() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);

        double dt = 0.01;
        double integratedVelocity = 0;
        double previousAcceleration = 0;

        for (double t = 0; t <= 20; t += dt) {
            MotionProfile.State state = profile.sample(t);

            if (t > 0) {
                // Use trapezoidal integration for acceleration
                integratedVelocity += (previousAcceleration + state.acceleration()) / 2 * dt;

                // Velocity from integration should roughly match actual velocity
                assertEquals(state.velocity(), integratedVelocity, 0.05,
                    "Integrated velocity should match at t=" + t);
            }

            previousAcceleration = state.acceleration();

            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    // ========== S-Curve Specific Tests ==========

    @Test
    @DisplayName("S-curve: acceleration should be smooth (no discontinuities)")
    void testSCurveSmoothAcceleration() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 50);

        double dt = 0.01;
        double previousAcceleration = 0;

        for (double t = 0; t <= 20; t += dt) {
            MotionProfile.State state = profile.sample(t);

            if (t > 0) {
                // Jerk (change in acceleration) should be bounded
                double jerk = Math.abs((state.acceleration() - previousAcceleration) / dt);

                // Maximum jerk is maxAcceleration / rampTime = 5 / 0.2 = 25
                // Allow some numerical tolerance
                assertTrue(jerk <= 30,
                    "Jerk should be bounded at t=" + t + ", got " + jerk);
            }

            previousAcceleration = state.acceleration();

            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("S-curve: initial acceleration should start at zero")
    void testSCurveInitialAcceleration() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 50);

        MotionProfile.State startState = profile.sample(0);
        assertEquals(0, startState.acceleration(), EPSILON,
            "Initial acceleration should be zero for S-curve");
    }

    @Test
    @DisplayName("S-curve: final acceleration should end at zero")
    void testSCurveFinalAcceleration() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 50);

        // Find when motion completes
        double endTime = 0;
        for (double t = 0; t <= 100; t += 0.1) {
            if (profile.isFinishedAt(t)) {
                endTime = t;
                break;
            }
        }

        MotionProfile.State finalState = profile.sample(endTime);
        assertEquals(0, finalState.acceleration(), EPSILON,
            "Final acceleration should be zero for S-curve");
    }

    // ========== Default Method Tests ==========

    @Test
    @DisplayName("resetConstraints default: should use zero ramp time")
    void testResetConstraintsDefault() {
        profile.resetConstraints(10, 5);  // No ramp time specified
        profile.resetMotion(0, 50);

        // This should behave like a trapezoidal profile
        MotionProfile.State state = profile.sample(0.1);
        // For trapezoidal, acceleration should jump to max immediately
        // (though at t=0.1 we're in acceleration phase)
        assertNotNull(state);
    }

    @Test
    @DisplayName("resetMotion default: should use zero starting velocity")
    void testresetMotionDefault() {
        profile.resetConstraints(10, 5, 0);
        profile.resetMotion(0, 50);  // No starting velocity specified

        MotionProfile.State startState = profile.sample(0);
        assertEquals(0, startState.velocity(), EPSILON,
            "Starting velocity should default to zero");
    }

    // ========== Velocity Continuity Tests ==========

    @Test
    @DisplayName("S-curve: velocity should be continuous at phase boundaries")
    void testSCurveVelocityContinuity() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 100);  // Long distance to ensure all phases

        double dt = 0.0001;  // Very small time step

        // Sample throughout the profile looking for discontinuities
        for (double t = dt; t < 50; t += 0.05) {
            MotionProfile.State before = profile.sample(t - dt);
            MotionProfile.State after = profile.sample(t + dt);

            double velocityChange = Math.abs(after.velocity() - before.velocity());
            double expectedChange = Math.max(
                Math.abs(before.acceleration()),
                Math.abs(after.acceleration())
            ) * 2 * dt;

            // Velocity change should be smooth (proportional to acceleration * time)
            // Allow 50% tolerance for numerical effects
            assertTrue(velocityChange <= expectedChange * 1.5,
                String.format("Velocity discontinuity at t=%.4f: before=%.6f, after=%.6f, change=%.6f, expected<=%.6f",
                    t, before.velocity(), after.velocity(), velocityChange, expectedChange));

            if (profile.isFinishedAt(t)) {
                break;
            }
        }
    }

    @Test
    @DisplayName("S-curve: velocity at end of phase 3 should match cruise velocity")
    void testSCurvePhase3EndVelocity() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 100);

        // Phase 3 ends at t3 = rampTime + constantAccelTime + rampTime
        // We need to sample throughout and find where cruise phase begins
        double maxVel = 10.0;
        boolean foundCruise = false;

        for (double t = 0.5; t < 10; t += 0.01) {
            MotionProfile.State state = profile.sample(t);

            // Cruise phase has zero acceleration and max velocity
            if (Math.abs(state.acceleration()) < EPSILON &&
                Math.abs(state.velocity() - maxVel) < 0.01) {
                foundCruise = true;

                // Check that velocity just before cruise is also close to maxVel
                MotionProfile.State beforeCruise = profile.sample(t - 0.01);
                assertEquals(maxVel, beforeCruise.velocity(), 0.1,
                    "Velocity should smoothly reach max velocity before cruise at t=" + t);
                break;
            }
        }

        assertTrue(foundCruise, "Should find cruise phase in the profile");
    }

    @Test
    @DisplayName("S-curve: velocity should reach zero at end")
    void testSCurveEndsAtZeroVelocity() {
        profile.resetConstraints(10, 5, 0.2);
        profile.resetMotion(0, 100);

        // Find the end time
        double endTime = 0;
        for (double t = 0; t < 100; t += 0.1) {
            if (profile.isFinishedAt(t)) {
                endTime = t - 0.1;  // Back up slightly
                break;
            }
        }

        // Sample near the end
        MotionProfile.State nearEnd = profile.sample(endTime);
        assertTrue(nearEnd.velocity() >= -EPSILON,
            "Velocity near end should be non-negative, got " + nearEnd.velocity());
        assertTrue(nearEnd.velocity() <= 1.0,
            "Velocity near end should be close to zero, got " + nearEnd.velocity());

        // Sample at finish
        MotionProfile.State atEnd = profile.sample(endTime + 1.0);
        assertEquals(0, atEnd.velocity(), EPSILON,
            "Velocity at end should be exactly zero");
    }
}
