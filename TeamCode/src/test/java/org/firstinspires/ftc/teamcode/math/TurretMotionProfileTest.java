package org.firstinspires.ftc.teamcode.math;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class TurretMotionProfileTest {
    private static final double EPS = 1e-8;

    @Test public void longMoveAcceleratesCruisesAndBrakesWithoutPassingGoal() {
        TurretMotionProfile profile = new TurretMotionProfile(0, 0, 100, 20, 10);
        assertEquals(7, profile.duration(), EPS);
        state(profile.sample(0), 0, 0, 10);
        state(profile.sample(1), 5, 10, 10);
        state(profile.sample(3), 40, 20, 0);
        state(profile.sample(6), 95, 10, -10);
        state(profile.sample(7), 100, 0, 0);
        assertEquals(0, profile.minPosition(), 0);
        assertEquals(100, profile.maxPosition(), 0);
        checkTrajectory(profile, 0, 20, 10);
    }

    @Test public void shortMoveIsTriangularInBothDirections() {
        for (double sign : new double[]{1, -1}) {
            TurretMotionProfile profile = new TurretMotionProfile(25, 0, 25 + sign * 10, 100, 40);
            assertEquals(1, profile.duration(), EPS);
            state(profile.sample(0.25), 25 + sign * 1.25, sign * 10, sign * 40);
            state(profile.sample(0.5), 25 + sign * 5, sign * 20, -sign * 40);
            state(profile.sample(0.75), 25 + sign * 8.75, sign * 10, -sign * 40);
            state(profile.sample(1), 25 + sign * 10, 0, 0);
            checkTrajectory(profile, 0, 100, 40);
        }
    }

    @Test public void motionAwayFromGoalBrakesBeforeReversing() {
        TurretMotionProfile profile = new TurretMotionProfile(0, -10, 20, 20, 10);
        state(profile.sample(0), 0, -10, 10);
        state(profile.sample(0.5), -3.75, -5, 10);
        state(profile.sample(1), -5, 0, 10);
        assertEquals(-5, profile.minPosition(), EPS);
        assertEquals(20, profile.maxPosition(), EPS);
        assertEquals(1 + 2 * Math.sqrt(2.5), profile.duration(), EPS);
        state(profile.sample(profile.duration()), 20, 0, 0);
        checkTrajectory(profile, -10, 20, 10);
    }

    @Test public void goalInsideStoppingDistanceIncludesUnavoidableOvershoot() {
        for (double sign : new double[]{1, -1}) {
            TurretMotionProfile profile = new TurretMotionProfile(0, sign * 20, sign * 5, 30, 10);
            state(profile.sample(1), sign * 15, sign * 10, -sign * 10);
            state(profile.sample(2), sign * 20, 0, -sign * 10);
            assertEquals(sign > 0 ? 0 : -20, profile.minPosition(), EPS);
            assertEquals(sign > 0 ? 20 : 0, profile.maxPosition(), EPS);
            state(profile.sample(profile.duration()), sign * 5, 0, 0);
            checkTrajectory(profile, sign * 20, 30, 10);
        }
    }

    @Test public void initialOverspeedDeceleratesContinuouslyInsteadOfClampingVelocity() {
        TurretMotionProfile profile = new TurretMotionProfile(0, 40, 200, 20, 10);
        state(profile.sample(0), 0, 40, -10);
        state(profile.sample(1), 35, 30, -10);
        state(profile.sample(2), 60, 20, 0);
        state(profile.sample(8), 180, 20, -10);
        state(profile.sample(10), 200, 0, 0);
        assertEquals(10, profile.duration(), EPS);
        assertEquals(200, profile.maxPosition(), 0);
        checkTrajectory(profile, 40, 20, 10);
    }

    @Test public void samePositionWithVelocityStillPlansAStopAndReturn() {
        TurretMotionProfile profile = new TurretMotionProfile(5, 10, 5, 20, 10);
        assertEquals(1 + Math.sqrt(2), profile.duration(), EPS);
        state(profile.sample(1), 10, 0, -10);
        state(profile.sample(profile.duration()), 5, 0, 0);
        assertEquals(5, profile.minPosition(), 0);
        assertEquals(10, profile.maxPosition(), EPS);
        checkTrajectory(profile, 10, 20, 10);
    }

    @Test public void goalExactlyAtStoppingDistanceNeedsOnlyBraking() {
        TurretMotionProfile profile = new TurretMotionProfile(0, 20, 20, 30, 10);
        assertEquals(2, profile.duration(), EPS);
        state(profile.sample(0), 0, 20, -10);
        state(profile.sample(1), 15, 10, -10);
        state(profile.sample(2), 20, 0, 0);
        assertEquals(20, profile.maxPosition(), 0);
    }

    @Test public void replanningDuringEachPhasePreservesPositionAndVelocity() {
        TurretMotionProfile original = new TurretMotionProfile(0, 0, 100, 20, 10);
        for (double time : new double[]{0.5, 3, 6.5}) {
            TurretMotionProfile.State current = original.sample(time);
            for (double newGoal : new double[]{-30, current.position + 0.5, 120}) {
                TurretMotionProfile replacement = new TurretMotionProfile(
                        current.position, current.velocity, newGoal, 20, 10);
                assertEquals(current.position, replacement.sample(0).position, 0);
                assertEquals(current.velocity, replacement.sample(0).velocity, 0);
                state(replacement.sample(replacement.duration()), newGoal, 0, 0);
                checkTrajectory(replacement, current.velocity, 20, 10);
            }
        }
    }

    @Test public void completedAndZeroLengthProfilesStayAtExactGoal() {
        TurretMotionProfile stationary = new TurretMotionProfile(-32.7, 0, -32.7, 25, 80);
        assertEquals(0, stationary.duration(), 0);
        state(stationary.sample(0), -32.7, 0, 0);
        state(stationary.sample(100), -32.7, 0, 0);
        TurretMotionProfile moving = new TurretMotionProfile(12.3, -5.6, 45.7, 25, 80);
        state(moving.sample(moving.duration()), 45.7, 0, 0);
        state(moving.sample(1e100), 45.7, 0, 0);
    }

    @Test public void rejectsInvalidInputsAndArithmeticOverflow() {
        for (double bad : new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(bad, 0, 10, 20, 10));
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(0, bad, 10, 20, 10));
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(0, 0, bad, 20, 10));
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(0, 0, 10, bad, 10));
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(0, 0, 10, 20, bad));
        }
        for (double bad : new double[]{0, -1}) {
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(0, 0, 10, bad, 10));
            assertThrows(IllegalArgumentException.class, () -> new TurretMotionProfile(0, 0, 10, 20, bad));
        }
        assertThrows(IllegalArgumentException.class,
                () -> new TurretMotionProfile(-Double.MAX_VALUE, 0, Double.MAX_VALUE, 20, 10));
        assertThrows(IllegalArgumentException.class,
                () -> new TurretMotionProfile(0, Double.MAX_VALUE, 10, 20, 10));
        TurretMotionProfile profile = new TurretMotionProfile(0, 0, 10, 20, 10);
        for (double bad : new double[]{-0.1, Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> profile.sample(bad));
        }
    }

    @Test public void variedInitialStatesRespectVelocityAccelerationAndReportedExtents() {
        Random random = new Random(8103);
        for (int i = 0; i < 250; i++) {
            double start = random.nextDouble() * 200 - 100;
            double velocity = random.nextDouble() * 120 - 60;
            double goal = random.nextDouble() * 300 - 150;
            double maxVelocity = 10 + random.nextDouble() * 40;
            double maxAcceleration = 5 + random.nextDouble() * 115;
            TurretMotionProfile profile = new TurretMotionProfile(start, velocity, goal,
                    maxVelocity, maxAcceleration);
            checkTrajectory(profile, velocity, maxVelocity, maxAcceleration);
            state(profile.sample(profile.duration()), goal, 0, 0);
        }
    }

    private static void checkTrajectory(TurretMotionProfile profile, double startVelocity,
                                        double maximumVelocity, double maximumAcceleration) {
        assertTrue(Double.isFinite(profile.duration()));
        assertTrue(profile.duration() >= 0);
        if (profile.duration() == 0) return;
        int steps = 500;
        double dt = profile.duration() / steps;
        TurretMotionProfile.State previous = profile.sample(0);
        for (int i = 1; i <= steps; i++) {
            TurretMotionProfile.State current = profile.sample(i * profile.duration() / steps);
            assertTrue(Double.isFinite(current.position));
            assertTrue(Double.isFinite(current.velocity));
            assertTrue(Double.isFinite(current.acceleration));
            assertTrue(current.position >= profile.minPosition() - EPS);
            assertTrue(current.position <= profile.maxPosition() + EPS);
            assertTrue(Math.abs(current.velocity) <= Math.max(Math.abs(startVelocity), maximumVelocity) + EPS);
            assertTrue(Math.abs(current.acceleration) <= maximumAcceleration + EPS);
            assertTrue(Math.abs(current.velocity - previous.velocity) <= maximumAcceleration * dt + EPS);
            // With bounded acceleration, a first-order position prediction differs by at most a*dt^2/2.
            assertEquals(previous.position + previous.velocity * dt, current.position,
                    maximumAcceleration * dt * dt / 2 + EPS);
            previous = current;
        }
    }

    private static void state(TurretMotionProfile.State actual, double position, double velocity, double acceleration) {
        assertEquals(position, actual.position, EPS);
        assertEquals(velocity, actual.velocity, EPS);
        assertEquals(acceleration, actual.acceleration, EPS);
    }
}
