package org.firstinspires.ftc.teamcode.math;

import org.junit.Test;

import static org.junit.Assert.*;

public class ContinuousAngleTrackerTest {
    @Test public void firstSampleSeedsShaftAngleWithoutInventingRevolutions() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(720, 0.1);
        assertFalse(tracker.hasSample());
        assertTrue(Double.isNaN(tracker.getDegrees()));
        assertTrue(tracker.update(275, 0));
        assertTrue(tracker.hasSample());
        assertEquals(275, tracker.getDegrees(), 0);
        assertEquals(0, tracker.getVelocityDegreesPerSecond(), 0);
    }

    @Test public void crossesEncoderSeamInBothDirections() {
        ContinuousAngleTracker forward = new ContinuousAngleTracker(720, 0.1);
        assertTrue(forward.update(359, 0));
        assertTrue(forward.update(1, 10_000_000));
        assertEquals(361, forward.getDegrees(), 1e-9);
        assertEquals(200, forward.getVelocityDegreesPerSecond(), 1e-9);

        ContinuousAngleTracker reverse = new ContinuousAngleTracker(720, 0.1);
        assertTrue(reverse.update(1, 0));
        assertTrue(reverse.update(359, 10_000_000));
        assertEquals(-1, reverse.getDegrees(), 1e-9);
        assertEquals(-200, reverse.getVelocityDegreesPerSecond(), 1e-9);
    }

    @Test public void countsManyTurnsWithoutApplyingTurretGearing() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(1000, 0.15);
        assertTrue(tracker.update(350, 0));
        double home = tracker.getDegrees();
        for (int step = 1; step <= 12; step++) {
            assertTrue(tracker.update((350 + step * 90) % 360, step * 100_000_000L));
        }
        assertEquals(1430, tracker.getDegrees(), 1e-9);
        // Three servo turns move a 4:1 geared turret 270 degrees; the caller applies this ratio.
        assertEquals(270, (tracker.getDegrees() - home) / 4, 1e-9);
    }

    @Test public void stationaryAndRepeatedIdenticalSampleRemainValid() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(720, 0.1);
        assertTrue(tracker.update(25, 0));
        assertTrue(tracker.update(25, 0));
        assertTrue(tracker.update(25, 50_000_000));
        assertEquals(25, tracker.getDegrees(), 0);
        assertEquals(0, tracker.getVelocityDegreesPerSecond(), 0);
        assertFalse(tracker.update(26, 50_000_000));
        assertLost(tracker);
    }

    @Test public void backwardTimeAndLongGapsLoseRevolutionReference() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(720, 0.1);
        assertTrue(tracker.update(40, 100_000_000));
        assertFalse(tracker.update(40, 99_000_000));
        assertLost(tracker);
        assertTrue(tracker.update(50, 200_000_000));
        assertEquals(50, tracker.getDegrees(), 0);
        assertFalse(tracker.update(50, 301_000_000));
        assertLost(tracker);
    }

    @Test public void rejectsSamplingIntervalThatCouldHideHalfTurnEvenWithoutApparentMovement() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(3600, 1);
        assertTrue(tracker.update(20, 0));
        assertFalse(tracker.update(20, 50_000_000));
        assertLost(tracker);
        assertTrue(tracker.getStatus().contains("half-turn"));
    }

    @Test public void exactHalfTurnIsAmbiguousInEitherDirection() {
        for (double first : new double[]{0, 180}) {
            ContinuousAngleTracker tracker = new ContinuousAngleTracker(1790, 0.1);
            assertTrue(tracker.update(first, 0));
            assertFalse(tracker.update(180 - first, 100_000_000));
            assertLost(tracker);
            assertTrue(tracker.getStatus().contains("ambiguous"));
        }
    }

    @Test public void noiseCannotReverseDirectionNearHalfTurnSamplingBoundary() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(1790, 0.1);
        assertTrue(tracker.update(0, 0));
        // 179 degrees physical travel plus 2 degrees noise must not be read as -179.
        assertFalse(tracker.update(181, 100_000_000));
        assertLost(tracker);
    }

    @Test public void rejectsImpossibleJumpButToleratesSmallEncoderNoise() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(100, 0.1);
        assertTrue(tracker.update(10, 0));
        assertTrue(tracker.update(12, 1_000_000));
        assertFalse(tracker.update(20, 2_000_000));
        assertLost(tracker);
        assertTrue(tracker.getStatus().contains("maximum speed"));
    }

    @Test public void invalidAnglesAndConfigurationCannotSeedTracker() {
        for (double invalid : new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, -1, 360}) {
            ContinuousAngleTracker tracker = new ContinuousAngleTracker(720, 0.1);
            assertFalse(tracker.update(invalid, 0));
            assertLost(tracker);
        }
        for (double invalid : new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, -1, 0}) {
            ContinuousAngleTracker invalidSpeed = new ContinuousAngleTracker(invalid, 0.1);
            assertFalse(invalidSpeed.update(0, 0));
            assertLost(invalidSpeed);
            ContinuousAngleTracker invalidGap = new ContinuousAngleTracker(720, invalid);
            assertFalse(invalidGap.update(0, 0));
            assertLost(invalidGap);
        }
    }

    @Test public void invalidReadingAndExplicitResetBothDiscardPriorTurns() {
        ContinuousAngleTracker tracker = new ContinuousAngleTracker(720, 0.1);
        assertTrue(tracker.update(359, 0));
        assertTrue(tracker.update(1, 10_000_000));
        assertFalse(tracker.update(Double.NaN, 20_000_000));
        assertLost(tracker);
        assertTrue(tracker.update(5, 30_000_000));
        assertEquals(5, tracker.getDegrees(), 0);
        tracker.reset();
        assertLost(tracker);
        assertTrue(tracker.update(300, 40_000_000));
        assertEquals(300, tracker.getDegrees(), 0);
    }

    private static void assertLost(ContinuousAngleTracker tracker) {
        assertFalse(tracker.hasSample());
        assertTrue(Double.isNaN(tracker.getDegrees()));
        assertTrue(Double.isNaN(tracker.getVelocityDegreesPerSecond()));
        assertNotNull(tracker.getStatus());
    }
}
