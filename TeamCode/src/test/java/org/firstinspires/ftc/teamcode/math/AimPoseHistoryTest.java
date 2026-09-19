package org.firstinspires.ftc.teamcode.math;

import org.junit.Test;

import static org.junit.Assert.*;

public class AimPoseHistoryTest {
    private static final long MS = 1_000_000L;

    @Test public void interpolatesRobotAndUnwrappedTurretAtCameraCaptureTime() {
        AimPoseHistory history = new AimPoseHistory();
        assertTrue(history.add(100 * MS, 10, 20, 0, 350));
        assertTrue(history.add(200 * MS, 12, 24, Math.PI / 2, 370));
        AimPoseHistory.Sample capture = history.interpolate(150 * MS);
        assertNotNull(capture);
        assertEquals(150 * MS, capture.nanos);
        assertEquals(11, capture.xInches, 1e-9);
        assertEquals(22, capture.yInches, 1e-9);
        assertEquals(Math.PI / 4, capture.headingRadians, 1e-9);
        assertEquals(360, capture.turretDegrees, 1e-9);
    }

    @Test public void chassisCrossesHeadingWrapViaShortestArcInBothDirections() {
        for (int sign : new int[]{-1, 1}) {
            AimPoseHistory history = new AimPoseHistory();
            history.add(0, 0, 0, Math.toRadians(sign * 179), 0);
            history.add(20 * MS, 0, 0, Math.toRadians(-sign * 179), 0);
            assertEquals(sign * Math.PI, history.interpolate(10 * MS).headingRadians, 1e-9);
        }
    }

    @Test public void turretInterpolationPreservesMultipleRevolutions() {
        AimPoseHistory history = new AimPoseHistory();
        history.add(0, 0, 0, 0, -720);
        history.add(50 * MS, 0, 0, 0, -360);
        assertEquals(-540, history.interpolate(25 * MS).turretDegrees, 1e-9);
    }

    @Test public void neverExtrapolatesAndSupportsExactEndpoints() {
        AimPoseHistory history = new AimPoseHistory();
        assertNull(history.interpolate(0));
        history.add(100 * MS, 10, 20, 0, 4);
        assertNull(history.interpolate(100 * MS - 1));
        assertNotNull(history.interpolate(100 * MS));
        assertNull(history.interpolate(100 * MS + 1));
        history.add(120 * MS, 30, 40, 0, 8);
        assertEquals(30, history.interpolate(120 * MS).xInches, 0);
        assertNull(history.interpolate(120 * MS + 1));
    }

    @Test public void sparseIntervalsRejectInterpolationButKeepActualMeasurements() {
        AimPoseHistory history = new AimPoseHistory();
        history.add(0, 0, 0, 0, 0);
        assertTrue(history.add(100 * MS + 1, 1, 2, 0, 3));
        assertNull(history.interpolate(50 * MS));
        assertNotNull(history.interpolate(0));
        assertNotNull(history.interpolate(100 * MS + 1));
    }

    @Test public void supportsAConfiguredInterpolationGap() {
        AimPoseHistory history = new AimPoseHistory(25 * MS);
        history.add(0, 0, 0, 0, 0);
        history.add(30 * MS, 0, 0, 0, 0);
        assertNull(history.interpolate(15 * MS));
        assertThrows(IllegalArgumentException.class, () -> new AimPoseHistory(0));
        assertThrows(IllegalArgumentException.class, () -> new AimPoseHistory(-1));
        assertThrows(IllegalArgumentException.class, () -> new AimPoseHistory(250 * MS + 1));
    }

    @Test public void duplicateBackwardsAndExcessiveSampleGapsEraseHistory() {
        for (long rejectedTime : new long[]{100 * MS, 99 * MS, 350 * MS + 1}) {
            AimPoseHistory history = new AimPoseHistory();
            history.add(100 * MS, 0, 0, 0, 0);
            assertFalse(history.add(rejectedTime, 1, 1, 1, 1));
            assertNull(history.interpolate(100 * MS));
            assertNull(history.interpolate(rejectedTime));
            assertTrue(history.add(500 * MS, 2, 2, 2, 2));
            assertNotNull(history.interpolate(500 * MS));
        }
    }

    @Test public void exactlyMaximumGapCanBeRecordedWithoutInterpolation() {
        AimPoseHistory history = new AimPoseHistory();
        history.add(0, 0, 0, 0, 0);
        assertTrue(history.add(250 * MS, 1, 1, 1, 1));
        assertNotNull(history.interpolate(0));
        assertNotNull(history.interpolate(250 * MS));
        assertNull(history.interpolate(125 * MS));
    }

    @Test public void everyNonfiniteMeasurementClearsPriorHistory() {
        for (double invalid : new double[]{Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
            for (int axis = 0; axis < 4; axis++) {
                AimPoseHistory history = new AimPoseHistory();
                history.add(0, 0, 0, 0, 0);
                double[] values = {1, 2, 3, 4};
                values[axis] = invalid;
                assertFalse(history.add(10 * MS, values[0], values[1], values[2], values[3]));
                assertNull(history.interpolate(0));
            }
        }
    }

    @Test public void retainsOnlyOneSecondOfHistory() {
        AimPoseHistory history = new AimPoseHistory();
        for (int i = 0; i <= 11; i++) history.add(i * 100 * MS, i, 0, 0, 0);
        assertNull(history.interpolate(0));
        assertNull(history.interpolate(100 * MS - 1));
        assertNotNull(history.interpolate(100 * MS));
        assertNotNull(history.interpolate(1_100 * MS));
    }

    @Test public void sampleCountIsBoundedEvenForUnusuallyFastSampling() {
        AimPoseHistory history = new AimPoseHistory();
        for (int i = 0; i < 300; i++) history.add(i, i, 0, 0, 0);
        assertNull(history.interpolate(0));
        assertNotNull(history.interpolate(299));
    }

    @Test public void clearPreventsFramesFromCrossingAnOriginOrTurretZeroReset() {
        AimPoseHistory history = new AimPoseHistory();
        history.add(0, 100, 100, 0, 300);
        history.clear();
        history.add(10 * MS, 0, 0, 0, 0);
        assertNull(history.interpolate(0));
        assertNull(history.interpolate(5 * MS));
        assertEquals(0, history.interpolate(10 * MS).turretDegrees, 0);
    }

    @Test public void monotonicClockMayBeNegativeButArithmeticOverflowIsRejected() {
        AimPoseHistory history = new AimPoseHistory();
        assertTrue(history.add(-20 * MS, 0, 0, 0, 0));
        assertTrue(history.add(-10 * MS, 1, 0, 0, 0));
        assertEquals(.5, history.interpolate(-15 * MS).xInches, 1e-9);
        history.clear();
        assertTrue(history.add(Long.MIN_VALUE, 0, 0, 0, 0));
        assertFalse(history.add(Long.MAX_VALUE, 0, 0, 0, 0));
        assertNull(history.interpolate(Long.MIN_VALUE));
    }
}
