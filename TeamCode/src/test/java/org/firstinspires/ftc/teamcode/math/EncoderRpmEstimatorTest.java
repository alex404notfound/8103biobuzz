package org.firstinspires.ftc.teamcode.math;

import org.junit.Test;

import static org.junit.Assert.*;

public class EncoderRpmEstimatorTest {
    @Test public void waitsForFullSpanAndUsesActualIrregularSampleTimes() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(2);
        int[] milliseconds = {0, 130, 300, 600, 970, 1200, 1620, 1990};
        for (int millisecondsSinceStart : milliseconds) {
            estimator.addSample(millisecondsSinceStart * 1_000_000L, millisecondsSinceStart);
            assertFalse(estimator.isReady());
        }
        estimator.addSample(2_200_000_000L, 2200);
        assertTrue(estimator.isReady());
        assertEquals(1000, estimator.getCountsPerSecond(), 1e-9);
        assertEquals(2.07, estimator.getSpanSeconds(), 1e-9);
        assertEquals(0, estimator.getResidualCounts(), 1e-9);
    }

    @Test public void fitsManyQuantizedSamplesInsteadOfTwoNoisyEndpoints() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(4);
        for (int i = 0; i <= 200; i++) {
            double seconds = i * 0.02;
            estimator.addSample(i * 20_000_000L, (int) Math.round(seconds * 26.7));
        }
        assertTrue(estimator.isReady());
        assertEquals(201, estimator.getSampleCount());
        assertEquals(26.7, estimator.getCountsPerSecond(), 0.02);
        assertTrue(estimator.getResidualCounts() < 0.31);
    }

    @Test public void rollingWindowForgetsAnOlderSpeed() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(2);
        for (int i = 0; i <= 40; i++) {
            int position = i <= 20 ? i * 100 : 2000 + (i - 20) * 200;
            estimator.addSample(i * 100_000_000L, position);
        }
        assertEquals(2000, estimator.getCountsPerSecond(), 1e-9);
        assertEquals(2, estimator.getSpanSeconds(), 0);
        assertEquals(21, estimator.getSampleCount());
    }

    @Test public void unwrapsTheEncoderAcrossBothSignedIntegerBoundaries() {
        for (int direction : new int[]{1, -1}) {
            EncoderRpmEstimator estimator = new EncoderRpmEstimator(2);
            int origin = direction > 0 ? Integer.MAX_VALUE - 50 : Integer.MIN_VALUE + 50;
            for (int i = 0; i <= 20; i++) {
                estimator.addSample(i * 100_000_000L, origin + direction * i * 100);
            }
            assertEquals(direction * 1000, estimator.getCountsPerSecond(), 1e-9);
        }
    }

    @Test public void centeringPreservesPrecisionAtLargeClockAndEncoderValues() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(2);
        long clockOrigin = 8_000_000_000_000_000_000L;
        int positionOrigin = Integer.MAX_VALUE - 50;
        for (int i = 0; i <= 20; i++) {
            estimator.addSample(clockOrigin + i * 100_000_000L, positionOrigin + i * 100);
        }
        assertEquals(1000, estimator.getCountsPerSecond(), 1e-9);
        assertEquals(0, estimator.getResidualCounts(), 1e-9);
    }

    @Test public void communicationGapInvalidatesFitUntilAnotherFullWindowArrives() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(1);
        for (int i = 0; i <= 4; i++) estimator.addSample(i * 250_000_000L, i * 250);
        assertTrue(estimator.isReady());
        estimator.addSample(2_000_000_000L, 2000);
        assertFalse(estimator.isReady());
        assertEquals(1, estimator.getSampleCount());
        for (int i = 1; i <= 4; i++) estimator.addSample(2_000_000_000L + i * 250_000_000L, 2000 + i * 250);
        assertTrue(estimator.isReady());
        assertEquals(1000, estimator.getCountsPerSecond(), 1e-9);
    }

    @Test public void nonIncreasingTimeAndExplicitResetDiscardOldMeasurements() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(1);
        for (int i = 0; i <= 4; i++) estimator.addSample(i * 250_000_000L, i * 250);
        estimator.addSample(1_000_000_000L, 1001);
        assertFalse(estimator.isReady());
        assertEquals(1, estimator.getSampleCount());
        estimator.addSample(900_000_000L, 900);
        assertEquals(1, estimator.getSampleCount());
        estimator.reset();
        assertEquals(0, estimator.getSampleCount());
        assertEquals(0, estimator.getSpanSeconds(), 0);
        assertTrue(Double.isNaN(estimator.getCountsPerSecond()));
        assertTrue(Double.isNaN(estimator.getResidualCounts()));
    }

    @Test public void stationaryEncoderReportsZeroAfterFullWindow() {
        EncoderRpmEstimator estimator = new EncoderRpmEstimator(1);
        for (int i = 0; i <= 4; i++) estimator.addSample(i * 250_000_000L, 12345);
        assertEquals(0, estimator.getCountsPerSecond(), 0);
        assertEquals(0, estimator.getResidualCounts(), 0);
    }

    @Test public void rejectsInvalidDurations() {
        for (double invalid : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY, 1e12}) {
            assertThrows(IllegalArgumentException.class, () -> new EncoderRpmEstimator(invalid));
            assertThrows(IllegalArgumentException.class, () -> new EncoderRpmEstimator(1, invalid));
        }
    }
}
