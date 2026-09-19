package org.firstinspires.ftc.teamcode.math;

import org.junit.Test;

import static org.firstinspires.ftc.teamcode.math.FieldSideEstimator.Lateral.*;
import static org.firstinspires.ftc.teamcode.math.FieldSideEstimator.Longitudinal.AUDIENCE_HALF;
import static org.firstinspires.ftc.teamcode.math.FieldSideEstimator.Longitudinal.FAR_HALF;
import static org.junit.Assert.*;

public class FieldSideEstimatorTest {
    @Test public void labelsAllQuadrantsInTheDocumentedFieldFrame() {
        assertSide(10, 10, RED_HALF, AUDIENCE_HALF);
        assertSide(10, 130, RED_HALF, FAR_HALF);
        assertSide(130, 10, BLUE_HALF, AUDIENCE_HALF);
        assertSide(130, 130, BLUE_HALF, FAR_HALF);
    }

    @Test public void positionAloneCannotSupplyAnUninitializedFieldReference() {
        assertUnknown(FieldSideEstimator.estimate(10, 10, false, true, 2));
        assertUnknown(FieldSideEstimator.estimate(130, 130, true, false, 2));
        assertUnknown(FieldSideEstimator.estimate(72, 72, false, false, 2));
    }

    @Test public void uncertaintyTouchingCenterLineProducesCenterBandOnEachAxis() {
        for (double x : new double[]{69, 72, 75}) {
            FieldSideEstimator.Result result = FieldSideEstimator.estimate(x, 30, true, true, 3);
            assertEquals(CENTER_BAND, result.lateral);
            assertEquals(AUDIENCE_HALF, result.longitudinal);
            assertTrue(result.isKnown());
        }
        for (double y : new double[]{69, 72, 75}) {
            FieldSideEstimator.Result result = FieldSideEstimator.estimate(100, y, true, true, 3);
            assertEquals(BLUE_HALF, result.lateral);
            assertEquals(FieldSideEstimator.Longitudinal.CENTER_BAND, result.longitudinal);
        }
        assertEquals(RED_HALF, FieldSideEstimator.estimate(68.999, 30, true, true, 3).lateral);
        assertEquals(BLUE_HALF, FieldSideEstimator.estimate(75.001, 30, true, true, 3).lateral);
    }

    @Test public void zeroUncertaintyStillLabelsExactCenterAsCenterBand() {
        FieldSideEstimator.Result result = FieldSideEstimator.estimate(72, 72, true, true, 0);
        assertEquals(CENTER_BAND, result.lateral);
        assertEquals(FieldSideEstimator.Longitudinal.CENTER_BAND, result.longitudinal);
    }

    @Test public void rejectsNonfiniteOrOutOfFieldPositions() {
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -.001, 144.001}) {
            assertUnknown(FieldSideEstimator.estimate(invalid, 50, true, true, 2));
            assertUnknown(FieldSideEstimator.estimate(50, invalid, true, true, 2));
        }
        assertSide(0, 0, RED_HALF, AUDIENCE_HALF);
        assertSide(144, 144, BLUE_HALF, FAR_HALF);
    }

    @Test public void rejectsInvalidUncertaintySettings() {
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -.001, 72, 100}) {
            assertUnknown(FieldSideEstimator.estimate(10, 10, true, true, invalid));
        }
    }

    @Test public void crossingSidesOnlyChangesPositionLabels() {
        FieldSideEstimator.Result red = FieldSideEstimator.estimate(40, 110, true, true, 4);
        FieldSideEstimator.Result center = FieldSideEstimator.estimate(72, 110, true, true, 4);
        FieldSideEstimator.Result blue = FieldSideEstimator.estimate(100, 110, true, true, 4);
        assertEquals(RED_HALF, red.lateral);
        assertEquals(CENTER_BAND, center.lateral);
        assertEquals(BLUE_HALF, blue.lateral);
        assertEquals(FAR_HALF, red.longitudinal);
        assertEquals(FAR_HALF, center.longitudinal);
        assertEquals(FAR_HALF, blue.longitudinal);
    }

    private static void assertSide(double x, double y, FieldSideEstimator.Lateral lateral,
                                   FieldSideEstimator.Longitudinal longitudinal) {
        FieldSideEstimator.Result result = FieldSideEstimator.estimate(x, y, true, true, 2);
        assertTrue(result.isKnown());
        assertEquals(lateral, result.lateral);
        assertEquals(longitudinal, result.longitudinal);
    }

    private static void assertUnknown(FieldSideEstimator.Result result) {
        assertFalse(result.isKnown());
        assertEquals(UNKNOWN, result.lateral);
        assertEquals(FieldSideEstimator.Longitudinal.UNKNOWN, result.longitudinal);
    }
}
