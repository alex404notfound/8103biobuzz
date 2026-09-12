package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class OffsetCalibrationTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void halfTurnWithZeroDeviceCompensationReturnsAbsoluteOffsets() {
        OffsetCalibration.Result result = OffsetCalibration.calculate(
                12.06404197873093,
                -8.182148970956878,
                Math.PI);

        assertEquals(-6.032020989365465, result.strafePodX, EPSILON);
        assertEquals(4.091074485478439, result.forwardPodY, EPSILON);
    }

    @Test
    public void halfTurnIsReadyInEitherDirectionButPartialTurnIsNot() {
        assertFalse(OffsetCalibration.hasCompletedHalfTurn(Math.PI - 1e-6));
        assertFalse(OffsetCalibration.hasCompletedHalfTurn(-Math.PI + 1e-6));
        assertTrue(OffsetCalibration.hasCompletedHalfTurn(Math.PI));
        assertTrue(OffsetCalibration.hasCompletedHalfTurn(-Math.PI));
    }

    @Test
    public void offsetCalculationAccountsForClockwiseOvershoot() {
        OffsetCalibration.Result result = OffsetCalibration.calculate(
                10.270075151412776,
                -9.810891343097646,
                Math.toRadians(-200));

        assertEquals(-6.0, result.strafePodX, EPSILON);
        assertEquals(4.0, result.forwardPodY, EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonfinitePositionCannotBecomeAnOffsetResult() {
        OffsetCalibration.calculate(Double.NaN, 0, Math.PI);
    }

    @Test(expected = IllegalArgumentException.class)
    public void fullRotationCannotProduceADivisionByZeroOffset() {
        OffsetCalibration.calculate(0, 0, 2 * Math.PI);
    }
}
