package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import org.junit.Test;
import java.util.Arrays;
import static org.junit.Assert.*;

public class CalibrationValuesTest {
    @Test public void finiteInputAtEitherBoundaryIsAccepted() {
        assertEquals(15, CalibrationValues.range(15, 15, 96, "distance"), 0);
        assertEquals(96, CalibrationValues.range(96, 15, 96, "distance"), 0);
    }
    @Test(expected = IllegalArgumentException.class) public void nanDistanceCannotStartAnUnboundedDrive() {
        CalibrationValues.range(Double.NaN, 15, 96, "distance");
    }
    @Test(expected = IllegalArgumentException.class) public void negativeDistanceIsRejected() {
        CalibrationValues.range(-1, 15, 96, "distance");
    }
    @Test(expected = IllegalArgumentException.class) public void stationaryRobotCannotProduceAValidMaximumVelocity() {
        CalibrationValues.positive(0, "velocity");
    }
    @Test(expected = IllegalArgumentException.class) public void infiniteFitCannotBecomeMotorControlParameters() {
        CalibrationValues.coefficients(Arrays.asList(1.0, Double.POSITIVE_INFINITY), 2);
    }
    @Test(expected = IllegalArgumentException.class) public void negativeFeedbackIsRejected() {
        CalibrationValues.positiveCoefficients(Arrays.asList(1.0, -0.5), 2);
    }
    @Test(expected = IllegalArgumentException.class) public void incompleteFitIsRejected() {
        CalibrationValues.coefficients(Arrays.asList(1.0), 2);
    }

    @Test public void slowStrafeMeasurementLowersTheSuggestedDecelerationTarget() {
        assertEquals(16, CalibrationValues.defaultDecelerationTarget(60, 20), 0);
        assertEquals(16, CalibrationValues.maximumDecelerationTarget(60, 20), 0);
        assertEquals(16, CalibrationValues.decelerationTarget(16, 60, 20), 0);
    }

    @Test public void fasterRobotKeepsTheThirtyInchPerSecondDefault() {
        assertEquals(30, CalibrationValues.defaultDecelerationTarget(60, 50), 0);
    }

    @Test(expected = IllegalArgumentException.class) public void targetAboveSlowerMeasuredMaximumCannotStartMotion() {
        CalibrationValues.decelerationTarget(30, 60, 20);
    }

    @Test(expected = IllegalArgumentException.class) public void implausiblyLowMaximumRejectsTheDecelerationStage() {
        CalibrationValues.defaultDecelerationTarget(60, 1);
    }

    @Test(expected = IllegalArgumentException.class) public void invalidMaximumRejectsTheDecelerationStage() {
        CalibrationValues.defaultDecelerationTarget(Double.NaN, 50);
    }

    @Test(expected = IllegalArgumentException.class) public void nonfiniteSelectedTargetCannotStartMotion() {
        CalibrationValues.decelerationTarget(Double.POSITIVE_INFINITY, 60, 20);
    }
}
