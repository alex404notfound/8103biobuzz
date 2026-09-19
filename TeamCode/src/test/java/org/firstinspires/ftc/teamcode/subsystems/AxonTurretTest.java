package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class AxonTurretTest {
    private final CRServo servo = mock(CRServo.class);
    private final AnalogInput encoder = mock(AnalogInput.class);
    private final AtomicLong now = new AtomicLong();
    private AxonTurret turret;
    private double rawDegrees = 180;

    @Before public void setUp() {
        defaults();
        when(encoder.getVoltage()).thenAnswer(call -> rawDegrees / 360 * 3.3);
        turret = new AxonTurret(servo, encoder, now::get);
        clearInvocations(servo, encoder);
    }

    @After public void tearDown() {
        defaults();
        AxonTurret.servoTurnsPerTurretTurn = AxonTurret.maximumServoDegreesPerSecond = 0;
        AxonTurret.directionsVerified = false;
        AxonTurret.minDegrees = AxonTurret.maxDegrees = 0;
        AxonTurret.profileEnabled = true;
    }

    private static void defaults() {
        AxonTurret.servoName = "turret"; AxonTurret.encoderName = "turretEncoder";
        AxonTurret.encoderMinVolts = 0; AxonTurret.encoderMaxVolts = 3.3;
        AxonTurret.encoderSign = AxonTurret.servoSign = 1;
        AxonTurret.servoTurnsPerTurretTurn = 5;
        AxonTurret.maximumServoDegreesPerSecond = 1000;
        AxonTurret.maximumSampleGapMs = 100;
        AxonTurret.directionsVerified = true; AxonTurret.limitTravel = true;
        AxonTurret.minDegrees = -90; AxonTurret.maxDegrees = 90;
        AxonTurret.kP = 0.01; AxonTurret.kI = AxonTurret.kD = 0;
        // These original tests exercise direct PID; profiled control has its own integration suite.
        AxonTurret.profileEnabled = false;
        AxonTurret.kS = AxonTurret.kV = AxonTurret.kA = 0;
        AxonTurret.maxProfileVelocityDegreesPerSecond = 30;
        AxonTurret.maxProfileAccelerationDegreesPerSecondSquared = 60;
        AxonTurret.maximumPower = 0.15; AxonTurret.manualPower = 0.10; AxonTurret.integralLimit = 20;
        AxonTurret.toleranceDegrees = 1; AxonTurret.settledVelocityDegreesPerSecond = 3;
        AxonTurret.settleTimeMs = 200; AxonTurret.moveTimeoutMs = 4000;
        AxonTurret.noMotionTimeoutMs = 750; AxonTurret.noMotionMinimumServoDegrees = 2;
        AxonTurret.noMotionPowerThreshold = 0.05;
    }

    @Test public void initNeverMovesAndStopIsTerminal() {
        turret.requestManual(1); turret.requestPosition(10); turret.zeroHere(); turret.periodic();
        assertFalse(turret.hasReference());
        verify(servo, never()).setPower(doubleThat(p -> p != 0));
        zero();
        turret.requestManual(1); step(20, 180);
        assertEquals(0.10, turret.getOutput(), 1e-9);
        turret.stop();
        assertEquals(0, turret.getOutput(), 0);
        assertFalse(turret.hasReference());
        clearInvocations(servo, encoder);
        turret.enable(); turret.clearFault(); turret.zeroHere();
        turret.requestManual(1); turret.requestPosition(10); turret.periodic(); turret.stop();
        verifyNoInteractions(servo, encoder);
    }

    @Test public void pidRequiresKnownZeroAndVerifiedDirectionsButRawJogCanCheckDirections() {
        turret.enable(); turret.requestPosition(10); turret.periodic();
        assertFaultAndZero();
        turret.clearFault();
        AxonTurret.directionsVerified = false;
        turret.zeroHere(); step(20, 180);
        assertFalse(turret.hasReference());
        turret.requestManual(1); step(20, 180);
        assertEquals(0.1, turret.getOutput(), 1e-9);
        turret.idle();
        AxonTurret.directionsVerified = true;
        turret.zeroHere(); step(20, 180);
        assertTrue(turret.hasReference());
        turret.requestPosition(10); step(20, 180);
        assertEquals(0.1, turret.getOutput(), 1e-9);
    }

    @Test public void wrapsFeedbackAndAppliesGearingAndIndependentSigns() {
        rawDegrees = 359;
        zero();
        step(20, 1);
        assertEquals(0.4, turret.getPositionDegrees(), 1e-8);
        step(20, 359);
        assertEquals(0, turret.getPositionDegrees(), 1e-8);

        AxonTurret.encoderSign = -1; AxonTurret.servoSign = -1;
        step(20, 1);
        assertFalse(turret.hasReference());
        turret.zeroHere(); step(20, 1);
        step(20, 359);
        assertEquals(0.4, turret.getPositionDegrees(), 1e-8);
        assertEquals(20, turret.getVelocityDegreesPerSecond(), 1e-8);
        turret.requestPosition(10); step(20, 359);
        assertTrue(turret.getOutput() < 0); // Logical positive turret motion, reversed physical PWM.
    }

    @Test public void positionRequestUsesUnwrappedTurretErrorInsteadOfShortestAngle() {
        AxonTurret.limitTravel = false;
        rawDegrees = 350;
        zero();
        turret.requestPosition(300); step(20, 350);
        assertEquals(0.15, turret.getOutput(), 1e-9);
        for (int i = 1; i <= 12; i++) step(100, (350 + i * 90) % 360);
        assertEquals(216, turret.getPositionDegrees(), 1e-8);
        assertEquals(84, turret.getErrorDegrees(), 1e-8);
        assertEquals(0.15, turret.getOutput(), 1e-9);
    }

    @Test public void limitsBlockOnlyOutwardManualMotionAndRejectOutOfRangeTarget() {
        AxonTurret.minDegrees = -10; AxonTurret.maxDegrees = 10;
        zero();
        step(100, 231);
        turret.requestManual(1); step(20, 231);
        assertEquals(0, turret.getOutput(), 0);
        turret.requestManual(-1); step(20, 231);
        assertEquals(-0.10, turret.getOutput(), 1e-9);
        step(100, 180);
        step(100, 129);
        assertEquals(0, turret.getOutput(), 0);
        turret.requestManual(1); step(20, 129);
        assertEquals(0.10, turret.getOutput(), 1e-9);
        turret.requestPosition(10.01); step(20, 129);
        assertFaultAndZero();
    }

    @Test public void unmeasuredGearingOrLimitsCannotEstablishZero() {
        turret.enable();
        AxonTurret.servoTurnsPerTurretTurn = 0;
        turret.zeroHere(); turret.periodic();
        assertFalse(turret.hasReference());
        AxonTurret.servoTurnsPerTurretTurn = 5;
        AxonTurret.minDegrees = AxonTurret.maxDegrees = 0;
        turret.zeroHere(); step(20, 180);
        assertFalse(turret.hasReference());
        AxonTurret.minDegrees = -90; AxonTurret.maxDegrees = 90;
        turret.zeroHere(); step(20, 180);
        assertTrue(turret.hasReference());
    }

    @Test public void invalidFeedbackAndControlSettingsStopAndLatchFault() {
        zero();
        turret.requestManual(1); step(20, 180);
        rawDegrees = Double.NaN;
        now.addAndGet(20_000_000);
        turret.periodic();
        assertFaultAndZero();
        turret.requestManual(-1); turret.requestPosition(0); turret.idle();
        assertEquals(AxonTurret.Mode.FAULT, turret.getMode());
        rawDegrees = 180;
        turret.clearFault(); step(20, 180);
        AxonTurret.kP = Double.NaN;
        turret.requestManual(1); step(20, 180);
        assertFaultAndZero();
    }

    @Test public void calibrationChangeWhilePoweredStopsAndInvalidatesHome() {
        zero();
        turret.requestManual(1); step(20, 180);
        AxonTurret.servoTurnsPerTurretTurn = 6;
        step(20, 180);
        assertFaultAndZero();
        turret.clearFault();
        turret.requestPosition(10); step(20, 180);
        assertFaultAndZero();
    }

    @Test public void sampleGapFaultNeedsClearAndNewZeroBeforePid() {
        zero();
        turret.requestPosition(10); step(20, 180);
        step(101, 180);
        assertFaultAndZero();
        turret.requestManual(1); step(20, 180);
        assertFaultAndZero();
        turret.clearFault(); step(20, 180);
        assertFalse(turret.hasReference());
        turret.zeroHere(); step(20, 180);
        assertTrue(turret.hasReference());
        turret.requestPosition(10); step(20, 180);
        assertEquals(0.1, turret.getOutput(), 1e-9);
    }

    @Test public void noMotionAndWrongDirectionCannotRunIndefinitely() {
        zero();
        turret.requestManual(1); step(20, 180);
        for (int i = 0; i < 8; i++) step(100, 180);
        assertFaultAndZero();
        assertTrue(turret.getStatus().contains("No encoder progress"));

        turret.clearFault(); step(20, 180);
        turret.zeroHere(); step(20, 180);
        turret.requestManual(1); step(20, 180);
        for (int i = 1; i <= 8; i++) step(100, 180 - i * 3);
        assertFaultAndZero();
    }

    @Test public void moveTimeoutAlsoCatchesSmallOutputsBelowNoMotionThreshold() {
        AxonTurret.moveTimeoutMs = 250;
        AxonTurret.kP = 0.001;
        zero();
        turret.requestPosition(10); step(20, 180);
        assertEquals(0.01, turret.getOutput(), 1e-9);
        for (int i = 0; i < 3; i++) step(100, 180);
        assertFaultAndZero();
        assertTrue(turret.getStatus().contains("timed out"));
    }

    @Test public void integralDoesNotWindUpAgainstSaturationAndResetsOnIdleOrTargetChange() {
        AxonTurret.kP = 0.1; AxonTurret.kI = 0.1;
        AxonTurret.toleranceDegrees = 0.1;
        zero();
        turret.requestPosition(10); step(20, 180);
        for (int i = 0; i < 5; i++) step(100, 180);
        assertEquals(0.15, turret.getOutput(), 1e-9);
        step(50, 227.5); // 9.5 turret degrees: the now-unsaturated integral starts from zero.
        assertEquals(0.0525, turret.getOutput(), 1e-8);
        turret.idle();
        turret.requestPosition(10); step(20, 227.5);
        assertEquals(0.05, turret.getOutput(), 1e-8);
        turret.requestPosition(9.75); step(20, 227.5);
        assertEquals(0.025, turret.getOutput(), 1e-8);
    }

    @Test public void derivativeRespondsToMotionInsteadOfSetpointSteps() {
        AxonTurret.kP = 0; AxonTurret.kD = 0.001;
        zero();
        turret.requestPosition(30); step(20, 180);
        assertEquals(0, turret.getOutput(), 0);
        turret.requestPosition(60); step(20, 180);
        assertEquals(0, turret.getOutput(), 0);
        step(100, 190);
        assertEquals(-0.02, turret.getOutput(), 1e-8);
    }

    @Test public void atTargetNeedsLowVelocityDwellAndFreshSamples() {
        zero();
        turret.requestPosition(10); step(20, 180);
        step(100, 230);
        assertFalse(turret.isAtTarget()); // Within position tolerance while still moving fast.
        step(100, 230);
        step(100, 230);
        assertFalse(turret.isAtTarget());
        step(100, 230);
        assertTrue(turret.isAtTarget());
        now.addAndGet(101_000_000);
        assertFalse(turret.isAtTarget());
    }

    @Test public void sensorExceptionStopsServoAndDiscardsReference() {
        zero();
        turret.requestManual(1); step(20, 180);
        when(encoder.getVoltage()).thenThrow(new IllegalStateException("encoder disconnected"));
        assertThrows(IllegalStateException.class, turret::periodic);
        assertFaultAndZero();
        verify(servo, atLeastOnce()).setPower(0);
    }

    @Test public void manualPowerRespectsGlobalCap() {
        AxonTurret.maximumPower = 0.05;
        zero();
        turret.requestManual(1); step(20, 180);
        assertEquals(0.05, turret.getOutput(), 0);
    }

    @Test public void blockingSensorReadCountsTowardTrackingGapBeforeNextOutput() {
        zero();
        turret.requestManual(1); step(20, 180);
        when(encoder.getVoltage()).thenAnswer(call -> {
            now.addAndGet(101_000_000);
            return 180.0 / 360 * 3.3;
        });
        turret.periodic();
        assertFaultAndZero();
    }

    @Test public void changingCalibrationAtSoftLimitCannotResumeHeldJogWithoutLimits() {
        AxonTurret.minDegrees = -10; AxonTurret.maxDegrees = 10;
        zero();
        step(100, 231);
        turret.requestManual(1); step(20, 231);
        assertEquals(0, turret.getOutput(), 0);
        AxonTurret.maxDegrees = 20;
        turret.requestManual(1); step(20, 231);
        assertFaultAndZero();
    }

    private void zero() {
        turret.enable();
        turret.periodic();
        turret.zeroHere();
        step(20, rawDegrees);
        assertTrue(turret.getStatus(), turret.hasReference());
    }

    private void step(long milliseconds, double shaftDegrees) {
        rawDegrees = shaftDegrees;
        now.addAndGet(milliseconds * 1_000_000);
        turret.periodic();
    }

    private void assertFaultAndZero() {
        assertEquals(AxonTurret.Mode.FAULT, turret.getMode());
        assertEquals(0, turret.getOutput(), 0);
        assertFalse(turret.hasReference());
    }
}
