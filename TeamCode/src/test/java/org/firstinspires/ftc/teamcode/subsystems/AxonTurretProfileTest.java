package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

/** Integration checks in turret degrees and normalized servo power, with no simulated plant. */
public class AxonTurretProfileTest {
    private final CRServo servo = mock(CRServo.class);
    private final AnalogInput encoder = mock(AnalogInput.class);
    private final AtomicLong now = new AtomicLong();
    private final Map<Field, Object> originalConfiguration = new HashMap<>();
    private double shaftDegrees = 180;
    private AxonTurret turret;

    @Before public void setUp() throws IllegalAccessException {
        for (Field field : AxonTurret.class.getFields()) {
            if (Modifier.isStatic(field.getModifiers()) && !Modifier.isFinal(field.getModifiers())) {
                originalConfiguration.put(field, field.get(null));
            }
        }
        AxonTurret.encoderMinVolts = 0;
        AxonTurret.encoderMaxVolts = 3.3;
        AxonTurret.encoderSign = AxonTurret.servoSign = 1;
        AxonTurret.servoTurnsPerTurretTurn = 5;
        AxonTurret.maximumServoDegreesPerSecond = 1000;
        AxonTurret.maximumSampleGapMs = 100;
        AxonTurret.directionsVerified = true;
        AxonTurret.limitTravel = true;
        AxonTurret.minDegrees = -180;
        AxonTurret.maxDegrees = 180;
        AxonTurret.profileEnabled = true;
        AxonTurret.maxProfileVelocityDegreesPerSecond = 30;
        AxonTurret.maxProfileAccelerationDegreesPerSecondSquared = 60;
        AxonTurret.kP = AxonTurret.kI = AxonTurret.kD = 0;
        AxonTurret.kS = AxonTurret.kV = AxonTurret.kA = 0;
        AxonTurret.maximumPower = 0.8;
        AxonTurret.manualPower = 0.1;
        AxonTurret.integralLimit = 20;
        AxonTurret.toleranceDegrees = 1;
        AxonTurret.settledVelocityDegreesPerSecond = 3;
        AxonTurret.settleTimeMs = 200;
        AxonTurret.moveTimeoutMs = 4000;
        AxonTurret.noMotionTimeoutMs = 10000;
        AxonTurret.noMotionMinimumServoDegrees = 2;
        AxonTurret.noMotionPowerThreshold = 0.05;
        when(encoder.getVoltage()).thenAnswer(call -> shaftDegrees / 360 * 3.3);
        turret = new AxonTurret(servo, encoder, now::get);
    }

    @After public void tearDown() throws IllegalAccessException {
        for (Map.Entry<Field, Object> entry : originalConfiguration.entrySet()) {
            entry.getKey().set(null, entry.getValue());
        }
    }

    @Test public void feedforwardUsesTurretVelocityAndAccelerationThroughAllMovePhases() {
        AxonTurret.kS = 0.015;
        AxonTurret.kV = 0.002;
        AxonTurret.kA = 0.001;
        startMove(60);
        // Zero position error at launch must not suppress acceleration feedforward.
        assertEquals(0.075, turret.getFeedforwardPower(), 1e-8);
        step(100, 0);
        assertEquals(6, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(60, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        assertEquals(0.087, turret.getFeedforwardPower(), 1e-8);
        assertEquals(turret.getFeedforwardPower(), turret.getOutput(), 1e-8);

        advanceAtRest(500); // t = 0.6 seconds: cruise at 30 turret degrees/second.
        assertEquals(30, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(0, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        assertEquals(0.075, turret.getFeedforwardPower(), 1e-8);

        advanceAtRest(1600); // t = 2.2 seconds: slowing to the 60 degree goal.
        assertEquals(18, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(-60, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        assertEquals(-0.009, turret.getFeedforwardPower(), 1e-8);
        assertEquals(-0.009, turret.getOutput(), 1e-8);
        assertEquals(2.5, turret.getProfileDurationSeconds(), 1e-8);
    }

    @Test public void negativeTravelReversesVelocityFrictionAndBrakingAcceleration() {
        AxonTurret.kS = 0.015;
        AxonTurret.kV = 0.002;
        AxonTurret.kA = 0.001;
        startMove(-60);
        step(100, 0);
        assertEquals(-6, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(-60, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        assertEquals(-0.087, turret.getFeedforwardPower(), 1e-8);
        advanceAtRest(2100);
        assertEquals(-18, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(60, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        assertEquals(0.009, turret.getFeedforwardPower(), 1e-8);
    }

    @Test public void entireFeedbackPlusFeedforwardIsClampedBeforeServoDirection() {
        AxonTurret.kP = 0.1;
        AxonTurret.kA = 0.01;
        AxonTurret.maximumPower = 0.15;
        AxonTurret.servoSign = -1;
        startMove(60);
        step(100, 0);
        assertEquals(0.03, turret.getFeedbackPower(), 1e-8);
        assertEquals(0.6, turret.getFeedforwardPower(), 1e-8);
        assertEquals(0.63, turret.getUnclampedPower(), 1e-8);
        assertTrue(turret.isOutputLimited());
        assertEquals(-0.15, turret.getOutput(), 1e-8);
        verify(servo, atLeastOnce()).setPower(-0.15);
    }

    @Test public void feedforwardSaturationDoesNotWindUpIntegral() {
        AxonTurret.kI = 0.1;
        AxonTurret.kA = 0.01;
        AxonTurret.maximumPower = 0.15;
        startMove(60);
        advanceAtRest(400);
        assertTrue(turret.isOutputLimited());
        assertEquals(0, turret.getFeedbackPower(), 1e-8);
        step(100, 0); // Acceleration ends; the first unsaturated integral update is 7.5 * 0.1.
        assertEquals(7.5, turret.getProfilePositionDegrees(), 1e-8);
        assertEquals(0, turret.getFeedforwardPower(), 1e-8);
        assertEquals(0.075, turret.getFeedbackPower(), 1e-8);
        assertEquals(0.075, turret.getOutput(), 1e-8);
        assertFalse(turret.isOutputLimited());
    }

    @Test public void derivativeTracksDesiredVelocityInsteadOfBrakingPlannedMotion() {
        AxonTurret.kD = 0.001;
        startMove(60);
        step(100, 0);
        assertEquals(0.006, turret.getFeedbackPower(), 1e-8);
        step(100, 1.2); // Measured and desired turret velocity both equal 12 degrees/second.
        assertEquals(12, turret.getVelocityDegreesPerSecond(), 1e-8);
        assertEquals(12, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(0, turret.getFeedbackPower(), 1e-8);
    }

    @Test public void liveDashboardGainsApplyToTheRunningInstanceAndResetIntegral() {
        startMove(60);
        step(100, 0); // Profile: 0.3 degrees, 6 deg/s, 60 deg/s^2.
        assertEquals(0, turret.getOutput(), 0);

        AxonTurret.kP = 0.1;
        turret.periodic();
        assertEquals(0.03, turret.getFeedbackPower(), 1e-8);
        AxonTurret.kD = 0.01;
        turret.periodic();
        assertEquals(0.09, turret.getFeedbackPower(), 1e-8);
        AxonTurret.kS = 0.02;
        turret.periodic();
        assertEquals(0.02, turret.getFeedforwardPower(), 1e-8);
        AxonTurret.kV = 0.01;
        turret.periodic();
        assertEquals(0.08, turret.getFeedforwardPower(), 1e-8);
        AxonTurret.kA = 0.001;
        turret.periodic();
        assertEquals(0.14, turret.getFeedforwardPower(), 1e-8);
        assertEquals(0.23, turret.getOutput(), 1e-8);

        AxonTurret.kI = 0.01;
        turret.periodic();
        step(100, 0); // Integral accumulates 1.2 deg * 0.1 s.
        assertEquals(0.2412, turret.getFeedbackPower(), 1e-8);
        AxonTurret.kI = 0.02;
        turret.periodic(); // Editing a gain discards old accumulated error.
        assertEquals(0.24, turret.getFeedbackPower(), 1e-8);
        step(100, 0);
        assertEquals(0.4554, turret.getFeedbackPower(), 1e-8);
        assertEquals(AxonTurret.Mode.POSITION, turret.getMode());
        assertTrue(turret.hasReference());
    }

    @Test public void liveProfileConstraintEditsReplanWithoutLosingPositionOrVelocity() {
        startMove(60);
        advanceAtRest(300);
        double position = turret.getProfilePositionDegrees();
        double velocity = turret.getProfileVelocityDegreesPerSecond();
        assertEquals(18, velocity, 1e-8);

        AxonTurret.maxProfileVelocityDegreesPerSecond = 40;
        AxonTurret.maxProfileAccelerationDegreesPerSecondSquared = 80;
        turret.periodic();
        assertEquals(position, turret.getProfilePositionDegrees(), 1e-8);
        assertEquals(velocity, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(80, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        step(100, 0);
        assertEquals(26, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        step(100, 0);
        step(100, 0);
        assertEquals(40, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(AxonTurret.Mode.POSITION, turret.getMode());
        assertTrue(turret.hasReference());
    }

    @Test public void reachingFinalAngleEarlyDoesNotDeclareAnUnfinishedProfileSettled() {
        startMove(10);
        step(100, 10);
        step(100, 10);
        step(100, 10);
        step(100, 10);
        assertFalse(turret.isProfileFinished());
        assertFalse(turret.isAtTarget());
        for (int i = 0; i < 8; i++) step(100, 10);
        assertTrue(turret.isProfileFinished());
        assertTrue(turret.isAtTarget());
        assertEquals(0, turret.getOutput(), 1e-8);
    }

    @Test public void retargetingPreservesDesiredPositionAndVelocityBeforeBraking() {
        startMove(60);
        advanceAtRest(300);
        double position = turret.getProfilePositionDegrees();
        double velocity = turret.getProfileVelocityDegreesPerSecond();
        assertEquals(18, velocity, 1e-8);
        turret.requestPosition(-30);
        turret.periodic(); // Same timestamp allows an exact continuity comparison.
        assertEquals(position, turret.getProfilePositionDegrees(), 1e-8);
        assertEquals(velocity, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(-60, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
        step(100, 0);
        assertEquals(12, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(position + 1.5, turret.getProfilePositionDegrees(), 1e-8);
    }

    @Test public void releaseAndManualModeDiscardOldProfileBeforeReengagingPosition() {
        AxonTurret.kV = 0.002;
        startMove(60);
        advanceAtRest(300);
        assertTrue(turret.getOutput() > 0);
        turret.idle();
        assertEquals(0, turret.getOutput(), 0);
        assertEquals(0, turret.getFeedforwardPower(), 0);
        assertTrue(Double.isNaN(turret.getProfilePositionDegrees()));

        turret.requestManual(-1);
        step(20, 0);
        assertEquals(-0.1, turret.getOutput(), 1e-8);
        turret.requestPosition(-10);
        step(20, 0);
        assertEquals(0, turret.getProfilePositionDegrees(), 1e-8);
        assertEquals(0, turret.getProfileVelocityDegreesPerSecond(), 1e-8);
        assertEquals(-60, turret.getProfileAccelerationDegreesPerSecondSquared(), 1e-8);
    }

    @Test public void initialProfileUsesMeasuredVelocityAndRejectsUnsafeStoppingExcursion() {
        AxonTurret.minDegrees = -10;
        AxonTurret.maxDegrees = 10;
        zero();
        turret.requestPosition(0);
        step(100, 8); // 80 deg/s needs more stopping distance than the remaining 2 degrees.
        assertFaultAndZero();
        assertTrue(turret.getStatus(), turret.getStatus().toLowerCase().contains("travel"));
    }

    @Test public void invalidProfileConstraintsStopInsteadOfApplyingFeedforward() {
        zero();
        AxonTurret.maxProfileVelocityDegreesPerSecond = 0;
        turret.requestPosition(10);
        step(20, 0);
        assertFaultAndZero();
    }

    @Test public void nonfiniteAccelerationOrFeedforwardGainIsRejected() {
        zero();
        AxonTurret.maxProfileAccelerationDegreesPerSecondSquared = Double.NaN;
        turret.requestPosition(10);
        step(20, 0);
        assertFaultAndZero();

        AxonTurret.maxProfileAccelerationDegreesPerSecondSquared = 60;
        turret.clearFault();
        turret.zeroHere();
        step(20, 0);
        assertTrue(turret.hasReference());
        AxonTurret.kV = Double.POSITIVE_INFINITY;
        turret.requestPosition(10);
        step(20, 0);
        assertFaultAndZero();
    }

    @Test public void moveTimeoutAllowsPlannedDurationThenLimitsFailureToReachGoal() {
        AxonTurret.kP = 0.001;
        AxonTurret.moveTimeoutMs = 250;
        startMove(60);
        assertEquals(2.5, turret.getProfileDurationSeconds(), 1e-8);
        advanceAtRest(2700);
        assertEquals(AxonTurret.Mode.POSITION, turret.getMode());
        assertTrue(turret.isProfileFinished());
        assertFalse(turret.isAtTarget());
        step(100, 0);
        assertFaultAndZero();
        assertTrue(turret.getStatus().contains("timed out"));
    }

    @Test public void brakingOppositeHealthyMotionIsAllowedButEncoderStallsStillFault() {
        AxonTurret.maxProfileAccelerationDegreesPerSecondSquared = 10;
        AxonTurret.kV = 0.001;
        AxonTurret.kA = 0.02;
        AxonTurret.noMotionTimeoutMs = 750;
        startMove(120); // 3 s acceleration, 1 s cruise, 3 s deceleration.
        for (int i = 1; i <= 49; i++) {
            double t = i / 10.0;
            double position = t < 3 ? 5 * t * t
                    : t < 4 ? 45 + 30 * (t - 3)
                    : 75 + 30 * (t - 4) - 5 * (t - 4) * (t - 4);
            step(100, position);
            assertEquals(turret.getStatus(), AxonTurret.Mode.POSITION, turret.getMode());
        }
        // Reverse power has been braking forward motion for longer than the watchdog duration.
        assertTrue(turret.getOutput() < 0);
        assertTrue(turret.getVelocityDegreesPerSecond() > 0);
        assertTrue(turret.getProfileVelocityDegreesPerSecond() > 0);
        assertFalse(turret.isProfileFinished());

        double stalledPosition = turret.getPositionDegrees();
        for (int i = 0; i < 9; i++) step(100, stalledPosition);
        assertFaultAndZero();
        assertTrue(turret.getStatus().contains("No encoder progress"));
    }

    private void zero() {
        turret.enable();
        turret.periodic();
        turret.zeroHere();
        step(20, 0);
        assertTrue(turret.getStatus(), turret.hasReference());
    }

    private void startMove(double target) {
        zero();
        turret.requestPosition(target);
        step(20, 0);
        assertEquals(turret.getStatus(), AxonTurret.Mode.POSITION, turret.getMode());
    }

    private void advanceAtRest(long milliseconds) {
        assertEquals(0, milliseconds % 100);
        for (long i = 0; i < milliseconds; i += 100) step(100, 0);
    }

    private void step(long milliseconds, double turretDegrees) {
        shaftDegrees = ((180 + turretDegrees * AxonTurret.servoTurnsPerTurretTurn) % 360 + 360) % 360;
        now.addAndGet(milliseconds * 1_000_000);
        turret.periodic();
    }

    private void assertFaultAndZero() {
        assertEquals(turret.getStatus(), AxonTurret.Mode.FAULT, turret.getMode());
        assertEquals(0, turret.getOutput(), 0);
        assertFalse(turret.hasReference());
    }
}
