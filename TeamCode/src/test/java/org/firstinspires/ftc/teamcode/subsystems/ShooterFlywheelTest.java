package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

/** Tests the linked flywheel as one mechanism, including failed feedback and tuning changes. */
public class ShooterFlywheelTest {
    private final DcMotorEx left = mock(DcMotorEx.class), right = mock(DcMotorEx.class);
    private final AtomicLong now = new AtomicLong();
    private double batteryVolts = 12;
    private ShooterFlywheel flywheel;

    @Before public void setUp() {
        defaults();
        flywheel = new ShooterFlywheel(left, right, () -> batteryVolts, now::get);
        clearInvocations(left, right);
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        ShooterFlywheel.leftMotorName = "launcherLeft";
        ShooterFlywheel.rightMotorName = "launcherRight";
        ShooterFlywheel.leftReversed = ShooterFlywheel.rightReversed = false;
        ShooterFlywheel.directionsVerified = false;
        ShooterFlywheel.kP = .002;
        ShooterFlywheel.kI = ShooterFlywheel.kD = ShooterFlywheel.kS = ShooterFlywheel.kA = 0;
        ShooterFlywheel.kV = 12.0 / 6000;
        ShooterFlywheel.maxVoltage = 3;
        ShooterFlywheel.testVoltage = 1;
        ShooterFlywheel.maxAccelerationRpmPerSecond = 1000;
        ShooterFlywheel.rpmTolerance = 150;
        ShooterFlywheel.speedDwellMs = 250;
        ShooterFlywheel.spinupTimeoutMs = 4000;
        ShooterFlywheel.maxEncoderDifferenceRpm = 250;
        ShooterFlywheel.encoderMismatchDwellMs = 300;
        ShooterFlywheel.encoderResponseTimeoutMs = 1000;
        ShooterFlywheel.minimumEncoderRpm = 50;
        ShooterFlywheel.maximumSampleGapMs = 250;
        ShooterFlywheel.integralLimitRpmSeconds = 5000;
    }

    @Test public void initializationUsesPowerControlAndFloatWithNoNestedHubVelocityLoop() {
        new ShooterFlywheel(left, right, () -> batteryVolts, now::get);
        verify(left).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(right).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(left).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        verify(right).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        verify(left, never()).setVelocity(anyDouble());
        verify(right, never()).setVelocity(anyDouble());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void initAndTerminalStopRejectEveryStartRequest() {
        ShooterFlywheel.directionsVerified = true;
        flywheel.requestRpm(1000);
        flywheel.requestVoltage(1);
        flywheel.requestMotorTest(true);
        flywheel.periodic();
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
        flywheel.enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        assertTrue(flywheel.getPower() > 0);
        flywheel.stop();
        clearInvocations(left, right);
        flywheel.enable();
        flywheel.idle();
        flywheel.requestRpm(1000);
        flywheel.requestVoltage(1);
        flywheel.requestMotorTest(false);
        flywheel.periodic();
        assertEquals(0, flywheel.getPower(), 0);
        assertFalse(flywheel.isAtSpeed());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void unverifiedDirectionsBlockPairedPowerButPermitIndividualDirectionChecks() {
        flywheel.enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        flywheel.idle();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
        flywheel.idle();
        flywheel.requestMotorTest(true);
        flywheel.periodic();
        assertEquals(ShooterFlywheel.Mode.LEFT_TEST, flywheel.getMode());
        verify(left).setPower(1.0 / 12);
        verify(right, never()).setPower(doubleThat(power -> power != 0));
        clearInvocations(left, right);
        flywheel.requestMotorTest(false);
        step(20);
        assertEquals(ShooterFlywheel.Mode.RIGHT_TEST, flywheel.getMode());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right).setPower(1.0 / 12);
    }

    @Test public void linkedMotorsReceiveIdenticalBatteryCompensatedVoltageAndPower() {
        enable();
        flywheel.requestVoltage(3);
        flywheel.periodic();
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        verify(left).setPower(.25);
        verify(right).setPower(.25);
        batteryVolts = 10;
        clearInvocations(left, right);
        step(20);
        assertEquals(10, flywheel.getBatteryVolts(), 1e-9);
        assertEquals(.3, flywheel.getPower(), 1e-9);
        verify(left).setPower(.3);
        verify(right).setPower(.3);
        verify(left, never()).setVelocity(anyDouble());
        verify(right, never()).setVelocity(anyDouble());
    }

    @Test public void voltageCapAndAvailableBatteryLimitBothMotorsTogether() {
        enable();
        flywheel.requestVoltage(6);
        flywheel.periodic();
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        assertTrue(flywheel.isOutputLimited());
        ShooterFlywheel.maxVoltage = 12;
        batteryVolts = 9;
        flywheel.requestVoltage(12);
        step(20);
        assertEquals(9, flywheel.getAppliedVolts(), 1e-9);
        assertEquals(1, flywheel.getPower(), 1e-9);
        assertTrue(flywheel.isOutputLimited());
    }

    @Test public void velocityUses28TicksPerMotorRevolutionAndOneAverageSpeedController() {
        ShooterFlywheel.kV = 0;
        ShooterFlywheel.maxAccelerationRpmPerSecond = 10000;
        rpm(1000, 1200);
        enable();
        flywheel.requestRpm(1300);
        flywheel.periodic();
        assertEquals(1000, flywheel.getLeftRpm(), 1e-9);
        assertEquals(1200, flywheel.getRightRpm(), 1e-9);
        assertEquals(1100, flywheel.getReferenceRpm(), 1e-9);
        step(20);
        assertEquals(1300, flywheel.getReferenceRpm(), 1e-9);
        assertEquals(.4, flywheel.getFeedbackVolts(), 1e-9);
        assertEquals(.4 / 12, flywheel.getPower(), 1e-9);
        verify(left).setPower(.4 / 12);
        verify(right).setPower(.4 / 12);
    }

    @Test public void feedforwardUsesRpmAndRpmPerSecondFromAccelerationLimitedReference() {
        ShooterFlywheel.kP = 0;
        ShooterFlywheel.kS = .2;
        ShooterFlywheel.kV = .001;
        ShooterFlywheel.kA = .0001;
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        assertEquals(0, flywheel.getReferenceRpm(), 0);
        assertEquals(0, flywheel.getReferenceAccelerationRpmPerSecond(), 0);
        step(100);
        assertEquals(100, flywheel.getReferenceRpm(), 1e-9);
        assertEquals(1000, flywheel.getReferenceAccelerationRpmPerSecond(), 1e-9);
        assertEquals(.2 + .001 * 100 + .0001 * 1000, flywheel.getFeedforwardVolts(), 1e-9);
        assertEquals(.4, flywheel.getAppliedVolts(), 1e-9);
        step(200);
        assertEquals(200, flywheel.getReferenceRpm(), 1e-9);
        assertEquals(.5, flywheel.getFeedforwardVolts(), 1e-9);
    }

    @Test public void repeatedHeldRequestDoesNotRestartRampAndRetargetingDoesNotJumpReference() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(100);
        flywheel.requestRpm(1000);
        step(200);
        assertEquals(200, flywheel.getReferenceRpm(), 1e-9);
        flywheel.requestRpm(2000);
        step(300);
        assertEquals(300, flywheel.getReferenceRpm(), 1e-9);
        assertEquals(2000, flywheel.getTargetRpm(), 1e-9);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void zeroTargetImmediatelyStopsWithoutFeedforwardOrAStoppingRamp() {
        enable();
        rpm(1000, 1000);
        flywheel.requestRpm(2000);
        flywheel.periodic();
        assertTrue(flywheel.getPower() > 0);
        clearInvocations(left, right);
        flywheel.requestRpm(0);
        assertEquals(ShooterFlywheel.Mode.OFF, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
        verify(left).setPower(0);
        verify(right).setPower(0);
        step(20);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void readyRequiresFinalReferenceThenContinuousDwellAndFreshSamples() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        rpm(1000, 1000);
        enable();
        flywheel.requestRpm(2000);
        flywheel.periodic();
        rpm(1990, 1990);
        step(250); step(500); step(750);
        assertFalse(flywheel.isAtSpeed());
        step(1000);
        assertEquals(2000, flywheel.getReferenceRpm(), 1e-9);
        assertFalse(flywheel.isAtSpeed());
        step(1125);
        assertFalse(flywheel.isAtSpeed());
        step(1250);
        assertTrue(flywheel.isAtSpeed());
        now.set(1501_000_000L);
        assertFalse(flywheel.hasFreshSample());
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void averageAtTargetCannotHideOneEncoderOutsideSpeedTolerance() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        ShooterFlywheel.rpmTolerance = 50;
        rpm(900, 1100);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(125); step(250);
        assertFalse(flywheel.isAtSpeed());
        rpm(1000, 1000);
        step(300); step(425); step(550);
        assertTrue(flywheel.isAtSpeed());
        rpm(900, 1100);
        step(570);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void encoderDisagreementMustPersistBeforeItStopsBothMotors() {
        enable();
        rpm(500, 1000);
        flywheel.requestVoltage(1);
        flywheel.periodic();
        step(150);
        assertEquals(ShooterFlywheel.Mode.VOLTAGE, flywheel.getMode());
        rpm(1000, 1000);
        step(200);
        rpm(500, 1000);
        step(250); step(450);
        assertEquals(ShooterFlywheel.Mode.VOLTAGE, flywheel.getMode());
        clearInvocations(left, right);
        step(550);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
        verify(left).setPower(0);
        verify(right).setPower(0);
    }

    @Test public void wrongEncoderDirectionLatchesFaultUntilReleased() {
        enable();
        rpm(-100, 100);
        flywheel.requestVoltage(1);
        flywheel.periodic();
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
        rpm(100, 100);
        flywheel.requestVoltage(1);
        step(20);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        flywheel.idle();
        flywheel.requestVoltage(1);
        step(40);
        assertEquals(ShooterFlywheel.Mode.VOLTAGE, flywheel.getMode());
        assertTrue(flywheel.getPower() > 0);
    }

    @Test public void poweredFlywheelWithNoEncoderResponseStopsInsteadOfRunningIndefinitely() {
        enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        step(250); step(500); step(750); step(1000); step(1250);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
    }

    @Test public void staleControlLoopCannotResumePowerWithoutRelease() {
        enable();
        rpm(1000, 1000);
        flywheel.requestVoltage(1);
        flywheel.periodic();
        clearInvocations(left, right);
        step(251);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
        flywheel.requestVoltage(1);
        step(270);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
    }

    @Test public void invalidBatteryOrEncoderDataNeverProducesNonzeroOrNonfinitePower() {
        enable();
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, 0, -1}) {
            batteryVolts = invalid;
            flywheel.idle();
            flywheel.requestVoltage(1);
            flywheel.periodic();
            assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
            assertEquals(0, flywheel.getPower(), 0);
        }
        batteryVolts = 12;
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
            when(left.getVelocity()).thenReturn(invalid);
            flywheel.idle();
            flywheel.requestRpm(1000);
            flywheel.periodic();
            assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        }
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void impossibleTargetOrGainFailsClosed() {
        enable();
        for (double invalid : new double[] {-1, 6001, Double.NaN, Double.POSITIVE_INFINITY}) {
            flywheel.idle();
            flywheel.requestRpm(invalid);
            flywheel.periodic();
            assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        }
        flywheel.idle();
        ShooterFlywheel.kV = Double.NaN;
        flywheel.requestRpm(1000);
        flywheel.periodic();
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void integralDoesNotWindUpWhenFeedforwardAloneExceedsVoltageCap() {
        ShooterFlywheel.kP = 0;
        ShooterFlywheel.kI = .001;
        ShooterFlywheel.kV = .003;
        rpm(1000, 1000);
        enable();
        flywheel.requestRpm(2000);
        flywheel.periodic();
        step(250); step(500); step(750); step(1000);
        assertTrue(flywheel.isOutputLimited());
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        assertEquals(0, flywheel.getFeedbackVolts(), 1e-9);
    }

    @Test public void integralAccumulatesInSecondsAndDashboardGainEditClearsItsHistory() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        ShooterFlywheel.kI = .001;
        ShooterFlywheel.maxAccelerationRpmPerSecond = 10000;
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(100);
        assertEquals(.1, flywheel.getFeedbackVolts(), 1e-9);
        step(200);
        assertEquals(.2, flywheel.getFeedbackVolts(), 1e-9);
        ShooterFlywheel.kI = .002;
        flywheel.periodic();
        assertEquals(0, flywheel.getFeedbackVolts(), 1e-9);
    }

    @Test public void dashboardVoltageCapEditTakesEffectOnNextLoop() {
        enable();
        flywheel.requestVoltage(3);
        flywheel.periodic();
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        ShooterFlywheel.maxVoltage = 1;
        step(20);
        assertEquals(1, flywheel.getAppliedVolts(), 1e-9);
        assertEquals(1.0 / 12, flywheel.getPower(), 1e-9);
    }

    @Test public void fallingBelowSpeedAfterReadyRestartsTrackingTimeout() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        ShooterFlywheel.spinupTimeoutMs = 300;
        rpm(1000, 1000);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(125); step(250);
        assertTrue(flywheel.isAtSpeed());
        rpm(500, 500);
        step(300); step(500); step(600);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
    }

    @Test public void newTargetGetsItsRampTimeBeforeTrackingTimeoutStarts() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        ShooterFlywheel.maxAccelerationRpmPerSecond = 10000;
        ShooterFlywheel.spinupTimeoutMs = 300;
        rpm(500, 500);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(50); step(250);
        flywheel.requestRpm(3000);
        step(300); step(400);
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertEquals(2500, flywheel.getReferenceRpm(), 1e-9);
        step(500);
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertEquals(3000, flywheel.getReferenceRpm(), 1e-9);
        step(700); step(800);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
    }

    @Test public void repeatedTargetEditsCannotHideAMissingEncoderResponse() {
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(250);
        flywheel.requestRpm(2000);
        step(500);
        flywheel.requestRpm(3000);
        step(750);
        flywheel.requestRpm(4000);
        step(1000);
        flywheel.requestRpm(5000);
        step(1250);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
    }

    @Test public void encoderDisagreementPreventsReadyBeforeItsFaultDwellExpires() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        rpm(850, 1150);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(125); step(250);
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void derivativeRespondsToMeasuredAccelerationWithoutSetpointKickOrReversePower() {
        ShooterFlywheel.kP = ShooterFlywheel.kV = 0;
        ShooterFlywheel.kD = .001;
        rpm(1000, 1000);
        enable();
        flywheel.requestRpm(2000);
        flywheel.periodic();
        step(100);
        assertEquals(0, flywheel.getFeedbackVolts(), 1e-9);
        rpm(1100, 1100);
        step(200);
        assertEquals(-1, flywheel.getFeedbackVolts(), 1e-9);
        assertEquals(0, flywheel.getPower(), 0);
        assertTrue(flywheel.isOutputLimited());
        verify(left, never()).setPower(doubleThat(power -> power < 0));
        verify(right, never()).setPower(doubleThat(power -> power < 0));
    }

    @Test public void dashboardDirectionEditStopsUntilHardwareIsReinitialized() {
        enable();
        rpm(1000, 1000);
        flywheel.requestVoltage(1);
        flywheel.periodic();
        ShooterFlywheel.rightReversed = true;
        clearInvocations(left, right);
        step(20);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void stopAttemptsBothMotorsEvenWhenOneControllerThrows() {
        enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        clearInvocations(left, right);
        doThrow(new IllegalStateException("disconnected")).when(left).setPower(0);
        assertThrows(IllegalStateException.class, flywheel::stop);
        verify(right).setPower(0);
        doNothing().when(left).setPower(0);
        clearInvocations(left, right);
        flywheel.enable();
        flywheel.idle();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        verify(right, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void encoderReadExceptionStopsBothPreviouslyPoweredMotors() {
        enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        clearInvocations(left, right);
        when(left.getVelocity()).thenThrow(new IllegalStateException("encoder disconnected"));
        assertThrows(IllegalStateException.class, flywheel::periodic);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left).setPower(0);
        verify(right).setPower(0);
    }

    private void enable() {
        ShooterFlywheel.directionsVerified = true;
        flywheel.enable();
    }

    private void rpm(double leftRpm, double rightRpm) {
        when(left.getVelocity()).thenReturn(leftRpm * 28 / 60);
        when(right.getVelocity()).thenReturn(rightRpm * 28 / 60);
    }

    private void step(long milliseconds) {
        now.set(milliseconds * 1_000_000L);
        flywheel.periodic();
    }
}
