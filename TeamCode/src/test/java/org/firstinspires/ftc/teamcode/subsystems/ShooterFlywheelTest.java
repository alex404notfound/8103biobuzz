package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

/** Tests one shooter with its own feedback, including failed sensors and tuning changes. */
public class ShooterFlywheelTest {
    private final DcMotorEx left = mock(DcMotorEx.class), right = mock(DcMotorEx.class);
    private final AtomicLong now = new AtomicLong();
    private double batteryVolts = 12;
    private ShooterFlywheel flywheel;
    private final ShooterFlywheel.Settings settings = new ShooterFlywheel.Settings("launcherLeft");

    @Before public void setUp() {
        flywheel = new ShooterFlywheel(left, settings, () -> batteryVolts, now::get);
        clearInvocations(left, right);
    }

    @Test public void initializationUsesPowerControlAndBrakeWithNoNestedHubVelocityLoop() {
        new ShooterFlywheel(left, settings, () -> batteryVolts, now::get);
        verify(left).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(left).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(left, never()).setVelocity(anyDouble());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void initAndTerminalStopRejectEveryStartRequest() {
        settings.directionVerified = true;
        flywheel.requestRpm(1000);
        flywheel.requestVoltage(1);
        flywheel.requestMotorTest();
        flywheel.periodic();
        verify(left, never()).setPower(doubleThat(power -> power != 0));
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
        flywheel.requestMotorTest();
        flywheel.periodic();
        assertEquals(0, flywheel.getPower(), 0);
        assertFalse(flywheel.isAtSpeed());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void unverifiedDirectionBlocksRpmButPermitsLowVoltageCheck() {
        flywheel.enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        flywheel.idle();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        verify(left, never()).setPower(doubleThat(power -> power != 0));
        flywheel.idle();
        flywheel.requestMotorTest();
        flywheel.periodic();
        assertEquals(ShooterFlywheel.Mode.DIRECTION_TEST, flywheel.getMode());
        verify(left).setPower(1.0 / 12);
        verifyNoInteractions(right);
    }

    @Test public void motorReceivesBatteryCompensatedVoltageAndPower() {
        enable();
        flywheel.requestVoltage(3);
        flywheel.periodic();
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        verify(left).setPower(.25);
        batteryVolts = 10;
        clearInvocations(left, right);
        step(20);
        assertEquals(10, flywheel.getBatteryVolts(), 1e-9);
        assertEquals(.3, flywheel.getPower(), 1e-9);
        verify(left).setPower(.3);
        verify(left, never()).setVelocity(anyDouble());
    }

    @Test public void ownEncoderClosesTheLoopAndCanReachReady() {
        rpm(1000, 0);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(100); step(200); step(300); step(400);
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertTrue(flywheel.isAtSpeed());
        assertEquals(2, flywheel.getAppliedVolts(), 1e-9);
        verify(left, atLeastOnce()).setPower(2.0 / 12);
    }

    @Test public void voltageCapAndAvailableBatteryLimitOutput() {
        enable();
        flywheel.requestVoltage(6);
        flywheel.periodic();
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        assertTrue(flywheel.isOutputLimited());
        settings.maxVoltage = 12;
        batteryVolts = 9;
        flywheel.requestVoltage(12);
        step(20);
        assertEquals(9, flywheel.getAppliedVolts(), 1e-9);
        assertEquals(1, flywheel.getPower(), 1e-9);
        assertTrue(flywheel.isOutputLimited());
    }

    @Test public void velocityUsesConfiguredMotorEncoderResolution() {
        settings.kV = 0;
        settings.maxAccelerationRpmPerSecond = 10000;
        rpm(1000, 1200);
        enable();
        flywheel.requestRpm(1200);
        flywheel.periodic();
        assertEquals(1000, flywheel.getMeasuredRpm(), 1e-9);
        assertEquals(1000, flywheel.getReferenceRpm(), 1e-9);
        step(20);
        assertEquals(1200, flywheel.getReferenceRpm(), 1e-9);
        assertEquals(.4, flywheel.getFeedbackVolts(), 1e-9);
        assertEquals(.4 / 12, flywheel.getPower(), 1e-9);
        verify(left).setPower(.4 / 12);
    }

    @Test public void feedforwardUsesRpmAndRpmPerSecondFromAccelerationLimitedReference() {
        settings.kP = 0;
        settings.kS = .2;
        settings.kV = .001;
        settings.kA = .0001;
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
        settings.kP = settings.kV = 0;
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
        step(20);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void readyRequiresFinalReferenceThenContinuousDwellAndFreshSamples() {
        settings.kP = settings.kV = 0;
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

    @Test public void ownFeedbackDeterminesSpeedToleranceAndDwell() {
        settings.kP = settings.kV = 0;
        settings.rpmTolerance = 50;
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

    @Test public void unusedEncoderDataCannotAlterControlOrRaiseAFault() {
        enable();
        rpm(1000, 0);
        flywheel.requestVoltage(1);
        flywheel.periodic();
        for (double unrelated : new double[]{0, -6000, 6000, Double.NaN, Double.POSITIVE_INFINITY}) {
            when(right.getVelocity()).thenReturn(unrelated);
            now.addAndGet(200_000_000L);
            flywheel.periodic();
            assertEquals(ShooterFlywheel.Mode.VOLTAGE, flywheel.getMode());
            assertEquals(1, flywheel.getAppliedVolts(), 1e-9);
        }
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

    @Test public void feedforwardTuningRunsWithoutEncoderAndRestartsAfterZeroGain() {
        enable();
        settings.maxVoltage = 12;
        ShooterFlywheel.SpeedTuning tuning = new ShooterFlywheel.SpeedTuning();
        tuning.kV = 0.2;
        for (int ms = 0; ms <= 1600; ms += 200) {
            now.set(ms * 1_000_000L);
            flywheel.updateTuningRpm(3000, tuning);
        }
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());

        tuning.kP = 0;
        for (int ms = 1800; ms <= 6800; ms += 200) {
            now.set(ms * 1_000_000L);
            flywheel.updateTuningRpm(3000, tuning);
        }
        assertEquals(ShooterFlywheel.Mode.FEEDFORWARD, flywheel.getMode());
        assertEquals(12, flywheel.getAppliedVolts(), 0);
        assertTrue(flywheel.isOutputLimited());

        tuning.kV = 0;
        flywheel.updateTuningRpm(3000, tuning);
        assertEquals(ShooterFlywheel.Mode.OFF, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);

        now.addAndGet(1_000_000_000L);
        tuning.kV = 0.2;
        flywheel.updateTuningRpm(3000, tuning);
        assertEquals(ShooterFlywheel.Mode.FEEDFORWARD, flywheel.getMode());
        assertEquals(1, flywheel.getPower(), 0);
    }

    @Test public void rpmTuningRecoversAfterALoopGapAndKeepsRunningBelowTarget() {
        enable();
        settings.maxVoltage = 12;
        settings.maxAccelerationRpmPerSecond = 6000;
        rpm(1000, 0);
        ShooterFlywheel.SpeedTuning tuning = new ShooterFlywheel.SpeedTuning(0.002, 0.002, false);
        flywheel.updateTuningRpm(2000, tuning);
        assertTrue(flywheel.getPower() > 0);

        now.set(300_000_000L);
        flywheel.updateTuningRpm(2000, tuning);
        assertEquals(0, flywheel.getPower(), 0);

        for (int ms = 400; ms <= 5000; ms += 100) {
            now.set(ms * 1_000_000L);
            flywheel.updateTuningRpm(2000, tuning);
        }
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertTrue(flywheel.getPower() > 0);
        assertFalse(flywheel.isAtSpeed());
    }

    @Test public void tuningReversalBrakesBeforeResumingRpmControl() {
        enable();
        ShooterFlywheel.SpeedTuning tuning = new ShooterFlywheel.SpeedTuning();
        rpm(1200, 0);
        flywheel.updateTuningRpm(1200, tuning);
        assertTrue(flywheel.getPower() > 0);

        clearInvocations(left);
        assertFalse(flywheel.prepareTuningDirection(true));
        InOrder commands = inOrder(left);
        commands.verify(left).setPower(0);
        commands.verify(left).setDirection(DcMotorSimple.Direction.REVERSE);

        now.set(500_000_000L);
        rpm(-1200, 0);
        assertFalse(flywheel.prepareTuningDirection(true));
        assertEquals(0, flywheel.getPower(), 0);

        rpm(0, 0);
        assertTrue(flywheel.prepareTuningDirection(true));
        flywheel.updateTuningRpm(1200, tuning);
        now.addAndGet(100_000_000L);
        flywheel.updateTuningRpm(1200, tuning);
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertTrue(flywheel.getPower() > 0);
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
        settings.kV = Double.NaN;
        flywheel.requestRpm(1000);
        flywheel.periodic();
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void integralDoesNotWindUpWhenFeedforwardAloneExceedsVoltageCap() {
        settings.kP = 0;
        settings.kI = .001;
        settings.kV = .003;
        rpm(1000, 1000);
        enable();
        flywheel.requestRpm(2000);
        flywheel.periodic();
        step(250); step(500); step(750); step(1000);
        assertTrue(flywheel.isOutputLimited());
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        assertEquals(0, flywheel.getFeedbackVolts(), 1e-9);
    }

    @Test public void integralAccumulatesInSecondsAndPanelsGainEditClearsItsHistory() {
        settings.kP = settings.kV = 0;
        settings.kI = .001;
        settings.maxAccelerationRpmPerSecond = 10000;
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(100);
        assertEquals(.1, flywheel.getFeedbackVolts(), 1e-9);
        step(200);
        assertEquals(.2, flywheel.getFeedbackVolts(), 1e-9);
        settings.kI = .002;
        flywheel.periodic();
        assertEquals(0, flywheel.getFeedbackVolts(), 1e-9);
    }

    @Test public void panelsVoltageCapEditTakesEffectOnNextLoop() {
        enable();
        flywheel.requestVoltage(3);
        flywheel.periodic();
        assertEquals(3, flywheel.getAppliedVolts(), 1e-9);
        settings.maxVoltage = 1;
        step(20);
        assertEquals(1, flywheel.getAppliedVolts(), 1e-9);
        assertEquals(1.0 / 12, flywheel.getPower(), 1e-9);
    }

    @Test public void fallingBelowSpeedAfterReadyRestartsTrackingTimeout() {
        settings.kP = settings.kV = 0;
        settings.spinupTimeoutMs = 300;
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
        settings.kP = settings.kV = 0;
        settings.maxAccelerationRpmPerSecond = 10000;
        settings.spinupTimeoutMs = 300;
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

    @Test public void unrelatedMotorCannotAffectThisShooter() {
        when(right.getVelocity()).thenThrow(new IllegalStateException("Unused encoder must not be read"));
        when(left.getVelocity()).thenReturn(1000.0 * 28 / 60);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(125); step(250); step(400);
        assertEquals(ShooterFlywheel.Mode.VELOCITY, flywheel.getMode());
        assertTrue(flywheel.isAtSpeed());
        assertEquals(1000, flywheel.getMeasuredRpm(), 1e-9);
        assertEquals(2, flywheel.getAppliedVolts(), 1e-9);
        verify(left, atLeastOnce()).setPower(2.0 / 12);
    }

    @Test public void directionTestUsesOwnEncoder() {
        rpm(500, 0);
        flywheel.enable();
        flywheel.requestMotorTest();
        flywheel.periodic();
        step(250); step(500); step(750); step(1000); step(1250);
        assertEquals(ShooterFlywheel.Mode.DIRECTION_TEST, flywheel.getMode());
        assertEquals(500, flywheel.getMeasuredRpm(), 1e-9);
        verify(left, atLeastOnce()).setPower(1.0 / 12);
        verify(right, never()).getVelocity();
    }

    @Test public void directionTestStopsWhenOwnEncoderDoesNotTurn() {
        rpm(0, 1000);
        flywheel.enable();
        flywheel.requestMotorTest();
        flywheel.periodic();
        step(250); step(500); step(750); step(1000);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
    }

    @Test public void ownEncoderStallStopsMotorDespiteUnrelatedMotorReading() {
        rpm(0, 1000);
        enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        step(250); step(500); step(750); step(1000);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        assertEquals(0, flywheel.getPower(), 0);
    }

    @Test public void editingMotorDirectionInvalidatesReadinessAndRequiresReinit() {
        rpm(1000, 1000);
        enable();
        flywheel.requestRpm(1000);
        flywheel.periodic();
        step(125); step(250);
        assertTrue(flywheel.isAtSpeed());
        settings.reversed = true;
        assertFalse(flywheel.isAtSpeed());
        assertFalse(flywheel.hasFreshSample());
        clearInvocations(left, right);
        step(270);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left).setPower(0);
        flywheel.idle(); flywheel.requestVoltage(1); step(290);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
    }

    @Test public void derivativeRespondsToMeasuredAccelerationWithoutSetpointKickOrReversePower() {
        settings.kP = settings.kV = 0;
        settings.kD = .001;
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
    }

    @Test public void panelsDirectionEditStopsUntilHardwareIsReinitialized() {
        enable();
        rpm(1000, 1000);
        flywheel.requestVoltage(1);
        flywheel.periodic();
        settings.reversed = true;
        clearInvocations(left, right);
        step(20);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void failedStopStillLatchesTerminalState() {
        enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        clearInvocations(left, right);
        doThrow(new IllegalStateException("disconnected")).when(left).setPower(0);
        assertThrows(IllegalStateException.class, flywheel::stop);
        doNothing().when(left).setPower(0);
        clearInvocations(left, right);
        flywheel.enable();
        flywheel.idle();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        verify(left, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void encoderReadExceptionStopsPreviouslyPoweredMotor() {
        enable();
        flywheel.requestVoltage(1);
        flywheel.periodic();
        clearInvocations(left, right);
        when(left.getVelocity()).thenThrow(new IllegalStateException("encoder disconnected"));
        assertThrows(IllegalStateException.class, flywheel::periodic);
        assertEquals(ShooterFlywheel.Mode.FAULT, flywheel.getMode());
        verify(left).setPower(0);
    }

    @Test public void encoderResolutionIsPerShooterAndCannotChangeWhileRunning() {
        ShooterFlywheel.Settings otherSettings = new ShooterFlywheel.Settings("launcherRight");
        otherSettings.ticksPerRevolution = 56;
        otherSettings.directionVerified = true;
        ShooterFlywheel other = new ShooterFlywheel(right, otherSettings, () -> batteryVolts, now::get);
        when(right.getVelocity()).thenReturn(560.0);
        other.enable(); other.requestVoltage(1); other.periodic();
        assertEquals(600, other.getMeasuredRpm(), 1e-9);
        otherSettings.ticksPerRevolution = 28;
        other.periodic();
        assertEquals(ShooterFlywheel.Mode.FAULT, other.getMode());
        assertEquals(0, other.getPower(), 0);
        assertEquals(28, settings.ticksPerRevolution, 0);
    }

    private void enable() {
        settings.directionVerified = true;
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
