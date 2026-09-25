package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.concurrent.atomic.AtomicLong;

import static org.firstinspires.ftc.teamcode.opmodes.testing.ShooterTuning.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class ShooterTuningTest {
    private final DcMotorEx motor = mock(DcMotorEx.class);
    private final DcMotorEx rightMotor = mock(DcMotorEx.class);
    private final Servo servo = mock(Servo.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private final Telemetry dashboardTelemetry = mock(Telemetry.class);
    private final AtomicLong now = new AtomicLong();
    private ShooterTuning op;
    private double power;
    private double rightPower;
    private Double position;

    @Before public void setUp() {
        defaults();
        doAnswer(call -> { power = call.getArgument(0); return null; }).when(motor).setPower(anyDouble());
        doAnswer(call -> { rightPower = call.getArgument(0); return null; }).when(rightMotor).setPower(anyDouble());
        doAnswer(call -> { position = call.getArgument(0); return null; }).when(servo).setPosition(anyDouble());
        op = new ShooterTuning(now::get, () -> dashboardTelemetry);
        op.hardwareMap = mock(HardwareMap.class);
        op.telemetry = telemetry;
        when(op.hardwareMap.get(DcMotorEx.class, "launcherLeft")).thenReturn(motor);
        when(op.hardwareMap.get(DcMotorEx.class, "launcherRight")).thenReturn(rightMotor);
        when(op.hardwareMap.get(Servo.class, "hood")).thenReturn(servo);
        when(op.hardwareMap.getAll(LynxModule.class)).thenReturn(Collections.emptyList());
        op.init();
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        leftMotorName = "launcherLeft"; rightMotorName = "launcherRight"; servoName = "hood";
        leftReversed = rightReversed = false;
        ticksPerRevolution = 28;
        runFlywheel = false; targetRpm = 1000; servoPosition = 0.5;
        kP = 0.0002; kI = kD = kS = kA = 0; kV = 1.0 / 6000;
        maxPower = 1.0; maxAccelerationRpmPerSecond = 3000; integralLimit = 1000;
    }

    @Test public void mapsOnlyTwoMotorsAndOneServoAndUsesSoftwareVelocityControl() {
        verify(op.hardwareMap).get(DcMotorEx.class, "launcherLeft");
        verify(op.hardwareMap).get(DcMotorEx.class, "launcherRight");
        verify(op.hardwareMap).get(Servo.class, "hood");
        verify(op.hardwareMap).getAll(LynxModule.class);
        verifyNoMoreInteractions(op.hardwareMap);
        verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(rightMotor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(rightMotor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Test public void bothMotorsSharePowerFromTheLeftEncoderAndStopTogether() {
        startRunning();
        kP = 0; kV = 0.0001; maxAccelerationRpmPerSecond = 100000;
        tick();
        assertEquals(0.1, power, 1e-9);
        assertEquals(0.1, rightPower, 1e-9);
        verify(rightMotor, never()).getVelocity();
        runFlywheel = false; tick();
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
        runFlywheel = true; tick();
        op.stop();
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
    }

    @Test public void initAndStartLeaveMotorOffAndServoWaitsUntilActiveLoop() {
        runFlywheel = true;
        op.init_loop(); op.loop();
        assertEquals(0, power, 0);
        assertNull(position);
        op.start();
        assertFalse(runFlywheel);
        tick();
        assertEquals(0, power, 0);
        assertEquals(0.5, position, 0);
    }

    @Test public void targetAndVelocityGainCanBeChangedLiveWithoutGamepad() {
        startRunning();
        kP = 0; kV = 0.0001; maxAccelerationRpmPerSecond = 100000;
        tick();
        assertEquals(0.1, power, 1e-9);
        targetRpm = 2000; kV = 0.0002;
        tick();
        assertEquals(0.4, power, 1e-9);
    }

    @Test public void encoderTicksAreConvertedToRpmBeforeFeedbackAndTelemetry() {
        startRunning();
        kV = 0; kP = 0.001; maxAccelerationRpmPerSecond = 100000;
        targetRpm = 800;
        when(motor.getVelocity()).thenReturn(280.0); // 10 rev/s = 600 RPM.
        tick();
        assertEquals(0.2, power, 1e-9);
        verify(telemetry).addData("shooter.measuredRpm", 600.0);
        verify(dashboardTelemetry).addData("shooter.measuredRpm", 600.0);
        ticksPerRevolution = 56;
        tick();
        assertEquals(0.5, power, 1e-9); // Now 300 RPM: 500 RPM error.
    }

    @Test public void accelerationFeedforwardOnlyAppliesDuringTheReferenceRamp() {
        startRunning();
        targetRpm = 100; kP = kV = 0; kA = 0.001;
        maxAccelerationRpmPerSecond = 100;
        tick();
        assertEquals(0.1, power, 1e-9);
        verify(telemetry).addData("shooter.referenceRpm", 2.0);
        for (int i = 0; i < 50; i++) tick();
        assertEquals(0, power, 1e-9);
    }

    @Test public void derivativeUsesMeasuredSpeedAndNotTheTargetStep() {
        startRunning();
        kP = kI = 0; kV = 0.0001; kD = 0.00001;
        maxAccelerationRpmPerSecond = 100000;
        tick();
        assertEquals(0.1, power, 1e-9);
        targetRpm = 2000;
        tick();
        assertEquals(0.2, power, 1e-9);
        when(motor.getVelocity()).thenReturn(28.0); // +60 RPM in 20 ms.
        tick();
        assertEquals(0.17, power, 1e-9);
    }

    @Test public void integralAccumulatesOverTimeAndResetsWhenDisabled() {
        startRunning();
        kP = kV = 0; kI = 0.001; maxAccelerationRpmPerSecond = 100000;
        tick(); tick();
        assertEquals(0.04, power, 1e-9);
        runFlywheel = false; tick();
        assertEquals(0, power, 0);
        runFlywheel = true; tick();
        assertEquals(0.02, power, 1e-9);
    }

    @Test public void powerIsLimitedWithoutIntegralWindupOrReverseDrive() {
        maxPower = 0.5;
        startRunning();
        kP = 0.001; kI = 0.01; kV = 0; maxAccelerationRpmPerSecond = 100000;
        for (int i = 0; i < 20; i++) tick();
        assertEquals(0.5, power, 0);
        when(motor.getVelocity()).thenReturn(1400.0 / 3); // 1000 RPM.
        tick();
        assertEquals(0, power, 1e-9);
        when(motor.getVelocity()).thenReturn(560.0); // Above target: zero output engages BRAKE.
        tick();
        assertEquals(0, power, 0);
    }

    @Test public void servoPositionIsLiveWhileFlywheelIsOffAndClampedToItsRange() {
        op.start();
        servoPosition = 0.72; tick();
        assertEquals(0.72, position, 0);
        servoPosition = 2; tick();
        assertEquals(1, position, 0);
        servoPosition = -1; tick();
        assertEquals(0, position, 0);
        assertEquals(0, power, 0);
    }

    @Test public void zeroTargetAndStopRemovePowerImmediately() {
        startRunning(); tick();
        assertTrue(power > 0);
        targetRpm = 0; tick();
        assertEquals(0, power, 0);
        targetRpm = 1000; tick();
        assertTrue(power > 0);
        op.stop();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
        clearInvocations(motor, rightMotor, servo);
        runFlywheel = true; op.start(); tick(); op.init_loop();
        verifyNoInteractions(motor, rightMotor, servo);
    }

    @Test public void invalidTuningOrFeedbackStopsAndClearsTheRunToggle() {
        startRunning(); tick();
        ticksPerRevolution = 0; tick();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
        ticksPerRevolution = 28; runFlywheel = true; kV = Double.NaN; tick();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
        kV = 0.0001; runFlywheel = true;
        when(motor.getVelocity()).thenReturn(Double.NaN);
        tick();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
    }

    @Test public void hardwareFailureStopsTheMotorBeforeRethrowing() {
        startRunning(); tick();
        when(motor.getVelocity()).thenThrow(new IllegalStateException("disconnected"));
        assertThrows(IllegalStateException.class, this::tick);
        assertEquals(0, power, 0);
        assertFalse(runFlywheel);
    }

    @Test public void staticFeedforwardContributesToBothMotorsWhileSpinning() {
        kP = 0; kS = 0.02; kV = 0.0001; maxAccelerationRpmPerSecond = 100000;
        startRunning(); tick();
        assertEquals(0.12, power, 1e-9);
        assertEquals(0.12, rightPower, 1e-9);
        targetRpm = 0; tick();
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
    }

    @Test public void proportionalCorrectionAddsToFeedforwardWithoutChangingIt() {
        targetRpm = 1000; maxAccelerationRpmPerSecond = 100000;
        kP = 0.0005; kV = 0.0004;
        when(motor.getVelocity()).thenReturn(800.0 * 28 / 60);
        startRunning(); tick();
        assertEquals(0.5, power, 1e-9); // 0.4 feedforward + 0.1 P correction.
        assertEquals(0.5, rightPower, 1e-9);
        verify(telemetry).addData("shooter.feedforwardPower", 0.4);

        kP = 0.001; tick();
        assertEquals(0.6, power, 1e-9); // Same 0.4 feedforward + 0.2 P correction.
        assertEquals(0.0004, kV, 0);
        verify(telemetry, times(2)).addData("shooter.feedforwardPower", 0.4);

        when(motor.getVelocity()).thenReturn(1000.0 * 28 / 60);
        tick();
        assertEquals(0.4, power, 1e-9); // P contributes zero at the target.
        when(motor.getVelocity()).thenReturn(1200.0 * 28 / 60);
        tick();
        assertEquals(0.2, power, 1e-9); // Above target: P subtracts 0.2.
    }

    @Test public void telemetrySeparatesProportionalPowerAndShowsOutputSaturation() {
        targetRpm = 2000; maxAccelerationRpmPerSecond = 100000;
        kP = 0.001; kV = 0.00069;
        when(motor.getVelocity()).thenReturn(2000.0 * 28 / 60);
        startRunning(); tick();
        assertEquals(1, power, 0);
        assertEquals(1, rightPower, 0);
        verify(telemetry).addData("shooter.pPower", 0.0);
        verify(telemetry).addData("shooter.iPower", 0.0);
        verify(telemetry).addData(eq("shooter.dPower"), doubleThat(value -> Math.abs(value) < 1e-9));
        verify(telemetry).addData("shooter.requestedPower", 1.38);
        verify(telemetry).addData("shooter.outputLimited", true);
    }

    @Test public void changingFeedforwardGainClearsOldIntegralCorrection() {
        targetRpm = 1000; maxAccelerationRpmPerSecond = 100000;
        kP = kV = 0; kI = 0.001;
        startRunning(); tick(); tick(); tick();
        assertEquals(0.06, power, 1e-9);
        kV = 0.0002; tick();
        assertEquals(0.22, power, 1e-9); // New 0.2 FF + only this loop's 0.02 I.
        assertEquals(0.22, rightPower, 1e-9);
    }

    @Test public void lowerRpmTargetImmediatelyReducesTheReferenceAndBrakes() {
        targetRpm = 3000; maxAccelerationRpmPerSecond = 100000;
        kP = 0.001; kV = 1.0 / 6000;
        when(motor.getVelocity()).thenReturn(1400.0); // 3000 RPM.
        startRunning(); tick();
        assertEquals(0.5, power, 1e-9);
        maxAccelerationRpmPerSecond = 100;
        targetRpm = 1000; tick();
        verify(telemetry).addData("shooter.referenceRpm", 1000.0);
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
        verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(rightMotor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        when(motor.getVelocity()).thenReturn(1000.0 * 28 / 60);
        tick();
        assertEquals(1.0 / 6, power, 1e-9); // Resume the new speed's feedforward.
    }

    @Test public void eitherDirectionEditRequiresReinitialization() {
        startRunning(); tick();
        rightReversed = true; tick();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
        rightReversed = false; leftReversed = true; runFlywheel = true; tick();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
    }

    @Test public void timingGapDisarmsBothMotorsUntilExplicitlyRestarted() {
        startRunning(); tick();
        now.addAndGet(300_000_000); tick();
        assertFalse(runFlywheel);
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
        tick();
        assertEquals(0, rightPower, 0);
        runFlywheel = true; tick();
        assertTrue(power > 0);
        assertEquals(power, rightPower, 0);
    }

    @Test public void stopStillZerosRightMotorIfLeftMotorWriteFails() {
        startRunning(); tick();
        doThrow(new IllegalStateException("left disconnected")).when(motor).setPower(0);
        assertThrows(IllegalStateException.class, op::stop);
        assertEquals(0, rightPower, 0);
        assertFalse(runFlywheel);
    }

    @Test public void servoFailureStopsBothMotorsAndClosesTheOpMode() {
        startRunning(); tick();
        doThrow(new IllegalStateException("servo disconnected")).when(servo).setPosition(anyDouble());
        assertThrows(IllegalStateException.class, this::tick);
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
        assertFalse(runFlywheel);
        clearInvocations(motor, rightMotor, servo);
        op.start(); tick(); op.init_loop();
        verifyNoInteractions(motor, rightMotor, servo);
    }

    @Test public void missingServoAtInitStillZerosBothMappedMotors() {
        when(op.hardwareMap.get(Servo.class, "hood")).thenThrow(new IllegalArgumentException("missing hood"));
        assertThrows(IllegalArgumentException.class, op::init);
        assertEquals(0, power, 0);
        assertEquals(0, rightPower, 0);
        assertFalse(runFlywheel);
    }

    private void startRunning() { op.start(); runFlywheel = true; }
    private void tick() { now.addAndGet(20_000_000); op.loop(); }
}
