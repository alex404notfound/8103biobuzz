package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TuningSafety;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.lang.reflect.Field;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

/** Runs the real AutoTune final lifecycle, Pedro Mecanum and PinpointLocalizer with fake devices. */
public class SafeTuningOpModeTest {
    private final HardwareMap map = mock(HardwareMap.class);
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final double[] powers = new double[4];
    private final DcMotor.ZeroPowerBehavior[] braking = new DcMotor.ZeroPowerBehavior[4];
    private double largestPower;
    private GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.READY;
    private double x, y, heading, vx, omega;
    private double xOffset, yOffset;
    private double savedX, savedY;
    private boolean savedTuned;
    private boolean zeroed;
    private int restorations;
    private double powerWhenRestored = -1;
    private Runnable afterPower = () -> { };
    private Runnable onSample = () -> { };
    private Runnable onZeroOffsets = () -> { };
    private Runnable onTelemetry = () -> { };
    private boolean readySeen;
    private boolean turnSeen;

    @Before public void setup() {
        savedX = Constants.localizerConfig.xPodOffset.get();
        savedY = Constants.localizerConfig.yPodOffset.get();
        savedTuned = Constants.foresightTuned;
        Constants.localizerConfig.xPodOffset.set(4.0);
        Constants.localizerConfig.yPodOffset.set(-6.0);
        Constants.foresightTuned = false;
        String[] names = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (int i = 0; i < names.length; i++) {
            final int index = i;
            DcMotorEx motor = mock(DcMotorEx.class);
            when(map.get(DcMotorEx.class, names[i])).thenReturn(motor);
            doAnswer(call -> {
                powers[index] = call.getArgument(0);
                largestPower = Math.max(largestPower, Math.abs(powers[index]));
                if (powers[index] != 0) afterPower.run();
                return null;
            }).when(motor).setPower(anyDouble());
            when(motor.getPower()).thenAnswer(call -> powers[index]);
            doAnswer(call -> { braking[index] = call.getArgument(0); return null; })
                    .when(motor).setZeroPowerBehavior(any());
        }
        when(map.get(GoBildaPinpointDriver.class, "pinpoint")).thenReturn(pinpoint);
        when(pinpoint.getDeviceStatus()).thenAnswer(call -> status);
        when(pinpoint.getPosX(DistanceUnit.INCH)).thenAnswer(call -> x);
        when(pinpoint.getPosY(DistanceUnit.INCH)).thenAnswer(call -> y);
        when(pinpoint.getHeading(AngleUnit.RADIANS)).thenAnswer(call -> heading);
        when(pinpoint.getVelX(DistanceUnit.INCH)).thenAnswer(call -> vx);
        when(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)).thenAnswer(call -> omega);
        when(pinpoint.getXOffset(DistanceUnit.INCH)).thenAnswer(call -> (float) xOffset);
        when(pinpoint.getYOffset(DistanceUnit.INCH)).thenAnswer(call -> (float) yOffset);
        doAnswer(call -> { onSample.run(); return null; }).when(pinpoint).update();
        doAnswer(call -> {
            Pose2D pose = call.getArgument(0);
            x = pose.getX(DistanceUnit.INCH); y = pose.getY(DistanceUnit.INCH);
            heading = pose.getHeading(AngleUnit.RADIANS);
            return null;
        }).when(pinpoint).setPosition(any(Pose2D.class));
        doAnswer(call -> {
            xOffset = call.getArgument(0); yOffset = call.getArgument(1);
            if (xOffset == 0 && yOffset == 0) {
                zeroed = true;
                onZeroOffsets.run();
            } else if (zeroed) {
                restorations++;
                powerWhenRestored = maximumPower();
                zeroed = false;
            }
            return null;
        }).when(pinpoint).setOffsets(anyDouble(), anyDouble(), eq(DistanceUnit.INCH));
    }

    @After public void restoreConfig() {
        Constants.localizerConfig.xPodOffset.set(savedX);
        Constants.localizerConfig.yPodOffset.set(savedY);
        Constants.foresightTuned = savedTuned;
        Thread.interrupted();
    }

    @Test(timeout = 5000) public void firstMotorCommandRequiresAFreshHealthySample() throws Exception {
        Probe phase = new Probe(() -> status = GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ, true);
        expectAbort(phase);
        assertEquals(0, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void actualForesightInitialDriveStopsOnFaultBeforeContinuing() throws Exception {
        afterPower = () -> status = GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
        expectAbort(new ForwardDeceleration(30));
        assertEquals("Fixture must exercise full real Mecanum output", 1, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void actualForesightPoweredLoopHonorsHeldB() throws Exception {
        ForwardVelocity phase = new ForwardVelocity(48);
        configure(phase);
        afterPower = () -> phase.gamepad1.b = true;
        expectConfiguredAbort(phase);
        assertEquals(1, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void actualWheelIdentifierStopsItsSingleMotorOnB() throws Exception {
        WheelIdentification phase = new WheelIdentification(2, "back left", "backLeft");
        configure(phase);
        afterPower = () -> phase.gamepad1.b = true;
        expectConfiguredAbort(phase);
        assertEquals(0.25, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void nonfiniteVelocityStopsAnAlreadyPoweredPhase() throws Exception {
        expectAbort(new Probe(() -> vx = Double.NaN, false));
        assertEquals(1, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void nonfinitePoseStopsAnAlreadyPoweredPhase() throws Exception {
        expectAbort(new Probe(() -> x = Double.NaN, false));
        assertEquals(1, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void sdkThreadInterruptionStopsBeforeAnotherMotorWrite() throws Exception {
        expectAbort(new Probe(() -> Thread.currentThread().interrupt(), false));
        Thread.interrupted();
        assertEquals(1, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void driverStationStopFlagAbortsTheRealPhase() throws Exception {
        SafeTuningOpMode<Void> phase = new SafeTuningOpMode<Void>("probe", "probe", true) {
            @Override protected Void runSafely() {
                drivetrain().drive(new DrivePowers(1, 0, 0), true);
                setSdkFlag(this, "stopRequested", true);
                localizer().update();
                fail("STOP must prevent further procedure actions");
                return null;
            }
        };
        expectAbort(phase);
        assertEquals(1, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void constructionAndArmingWaitForCalibrationWithZeroMotorPower() throws Exception {
        status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
        onTelemetry = () -> {
            assertEquals(0, maximumPower(), 0);
            status = GoBildaPinpointDriver.DeviceStatus.READY;
        };
        SafeTuningOpMode<Void> phase = new SafeTuningOpMode<Void>("probe", "probe", true) {
            @Override protected Void runSafely() { return null; }
        };
        run(phase);
        assertEquals(0, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void completionForcesZeroEvenBelowPedrosCachedPowerThreshold() throws Exception {
        SafeTuningOpMode<Void> phase = new SafeTuningOpMode<Void>("probe", "probe", true) {
            @Override protected Void runSafely() { spinMotor(0, 0.001); return null; }
        };
        run(phase);
        assertEquals(0.001, largestPower, 0);
        assertStopped();
    }

    @Test(timeout = 5000) public void offsetFaultRestoresOriginalHardwareOffsetsExactlyOnce() throws Exception {
        onZeroOffsets = () -> status = GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
        expectAbort(new PinpointOffsets());
        assertOffsetsRestored();
        assertStopped();
    }

    @Test(timeout = 5000) public void partiallyFailedZeroOffsetWriteStillRestoresOriginalOffsets() throws Exception {
        onZeroOffsets = () -> { throw new IllegalStateException("I2C write failed"); };
        try { run(new PinpointOffsets()); fail("Expected I2C failure"); }
        catch (IllegalStateException expected) { assertEquals("I2C write failed", expected.getMessage()); }
        assertOffsetsRestored();
        assertStopped();
    }

    @Test(timeout = 5000) public void completedHalfTurnRestoresOffsetsBeforeAutoTuneReceivesResult() throws Exception {
        PinpointOffsets phase = new PinpointOffsets();
        configure(phase);
        final int[] offsetSamples = {0};
        onSample = () -> {
            if (zeroed && ++offsetSamples[0] >= 4) {
                x = 12; y = -8; heading = Math.PI;
            }
        };
        onTelemetry = () -> {
            if (turnSeen) phase.gamepad1.a = offsetSamples[0] >= 4;
        };
        phase.start();
        phase.runOpMode();
        assertOffsetsRestored();
        assertStopped();
    }

    @Test(timeout = 5000) public void pathTestCannotUseUnmeasuredForesightScaffold() throws Exception {
        expectAbort(new PathTest(Tests.Test.LINE, 24));
        assertEquals(0, largestPower, 0);
        assertStopped();
    }

    private final class Probe extends SafeTuningOpMode<Void> {
        private final Runnable fault;
        private final boolean beforeMotion;
        Probe(Runnable fault, boolean beforeMotion) { super("probe", "probe", true); this.fault = fault; this.beforeMotion = beforeMotion; }
        @Override protected Void runSafely() {
            if (beforeMotion) fault.run();
            drivetrain().drive(new DrivePowers(1, 0, 0), true);
            if (!beforeMotion) fault.run();
            localizer().update();
            fail("Fault must prevent remaining procedure actions");
            return null;
        }
    }

    private void configure(SafeTuningOpMode<?> phase) {
        // SDK 12's LinearOpMode.start() is a no-op; OpModeManagerImpl owns this flag.
        setSdkFlag(phase, "isStarted", true);
        phase.hardwareMap = map;
        phase.gamepad1 = mock(Gamepad.class);
        phase.gamepad2 = mock(Gamepad.class);
        phase.telemetry = mock(Telemetry.class);
        doAnswer(call -> { if ("READY".equals(call.getArgument(1))) readySeen = true; return null; })
                .when(phase.telemetry).addData(eq("Pinpoint"), any(Object.class));
        doAnswer(call -> { turnSeen = true; return null; })
                .when(phase.telemetry).addData(eq("Turn (degrees)"), any(Object.class));
        doAnswer(call -> {
            if (readySeen && !turnSeen) phase.gamepad1.a = true;
            onTelemetry.run();
            return true;
        }).when(phase.telemetry).update();
    }

    private void run(SafeTuningOpMode<?> phase) throws Exception { configure(phase); phase.start(); phase.runOpMode(); }
    private void expectAbort(SafeTuningOpMode<?> phase) throws Exception { configure(phase); expectConfiguredAbort(phase); }
    private void expectConfiguredAbort(SafeTuningOpMode<?> phase) throws Exception {
        phase.start();
        try { phase.runOpMode(); fail("Expected tuning abort"); }
        catch (TuningSafety.Aborted expected) { }
    }
    private double maximumPower() {
        double maximum = 0;
        for (double power : powers) maximum = Math.max(maximum, Math.abs(power));
        return maximum;
    }
    private void assertStopped() {
        for (int i = 0; i < powers.length; i++) {
            assertEquals("Motor " + i, 0, powers[i], 0);
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, braking[i]);
        }
    }
    private void assertOffsetsRestored() {
        assertEquals(4, xOffset, 0);
        assertEquals(-6, yOffset, 0);
        assertEquals(1, restorations);
        assertEquals(0, powerWhenRestored, 0);
    }

    private static void setSdkFlag(SafeTuningOpMode<?> phase, String name, boolean value) {
        try {
            Field flag = Class.forName("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal").getDeclaredField(name);
            flag.setAccessible(true);
            flag.setBoolean(phase, value);
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError("Cannot simulate the SDK OpModeManager lifecycle", failure);
        }
    }
}
