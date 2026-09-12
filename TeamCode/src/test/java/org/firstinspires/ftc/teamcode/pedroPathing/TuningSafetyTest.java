package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.telemetry.Selector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.lang.reflect.Field;
import java.util.Collections;
import java.util.function.Supplier;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class TuningSafetyTest {
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final DcMotorEx motor = mock(DcMotorEx.class);
    private final HardwareMap map = mock(HardwareMap.class);
    private Follower real;
    private double power;

    @SuppressWarnings("unchecked")
    @Before public void setUp() throws Exception {
        when(map.get(eq(DcMotorEx.class), anyString())).thenReturn(motor);
        when(map.get(eq(GoBildaPinpointDriver.class), anyString())).thenReturn(pinpoint);
        when(motor.getMotorType()).thenReturn(new MotorConfigurationType());
        doAnswer(call -> { power = call.getArgument(0); return null; }).when(motor).setPower(anyDouble());
        when(motor.getPower()).thenAnswer(call -> power);
        HardwareMap.DeviceMapping<VoltageSensor> voltage = mock(HardwareMap.DeviceMapping.class);
        VoltageSensor battery = mock(VoltageSensor.class);
        when(battery.getVoltage()).thenReturn(12.0);
        when(voltage.iterator()).thenReturn(Collections.singletonList(battery).iterator());
        Field field = HardwareMap.class.getField("voltageSensor");
        field.setAccessible(true);
        field.set(map, voltage);
        when(pinpoint.getPosition()).thenReturn(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        real = Constants.createGuardedFollower(map);
        Tuning.follower = real;
        Tuning.telemetryM = new TelemetryManager(com.bylazar.telemetry.TelemetryPluginConfig::new,
                lines -> kotlin.Unit.INSTANCE, interval -> kotlin.Unit.INSTANCE);
    }

    @After public void tearDown() { real.breakFollowing(); Tuning.follower = null; }

    @Test public void selectedOffsetTunerWaitsForCalibrationInsteadOfThrowingFromInit() throws Exception {
        Tuning tuning = new Tuning() { @Override public void onSelect() { } };
        tuning.hardwareMap = map;
        tuning.telemetry = mock(Telemetry.class);
        tuning.gamepad1 = mock(Gamepad.class);
        tuning.gamepad2 = mock(Gamepad.class);
        Field field = SelectableOpMode.class.getDeclaredField("selector");
        field.setAccessible(true);
        @SuppressWarnings("unchecked") Selector<Supplier<OpMode>> selector =
                (Selector<Supplier<OpMode>>) field.get(tuning);
        selector.select(); // Localization folder.
        for (int i = 0; i < 4; i++) selector.incrementSelected();
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.CALIBRATING);
        try { selector.select(); tuning.init_loop(); }
        catch (Constants.LocalizationNotReady failure) { fail("INIT must wait for calibration without terminating the tuning menu"); }
        assertEquals(0, power, 0);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        tuning.init_loop();
        tuning.start();
        tuning.loop();
        tuning.stop();
        verify(pinpoint, atLeastOnce()).setOffsets(0, 0, DistanceUnit.INCH);
    }

    @Test public void startWithoutInitializedHealthyChildNeverRunsMotion() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init();
        safe.start();
        safe.loop();
        assertEquals(0, child.starts);
        assertEquals(0, power, 0);
    }

    @Test public void faultAtStartRejectsPreviouslyReadyTuner() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init(); safe.init_loop();
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        safe.start();
        assertEquals(0, child.starts);
        assertEquals(1, child.stops);
        assertEquals(0, power, 0);
    }

    @Test public void badReadStopsFullPowerBeforeChildCleanupAndRecoveryNeverResumes() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init(); safe.init_loop(); safe.start();
        assertTrue("Fixture must have real Pedro motor output before the fault", power > 0.5);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        safe.loop();
        assertEquals(0, power, 0);
        assertEquals("Stop motors before child cleanup may perform I2C writes", 0, child.powerAtStop, 0);
        assertEquals(0, child.completedLoops);
        assertEquals(1, child.stops);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        safe.loop(); safe.start(); safe.stop();
        assertEquals(1, child.starts);
        assertEquals(0, child.completedLoops);
        assertEquals(1, child.stops);
        assertEquals(0, power, 0);
    }

    @Test public void faultInsideAChildUpdatePreventsRemainingChildActions() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init(); safe.init_loop(); safe.start();
        AtomicInteger reads = new AtomicInteger();
        doAnswer(call -> {
            if (reads.incrementAndGet() == 2)
                when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
            return null;
        }).when(pinpoint).update();
        safe.loop();
        assertEquals(2, reads.get());
        assertEquals(0, child.completedLoops);
        assertEquals(1, child.stops);
        assertEquals(0, power, 0);
    }

    @Test public void nonfiniteVelocityAlsoStopsTheRunningTuner() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init(); safe.init_loop(); safe.start();
        when(pinpoint.getVelX(DistanceUnit.INCH)).thenReturn(Double.NaN);
        safe.loop();
        assertEquals(0, child.completedLoops);
        assertEquals(1, child.stops);
        assertEquals(0, power, 0);
    }

    @Test public void explicitCalibrationWaitsForMinimumTimeAndNewReadySample() {
        AtomicLong now = new AtomicLong();
        AtomicInteger initialized = new AtomicInteger();
        TuningSafety safe = new TuningSafety(() -> { initialized.incrementAndGet(); return new Probe(); }, real, now::get);
        configure(safe);
        safe.init(); safe.init_loop();
        assertEquals(1, initialized.get());
        when(safe.gamepad1.xWasPressed()).thenReturn(true, false);
        safe.init_loop();
        verify(pinpoint).recalibrateIMU();
        assertEquals(1, initialized.get());
        now.set(299_000_000L); safe.init_loop();
        assertEquals(1, initialized.get());
        now.set(300_000_000L);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.CALIBRATING);
        safe.init_loop();
        assertEquals(1, initialized.get());
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        safe.init_loop();
        assertEquals(2, initialized.get());
        safe.stop();
    }

    @Test public void faultRestoresOffsetTunersOriginalDeviceOffsets() {
        when(pinpoint.getXOffset(DistanceUnit.INCH)).thenReturn(4.0f);
        when(pinpoint.getYOffset(DistanceUnit.INCH)).thenReturn(-6.0f);
        TuningSafety safe = safety(OffsetsTuner::new);
        safe.init(); safe.init_loop(); safe.start();
        verify(pinpoint).setOffsets(0, 0, DistanceUnit.INCH);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        safe.loop();
        verify(pinpoint).setOffsets(4, -6, DistanceUnit.INCH);
        assertEquals(0, power, 0);
        safe.stop();
        verify(pinpoint, times(1)).setOffsets(4, -6, DistanceUnit.INCH);
    }

    @Test public void emergencyButtonStopsBeforeTheNestedOpModeUsesSdkStopServices() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init(); safe.init_loop(); safe.start();
        when(safe.gamepad1.bWasPressed()).thenReturn(true);
        safe.loop();
        assertEquals(0, child.completedLoops);
        assertEquals(1, child.stops);
        assertEquals(0, power, 0);
    }

    @Test public void heldStopButtonIsHonoredWithoutANewPressEdge() {
        Probe child = new Probe();
        TuningSafety safe = safety(() -> child);
        safe.init(); safe.init_loop(); safe.start();
        safe.gamepad1.b = true;
        safe.loop();
        assertEquals(0, child.completedLoops);
        assertEquals(1, child.stops);
        assertEquals(0, power, 0);
    }

    private TuningSafety safety(Supplier<OpMode> factory) {
        TuningSafety safe = new TuningSafety(factory, real);
        configure(safe);
        return safe;
    }

    private void configure(OpMode opMode) {
        opMode.hardwareMap = map;
        opMode.telemetry = mock(Telemetry.class);
        opMode.gamepad1 = mock(Gamepad.class);
        opMode.gamepad2 = mock(Gamepad.class);
    }

    private class Probe extends OpMode {
        int starts, completedLoops, stops;
        double powerAtStop;
        @Override public void init() { }
        @Override public void start() {
            starts++;
            real.startTeleopDrive(true);
            real.setTeleOpDrive(1, 0, 0, true);
            real.update();
        }
        @Override public void loop() { real.update(); completedLoops++; }
        @Override public void stop() { powerAtStop = power; stops++; }
    }

}
