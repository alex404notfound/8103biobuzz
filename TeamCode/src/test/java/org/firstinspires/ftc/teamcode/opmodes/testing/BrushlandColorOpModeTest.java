package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicLong;

import static org.mockito.Mockito.*;

public class BrushlandColorOpModeTest {
    private final LynxModule controlHub = mock(LynxModule.class);
    private final LynxModule expansionHub = mock(LynxModule.class);
    private final DigitalChannel pin0 = mock(DigitalChannel.class);
    private final DigitalChannel pin1 = mock(DigitalChannel.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private final AtomicLong now = new AtomicLong();
    private BrushlandColorTest op;

    @Before public void setUp() {
        when(controlHub.getBulkCachingMode()).thenReturn(LynxModule.BulkCachingMode.AUTO);
        when(expansionHub.getBulkCachingMode()).thenReturn(LynxModule.BulkCachingMode.OFF);
        BrushlandColorSensor sensor = new BrushlandColorSensor(pin0, pin1, 40, now::get);
        op = new BrushlandColorTest(sensor, Arrays.asList(controlHub, expansionHub), telemetry, now::get);
        op.gamepad1 = mock(Gamepad.class);
        op.init();
    }

    @Test public void everyInitAndRunCycleClearsEachHubOnceBeforeSinglePinReads() {
        verify(controlHub).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        verify(expansionHub).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        clearInvocations(controlHub, expansionHub, pin0, pin1);
        op.init_loop();
        op.start();
        op.loop();
        InOrder order = inOrder(controlHub, expansionHub, pin0, pin1);
        for (int i = 0; i < 2; i++) {
            order.verify(controlHub).clearBulkCache();
            order.verify(expansionHub).clearBulkCache();
            order.verify(pin0).getState();
            order.verify(pin1).getState();
        }
        order.verifyNoMoreInteractions();
    }

    @Test public void telemetryIsThrottledWhileSamplingContinuesEveryCycle() {
        clearInvocations(telemetry, pin0, pin1);
        op.init_loop();
        now.set(20_000_000L); op.init_loop();
        now.set(99_000_000L); op.init_loop();
        verify(telemetry, times(1)).update();
        now.set(100_000_000L); op.init_loop();
        verify(telemetry, times(2)).update();
        verify(pin0, times(4)).getState();
        verify(pin1, times(4)).getState();
    }

    @Test public void allianceCanChangeInInitAndLocksAtStart() {
        op.gamepad1.dpad_right = true;
        op.init_loop();
        verify(telemetry).addData("Alliance", Alliance.BLUE);
        op.gamepad1.dpad_right = false;
        op.gamepad1.dpad_left = true;
        now.set(100_000_000L); op.init_loop();
        verify(telemetry).addData("Alliance", Alliance.RED);
        op.start();
        clearInvocations(telemetry);
        op.gamepad1.dpad_left = false;
        op.gamepad1.dpad_right = true;
        op.loop();
        verify(telemetry).addData("Alliance", Alliance.RED);
        verify(telemetry, never()).addData("Alliance", Alliance.BLUE);
    }

    @Test public void stopRestoresEachPreviousCacheModeAndPreventsFurtherReads() {
        clearInvocations(controlHub, expansionHub, pin0, pin1);
        op.stop();
        verify(controlHub).setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        verify(expansionHub).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        clearInvocations(controlHub, expansionHub);
        op.stop(); op.start(); op.loop(); op.init_loop();
        verifyNoInteractions(controlHub, expansionHub, pin0, pin1);
    }
}
