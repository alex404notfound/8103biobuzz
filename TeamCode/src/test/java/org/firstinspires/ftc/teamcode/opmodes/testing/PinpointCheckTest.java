package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.field.Canvas;
import com.bylazar.field.Circle;
import com.bylazar.field.Line;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class PinpointCheckTest {
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final HardwareMap hardware = mock(HardwareMap.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private final Gamepad gamepad = mock(Gamepad.class);
    private final AtomicLong now = new AtomicLong();
    private final List<Canvas> frames = new ArrayList<>();
    private final double savedX = PinpointCheck.startX, savedY = PinpointCheck.startY,
            savedHeading = PinpointCheck.startHeadingDegrees;
    private PinpointCheck op;

    @Before public void setUp() {
        PinpointCheck.startX = PinpointCheck.startY = 72;
        PinpointCheck.startHeadingDegrees = 0;
        when(hardware.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get())).thenReturn(pinpoint);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        when(pinpoint.getDeviceVersion()).thenReturn(1);
        when(pinpoint.getPosition()).thenReturn(pose(20, 30, 90));
        op = new PinpointCheck() {
            @Override protected Telemetry withBrowserTelemetry(Telemetry driver) { return driver; }
            @Override protected void publishField(Canvas canvas) { frames.add(canvas); }
            @Override protected long nowNs() { return now.get(); }
        };
        op.hardwareMap = hardware;
        op.telemetry = telemetry;
        op.gamepad1 = gamepad;
        op.init();
    }

    @After public void restoreSettings() {
        PinpointCheck.startX = savedX;
        PinpointCheck.startY = savedY;
        PinpointCheck.startHeadingDegrees = savedHeading;
    }

    @Test public void enabledDiagnosticReadsOnlyPinpointAndPublishesActualPoseAndHeading() {
        assertNull(PinpointCheck.class.getAnnotation(Disabled.class));
        assertEquals("Pinpoint Check", PinpointCheck.class.getAnnotation(TeleOp.class).name());
        verify(hardware).get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        verifyNoMoreInteractions(hardware);
        verify(pinpoint, never()).setPosition(any());
        verify(pinpoint, never()).resetPosAndIMU();
        verify(pinpoint, never()).recalibrateIMU();
        assertTrue(latest().getItems().isEmpty());

        op.init_loop();
        assertRobot(20, 30, 90);
        verify(telemetry).addData("pinpoint.xInches", 20.0);
        verify(telemetry).addData("pinpoint.yInches", 30.0);
        verify(telemetry).addData("pinpoint.headingDegrees", 90.0);
        verify(telemetry).addData("pinpoint.headingRadians", Math.PI / 2);
        verify(pinpoint, never()).getPitch(any());
        verify(pinpoint, never()).getRoll(any());
        op.start();
        now.set(100_000_000L);
        when(pinpoint.getPosition()).thenReturn(pose(25, 40, 180));
        op.loop();
        verify(pinpoint, times(2)).update();
        assertRobot(25, 40, 180);
    }

    @Test public void newerFirmwareShowsPitchAndRollAndFiltersInvalidTilt() {
        when(pinpoint.getDeviceVersion()).thenReturn(3);
        when(pinpoint.getPitch(AngleUnit.DEGREES)).thenReturn(12.5);
        when(pinpoint.getRoll(AngleUnit.DEGREES)).thenReturn(-3.0);
        op.init_loop();
        verify(telemetry).addData("pinpoint.pitchDegrees", 12.5);
        verify(telemetry).addData("pinpoint.rollDegrees", -3.0);
        when(pinpoint.getPitch(AngleUnit.DEGREES)).thenReturn(Double.NaN);
        when(pinpoint.getRoll(AngleUnit.DEGREES)).thenReturn(Double.POSITIVE_INFINITY);
        op.loop();
        verify(telemetry).addData("pinpoint.pitchDegrees", "unavailable");
        verify(telemetry).addData("pinpoint.rollDegrees", "unavailable");
    }

    @Test public void startingPoseRequiresInitButtonAndResetsTheTrailWithoutAJump() {
        PinpointCheck.startX = 55;
        PinpointCheck.startY = 65;
        PinpointCheck.startHeadingDegrees = 180;
        op.init_loop();
        verify(pinpoint, never()).setPosition(any());
        now.set(100_000_000L);
        when(pinpoint.getPosition()).thenReturn(pose(22, 30, 90));
        op.init_loop();
        assertEquals(3, latest().getItems().size());
        when(gamepad.crossWasPressed()).thenReturn(true, false);
        op.init_loop();
        ArgumentCaptor<Pose2D> applied = ArgumentCaptor.forClass(Pose2D.class);
        verify(pinpoint).setPosition(applied.capture());
        assertEquals(55, applied.getValue().getX(DistanceUnit.INCH), 1e-8);
        assertEquals(65, applied.getValue().getY(DistanceUnit.INCH), 1e-8);
        assertEquals(-180, applied.getValue().getHeading(AngleUnit.DEGREES), 1e-8);
        assertEquals(2, latest().getItems().size());
        assertRobot(55, 65, 180);
        op.init_loop();
        verify(pinpoint, times(1)).setPosition(any());
        clearInvocations(pinpoint, gamepad);
        when(gamepad.crossWasPressed()).thenReturn(true);
        when(gamepad.squareWasPressed()).thenReturn(true);
        op.start();
        op.loop();
        verifyNoInteractions(gamepad);
        verify(pinpoint, never()).setPosition(any());
        verify(pinpoint, never()).recalibrateIMU();
    }

    @Test public void calibrationHidesMarkerAndRefusesStartingPoseUntilSettledAndReady() {
        op.init_loop();
        when(gamepad.squareWasPressed()).thenReturn(true, false);
        op.init_loop();
        verify(pinpoint).recalibrateIMU();
        assertTrue(latest().getItems().isEmpty());
        when(gamepad.crossWasPressed()).thenReturn(true);
        now.set(299_999_999L);
        op.init_loop();
        verify(pinpoint, never()).setPosition(any());
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.CALIBRATING);
        now.set(300_000_000L);
        op.init_loop();
        verify(pinpoint, never()).setPosition(any());
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        op.init_loop();
        verify(pinpoint).setPosition(any());
        assertRobot(72, 72, 0);
    }

    @Test public void invalidStartingPoseNeverWritesToTheComputer() {
        when(gamepad.crossWasPressed()).thenReturn(true);
        for (double x : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1, 145}) {
            PinpointCheck.startX = x;
            op.init_loop();
        }
        PinpointCheck.startX = 72;
        PinpointCheck.startY = 145;
        op.init_loop();
        PinpointCheck.startY = 72;
        PinpointCheck.startHeadingDegrees = Double.NaN;
        op.init_loop();
        verify(pinpoint, never()).setPosition(any());
    }

    @Test public void faultsAndInvalidSamplesClearTheMarkerAndRecoveryStartsANewTrail() {
        op.init_loop();
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED);
        now.incrementAndGet();
        op.loop();
        assertTrue(latest().getItems().isEmpty());
        verify(telemetry).addData("pinpoint.xInches", "unavailable");
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        for (Pose2D invalid : new Pose2D[]{null, pose(Double.NaN, 30, 0), pose(20, 30, Double.NaN)}) {
            when(pinpoint.getPosition()).thenReturn(invalid);
            now.addAndGet(100_000_000L);
            op.loop();
            assertTrue(latest().getItems().isEmpty());
        }
        when(pinpoint.getPosition()).thenReturn(pose(50, 60, 0));
        now.incrementAndGet();
        op.loop();
        assertEquals(2, latest().getItems().size());
        assertRobot(50, 60, 0);
    }

    @Test public void stopClearsPanelsAndPreventsFurtherSensorReads() {
        op.init_loop();
        op.stop();
        assertTrue(latest().getItems().isEmpty());
        clearInvocations(pinpoint);
        op.start();
        op.init_loop();
        op.loop();
        op.stop();
        verifyNoInteractions(pinpoint);
    }

    @Test public void readFailureClearsTheLastRobotAndStopsPolling() {
        op.init_loop();
        doThrow(new IllegalStateException("I2C disconnected")).when(pinpoint).update();
        assertThrows(IllegalStateException.class, op::loop);
        assertTrue(latest().getItems().isEmpty());
        clearInvocations(pinpoint);
        op.init_loop();
        op.loop();
        verifyNoInteractions(pinpoint);
    }

    private void assertRobot(double x, double y, double degrees) {
        Circle robot = (Circle) latest().getItems().stream().filter(Circle.class::isInstance).findFirst().get();
        assertEquals(x, robot.getX(), 1e-8);
        assertEquals(y, robot.getY(), 1e-8);
        Line heading = (Line) latest().getItems().get(latest().getItems().size() - 1);
        assertEquals(x + 10 * Math.cos(Math.toRadians(degrees)), heading.getX2(), 1e-8);
        assertEquals(y + 10 * Math.sin(Math.toRadians(degrees)), heading.getY2(), 1e-8);
    }

    private Canvas latest() { return frames.get(frames.size() - 1); }
    private static Pose2D pose(double x, double y, double degrees) {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, degrees);
    }
}
