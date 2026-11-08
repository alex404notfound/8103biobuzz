package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.field.Canvas;
import com.bylazar.field.Circle;
import com.bylazar.field.Line;
import com.bylazar.field.Style;
import com.google.gson.Gson;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class FieldMapTelemetryTest {
    private final AtomicLong now = new AtomicLong();
    private final List<Canvas> frames = new ArrayList<>();
    private final Telemetry telemetry = mock(Telemetry.class);
    private final FieldMapTelemetry map = new FieldMapTelemetry(telemetry, frames::add, now::get);

    @Test public void telemetryKeepsTheLatestPoseEvenBetweenFieldFrames() {
        map.update(new Pose(31, 42, Math.PI / 2), true, "READY");
        verify(telemetry).addData("Field X (in)", 31.0);
        verify(telemetry).addData("Field Y (in)", 42.0);
        verify(telemetry).addData("Field heading (deg)", 90.0);
        clearInvocations(telemetry);
        now.set(20_000_000);
        map.update(new Pose(32, 43, 0), true, "READY");
        assertEquals(1, frames.size());
        verify(telemetry).addData("Field X (in)", 32.0);
        verify(telemetry).addData("Field Y (in)", 43.0);
        verify(telemetry).addData("Field heading (deg)", 0.0);
    }

    @Test public void sendsAtTenHzWithBoundedHistoryAndClearsOnPoseReset() {
        map.update(Pose.zero(), true, "READY");
        now.set(99_999_999L);
        map.update(new Pose(1, 1, 0), true, "READY");
        assertEquals(1, frames.size());
        for (int i = 1; i <= 120; i++) {
            now.set(i * 100_000_000L);
            map.update(new Pose(i, 24, 0), true, "READY");
        }
        assertEquals(101, latest().getItems().size()); // 99 trail segments, robot, heading
        Line first = (Line) latest().getItems().get(0);
        Line last = (Line) latest().getItems().get(98);
        assertEquals(21, first.getX1(), 0);
        assertEquals(22, first.getX2(), 0);
        assertEquals(120, last.getX2(), 0);
        map.resetTrail();
        map.update(new Pose(10, 10, 0), true, "READY");
        assertEquals(2, latest().getItems().size()); // robot and heading, no reset jump
    }

    @Test public void faultClearsRobotAndTrailImmediatelyAndRecoveryDoesNotBridgeTheGap() {
        map.update(new Pose(20, 30, 0), true, "READY");
        now.set(100_000_000L);
        map.update(new Pose(21, 30, 0), true, "READY");
        assertEquals(3, latest().getItems().size());
        now.incrementAndGet();
        map.update(new Pose(21, 30, 0), false, "CALIBRATING");
        assertEquals(3, frames.size());
        assertTrue(latest().getItems().isEmpty());
        verify(telemetry).addData("Field X (in)", "unavailable");
        verify(telemetry).addData("Field localization status", "CALIBRATING");
        now.incrementAndGet();
        map.update(new Pose(45, 30, 0), true, "READY");
        assertEquals(4, frames.size());
        assertEquals(2, latest().getItems().size());
    }

    @Test public void nonFiniteOrMissingPoseNeverEntersTheNativeFrameOrTelemetry() {
        for (Pose pose : new Pose[]{null, new Pose(Double.NaN, 0, 0),
                new Pose(0, Double.POSITIVE_INFINITY, 0), new Pose(0, 0, Double.NaN)}) {
            map.resetTrail();
            clearInvocations(telemetry);
            map.update(pose, true, "READY");
            assertTrue(latest().getItems().isEmpty());
            verify(telemetry).addData("Field localization ready", false);
            verify(telemetry).addData("Field X (in)", "unavailable");
            verify(telemetry).addData("Field Y (in)", "unavailable");
            verify(telemetry).addData("Field heading (deg)", "unavailable");
            // Panels serializes these native frames as JSON.
            assertFalse(new Gson().toJson(latest()).contains("NaN"));
        }
    }

    @Test public void nativeBiobuzzFrameDoesNotClampAnOutOfBoundsEstimate() {
        map.update(new Pose(-10, 160, 0), true, "READY");
        assertEquals("BIOBUZZ / Pedro", latest().getPreset().getName());
        Circle robot = (Circle) latest().getItems().get(0);
        assertEquals(-10, robot.getX(), 0);
        assertEquals(160, robot.getY(), 0);
        verify(telemetry).addData("Field map", "POSE OUTSIDE FIELD - check starting pose");
    }

    @Test public void calibrationOverlaySharesRobotFrameAndCanBeRemoved() {
        map.setAdditionalOverlay(canvas -> canvas.getItems().add(
                new Circle(40, 50, 1.5, new Style("#fbbf24", "transparent", 0))));
        map.update(new Pose(20, 30, 0), true, "READY");
        assertEquals(3, latest().getItems().size());
        assertEquals(20, ((Circle) latest().getItems().get(0)).getX(), 0);
        assertEquals(40, ((Circle) latest().getItems().get(2)).getX(), 0);
        map.setAdditionalOverlay(null);
        now.set(100_000_000L);
        map.update(new Pose(20, 30, 0), true, "READY");
        assertEquals(3, latest().getItems().size()); // trail, robot, heading
        assertEquals(1, latest().getItems().stream().filter(Circle.class::isInstance).count());
    }

    private Canvas latest() { return frames.get(frames.size() - 1); }
}
