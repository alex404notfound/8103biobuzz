package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.field.Canvas;
import com.bylazar.field.Circle;
import com.bylazar.field.Drawable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.Line;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

public class PanelsFieldTelemetryTest {
    private final List<Canvas> frames = new ArrayList<>();
    private final AtomicLong now = new AtomicLong();
    private final FieldMapTelemetry map = new FieldMapTelemetry(mock(Telemetry.class), frames::add, now::get);

    @Test public void nativePanelsCoordinatesAndHeadingMatchAllPedroFieldCorners() {
        double[][] corners = {{0, 0}, {144, 0}, {0, 144}, {144, 144}, {72, 72}};
        for (double[] corner : corners) {
            map.resetTrail();
            map.update(new Pose(corner[0], corner[1], 0), true, "READY");
            Canvas canvas = latest();
            Circle robot = (Circle) canvas.getItems().get(0);
            assertEquals(corner[0], robot.getX(), 1e-8);
            assertEquals(corner[1], robot.getY(), 1e-8);
            assertArrayEquals(new double[]{corner[0], 144 - corner[1]},
                    screenPoint(canvas.getPreset(), robot.getX(), robot.getY()), 1e-8);
            Line heading = (Line) canvas.getItems().get(1);
            assertEquals(corner[0] + 10, heading.getX2(), 1e-8);
            assertEquals(corner[1], heading.getY2(), 1e-8);
        }
        double[][] tips = {{82, 72}, {72, 62}, {62, 72}, {72, 82}};
        for (int i = 0; i < tips.length; i++) {
            map.resetTrail();
            map.update(new Pose(72, 72, i * Math.PI / 2), true, "READY");
            Line heading = (Line) latest().getItems().get(1);
            assertArrayEquals(tips[i], screenPoint(latest().getPreset(), heading.getX2(), heading.getY2()), 1e-8);
        }
    }

    @Test public void routeAndShooterCalibrationMarkersShareTheRobotFrame() {
        map.setAdditionalOverlay(canvas -> {
            Style routeStyle = new Style("transparent", "#a78bfa", .2);
            canvas.getItems().add(new Line(20, 40, 30, 50, routeStyle));
            canvas.getItems().add(new Line(30, 50, 40, 60, routeStyle));
            canvas.getItems().add(new Circle(55, 65, 1.5, new Style("#fbbf24", "transparent", 0)));
        });
        map.update(new Pose(10, 20, 0), true, "READY");
        List<Drawable> items = latest().getItems();
        Line route = (Line) items.get(2);
        assertEquals(20, route.getX1(), 1e-8);
        assertEquals(40, route.getY1(), 1e-8);
        assertEquals("#a78bfa", route.getStyle().getOutlineFill());
        Circle savedShot = (Circle) items.get(4);
        assertEquals(55, savedShot.getX(), 1e-8);
        assertEquals(65, savedShot.getY(), 1e-8);
        assertEquals("#fbbf24", savedShot.getStyle().getFill());
    }

    @Test public void nativePublisherDoesNotThrottleImmediateFaultOrRecoveryFrames() {
        FieldManager field = new FieldManager();
        List<Integer> publishedSizes = new ArrayList<>();
        field.setSendCanvas(canvas -> { publishedSizes.add(canvas.getItems().size()); return kotlin.Unit.INSTANCE; });
        FieldMapTelemetry live = new FieldMapTelemetry(
                mock(Telemetry.class),
                canvas -> PanelsFieldTelemetry.send(canvas, field), now::get);
        live.update(new Pose(10, 20, 0), true, "READY");
        field.setLastUpdate(System.currentTimeMillis() + 60_000);
        now.incrementAndGet();
        live.update(null, false, "FAULT");
        field.setLastUpdate(System.currentTimeMillis() + 60_000);
        now.incrementAndGet();
        live.update(new Pose(100, 120, 0), true, "READY");
        assertEquals(java.util.Arrays.asList(2, 0, 2), publishedSizes);
    }

    @Test public void nativePublisherKeepsACompleteSnapshotForLateClientsAndCapture() {
        FieldManager field = new FieldManager();
        java.util.UUID background = java.util.UUID.randomUUID();
        field.setDefaultBgID(background);
        List<Canvas> published = new ArrayList<>();
        field.setSendCanvas(canvas -> {
            published.add(new Canvas(canvas.getPreset(), new ArrayList<>(canvas.getItems()), canvas.getBgID()));
            return kotlin.Unit.INSTANCE;
        });
        map.update(new Pose(20, 30, 0), true, "READY");
        PanelsFieldTelemetry.send(latest(), field);
        assertEquals(1, published.size());
        assertEquals(2, published.get(0).getItems().size());
        assertEquals(background, published.get(0).getBgID());
        assertEquals(2, field.getLastCanvas().getItems().size());
        // The native manager clears the working buffer after publishing; the saved frame must survive.
        assertTrue(field.getCanvas().getItems().isEmpty());
        assertEquals(2, latest().getItems().size());
        now.incrementAndGet();
        map.update(null, false, "FAULT");
        PanelsFieldTelemetry.send(latest(), field);
        assertTrue(field.getLastCanvas().getItems().isEmpty());
    }

    private Canvas latest() {
        return frames.get(frames.size() - 1);
    }

    /** Same field-inch transform as the pinned Panels 1.0.7 browser renderer. */
    private double[] screenPoint(FieldPresetParams preset, double x, double y) {
        x = (x + preset.getOffsetX()) * (preset.getFlipX() ? -1 : 1);
        y = (y + preset.getOffsetY()) * (preset.getFlipY() ? -1 : 1);
        if (preset.getReverseXY()) { double swap = x; x = y; y = swap; }
        double sx = y, sy = -x;
        double theta = preset.getRotation().ordinal() * Math.PI / 2;
        return new double[]{72 + sx * Math.cos(theta) - sy * Math.sin(theta),
                72 + sx * Math.sin(theta) + sy * Math.cos(theta)};
    }
}
