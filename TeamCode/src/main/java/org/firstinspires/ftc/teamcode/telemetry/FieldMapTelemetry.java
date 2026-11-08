package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.field.Canvas;
import com.bylazar.field.Circle;
import com.bylazar.field.Line;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

/** Live Pinpoint/Pedro estimate and overlays on Panels' native BIOBUZZ field. */
public final class FieldMapTelemetry {
    private static final long INTERVAL_NANOS = 100_000_000L;
    private static final int TRAIL_POINTS = 100;
    private static final double ROBOT_RADIUS = 7;
    private static final Style ROBOT_STYLE = new Style("transparent", "#2dd4bf", .4);
    private static final Style TRAIL_STYLE = new Style("transparent", "#2dd4bf", .2);
    private final Telemetry telemetry;
    private final Consumer<Canvas> sendFrame;
    private final LongSupplier clock;
    private final Deque<Pose> trail = new ArrayDeque<>();
    private long lastSent;
    private boolean sent;
    private boolean lastReady;
    private Consumer<Canvas> additionalOverlay = canvas -> { };

    public FieldMapTelemetry(Telemetry telemetry, Consumer<Canvas> sendFrame, LongSupplier clock) {
        this.telemetry = telemetry;
        this.sendFrame = sendFrame;
        this.clock = clock;
    }

    /** Called after localization; rate limiting never changes how often sensors are sampled. */
    public void update(Pose pose, boolean localizationReady, String status) {
        boolean ready = localizationReady && Constants.isFinitePose(pose);
        boolean outside = ready && (pose.x() < 0 || pose.x() > FieldConstants.FIELD_WIDTH_INCHES
                || pose.y() < 0 || pose.y() > FieldConstants.FIELD_LENGTH_INCHES);
        // Panels' field format has no text primitive; send status beside the field in Telemetry.
        telemetry.addData("Field localization ready", ready);
        telemetry.addData("Field localization status", status);
        telemetry.addData("Field X (in)", ready ? pose.x() : "unavailable");
        telemetry.addData("Field Y (in)", ready ? pose.y() : "unavailable");
        telemetry.addData("Field heading (deg)", ready ? Math.toDegrees(pose.heading()) : "unavailable");
        telemetry.addData("Field map", !ready ? "POSITION UNAVAILABLE - check telemetry"
                : outside ? "POSE OUTSIDE FIELD - check starting pose"
                : "ODOMETRY ESTIMATE / heading + recent trail");

        if (!ready) trail.clear();
        long now = clock.getAsLong();
        // Publish faults/recovery immediately, even between the regular 10 Hz frames.
        if (sent && ready == lastReady && now - lastSent < INTERVAL_NANOS) return;
        lastSent = now;
        sent = true;
        lastReady = ready;

        Canvas canvas = PanelsFieldTelemetry.newCanvas();
        if (ready) {
            trail.addLast(pose);
            while (trail.size() > TRAIL_POINTS) trail.removeFirst();
            Pose previous = null;
            for (Pose point : trail) {
                if (previous != null) canvas.getItems().add(new Line(previous.x(), previous.y(),
                        point.x(), point.y(), TRAIL_STYLE));
                previous = point;
            }
            canvas.getItems().add(new Circle(pose.x(), pose.y(), ROBOT_RADIUS, ROBOT_STYLE));
            canvas.getItems().add(new Line(pose.x(), pose.y(),
                    pose.x() + (ROBOT_RADIUS + 3) * Math.cos(pose.heading()),
                    pose.y() + (ROBOT_RADIUS + 3) * Math.sin(pose.heading()), ROBOT_STYLE));
        }
        additionalOverlay.accept(canvas);
        sendFrame.accept(canvas);
    }

    /** The callback draws native Panels shapes in the same Pedro frame as the robot. */
    public void setAdditionalOverlay(Consumer<Canvas> overlay) {
        additionalOverlay = overlay == null ? canvas -> { } : overlay;
    }

    /** A pose reset starts a new trail, without a fictitious line across the field. */
    public void resetTrail() {
        trail.clear();
        sent = false;
    }
}
