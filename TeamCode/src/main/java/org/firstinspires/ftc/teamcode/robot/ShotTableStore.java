package org.firstinspires.ftc.teamcode.robot;

import android.util.AtomicFile;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.math.ShotTable;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.Launcher.Ball;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/** Versioned on-hub calibration. A failed load never silently replaces measured data with an empty file. */
public final class ShotTableStore {
    public static final String FILE_NAME = "shooter-distance-tables-v1.json";
    private static final String DISTANCE_REFERENCE = "ROBOT_CENTER_TO_MAPPED_CELL_INCHES";
    private static final int MAX_BYTES = 100_000;
    public interface Storage {
        /** null means no calibration file has been saved yet; other read failures must throw. */
        String read() throws IOException;
        void write(String content) throws IOException;
    }
    private final Storage storage;

    public ShotTableStore(Storage storage) { this.storage = storage; }
    public ShotTableStore(File file) { this(new AndroidStorage(file)); }

    public static Map<Ball, ShotTable> empty() {
        Map<Ball, ShotTable> result = new EnumMap<>(Ball.class);
        for (Ball ball : Ball.values()) result.put(ball, new ShotTable());
        return result;
    }

    public Map<Ball, ShotTable> load() throws IOException {
        String content = storage.read();
        if (content == null) return empty();
        if (content.getBytes(StandardCharsets.UTF_8).length > MAX_BYTES) throw new IOException("Calibration file too large");
        try {
            JSONObject root = new JSONObject(content);
            if (root.getDouble("version") != 1 || !DISTANCE_REFERENCE.equals(root.getString("distanceReference")))
                throw new IllegalArgumentException("Unsupported calibration format");
            JSONObject field = root.getJSONObject("field");
            if (field.getDouble("redX") != FieldConstants.RED_HIVE_X_INCHES
                    || field.getDouble("blueX") != FieldConstants.BLUE_HIVE_X_INCHES
                    || field.getDouble("audienceY") != FieldConstants.AUDIENCE_CELL_UP_Y_INCHES
                    || field.getDouble("farY") != FieldConstants.FAR_CELL_UP_Y_INCHES)
                throw new IllegalArgumentException("Field target map changed; recalibration required");
            Map<Ball, ShotTable> result = empty();
            JSONObject tables = root.getJSONObject("tables");
            for (Ball ball : Ball.values()) {
                JSONArray rows = tables.getJSONArray(ball.name());
                if (rows.length() > ShotTable.MAX_POINTS) throw new IllegalArgumentException("Too many samples");
                List<ShotTable.Point> points = new ArrayList<>();
                for (int i = 0; i < rows.length(); i++) {
                    JSONObject row = rows.getJSONObject(i);
                    ShotTable.Point point = new ShotTable.Point(new Pose(row.getDouble("x"), row.getDouble("y"),
                            row.getDouble("headingRadians")), HiveCell.valueOf(row.getString("cell")), row.getDouble("rpm"));
                    double savedDistance = row.getDouble("distanceInches");
                    if (!Double.isFinite(savedDistance) || Math.abs(savedDistance - point.distance) > 1e-6)
                        throw new IllegalArgumentException("Sample distance does not match its field position");
                    points.add(point);
                }
                result.put(ball, new ShotTable(points));
            }
            return result;
        } catch (JSONException | IllegalArgumentException malformed) {
            throw new IOException("Calibration file invalid: " + malformed.getMessage(), malformed);
        }
    }

    public void save(Map<Ball, ShotTable> tables) throws IOException {
        try {
            JSONObject root = new JSONObject().put("version", 1).put("distanceReference", DISTANCE_REFERENCE);
            root.put("field", new JSONObject().put("redX", FieldConstants.RED_HIVE_X_INCHES)
                    .put("blueX", FieldConstants.BLUE_HIVE_X_INCHES)
                    .put("audienceY", FieldConstants.AUDIENCE_CELL_UP_Y_INCHES)
                    .put("farY", FieldConstants.FAR_CELL_UP_Y_INCHES));
            JSONObject data = new JSONObject();
            for (Ball ball : Ball.values()) {
                if (tables.get(ball) == null) throw new IllegalArgumentException("Missing shooter table");
                JSONArray rows = new JSONArray();
                for (ShotTable.Point point : tables.get(ball).points()) {
                    rows.put(new JSONObject().put("distanceInches", point.distance).put("rpm", point.rpm)
                            .put("x", point.x).put("y", point.y).put("headingRadians", point.heading).put("cell", point.cell.name()));
                }
                data.put(ball.name(), rows);
            }
            root.put("tables", data);
            storage.write(root.toString(2));
        } catch (JSONException | IllegalArgumentException invalid) {
            throw new IOException("Cannot save calibration: " + invalid.getMessage(), invalid);
        }
    }

    /** Android's journaled replacement preserves the prior file if a write is interrupted. */
    private static final class AndroidStorage implements Storage {
        private final AtomicFile atomic;
        AndroidStorage(File file) { atomic = new AtomicFile(file); }
        @Override public String read() throws IOException {
            try (FileInputStream input = atomic.openRead(); ByteArrayOutputStream bytes = new ByteArrayOutputStream()) {
                byte[] buffer = new byte[4096];
                int count;
                while ((count = input.read(buffer)) != -1) {
                    if (bytes.size() + count > MAX_BYTES) throw new IOException("Calibration file too large");
                    bytes.write(buffer, 0, count);
                }
                return new String(bytes.toByteArray(), StandardCharsets.UTF_8);
            } catch (FileNotFoundException missing) {
                if (atomic.getBaseFile().exists() || new File(atomic.getBaseFile() + ".bak").exists()) throw missing;
                return null;
            }
        }
        @Override public void write(String content) throws IOException {
            FileOutputStream output = null;
            try {
                output = atomic.startWrite();
                output.write(content.getBytes(StandardCharsets.UTF_8));
                // Explicit sync surfaces failures before finishWrite's internal logging-only sync.
                output.getFD().sync();
                atomic.finishWrite(output);
            } catch (IOException | RuntimeException failed) {
                atomic.failWrite(output);
                throw failed;
            }
            // Some Android versions only log rename failures; do not report a save that did not stick.
            if (!content.equals(read())) throw new IOException("Saved calibration could not be verified");
        }
    }
}
