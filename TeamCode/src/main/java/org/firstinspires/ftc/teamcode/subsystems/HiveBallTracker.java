package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import org.firstinspires.ftc.teamcode.math.HiveBallGeometry;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.LongSupplier;

/**
 * Stationary perception adaptation of Hive Vision's BallTracker (MIT, Harjas Sidhu).
 * Uses one SDK snapshot, exposure-age gating and distinct-frame confirmation; never predicts a lost ball.
 * See third_party/hive-vision/LICENSE and docs/HIVE_VISION_TEST.md.
 */
public final class HiveBallTracker {
    public enum Filter {
        YELLOW(0), RED(1), BLUE(2), ALL(-1);
        private final int classId;
        Filter(int classId) { this.classId = classId; }
        boolean accepts(int id) { return this == ALL || classId == id; }
    }

    public static final class Detection {
        public final int classId;
        public final String className;
        public final double txDegrees, tyDegrees, rawConfidence, confidence, area;
        /** Empty when eligible for tracking. */
        public final String rejection;
        /** Null until calibrated, or when the ray misses the configured floor working area. */
        public final HiveBallGeometry.Position position;

        private Detection(DetectorResult result, double confidenceDivisor, double minimumConfidence,
                          Filter filter, HiveBallGeometry geometry, JSONObject raw) {
            classId = result.getClassId();
            className = result.getClassName();
            // Crosshair offsets are for aiming, not geometric projection from the optical center.
            // SDK 12 silently returns zero when these fields are absent. Do not steer toward
            // that fabricated center point on firmware which omits optical-angle fields.
            txDegrees = raw == null ? Double.NaN : raw.optDouble("tx_nocross", Double.NaN);
            tyDegrees = raw == null ? Double.NaN : raw.optDouble("ty_nocross", Double.NaN);
            rawConfidence = result.getConfidence();
            confidence = rawConfidence / confidenceDivisor;
            area = result.getTargetArea();
            if (classId < 0 || classId > 2) rejection = "Unknown class ID";
            else if (!label(classId).equals(className)) rejection = "Label/order mismatch";
            else if (!Double.isFinite(confidence) || confidence < 0 || confidence > 1)
                rejection = "Invalid confidence scale";
            else if (!Double.isFinite(txDegrees) || !Double.isFinite(tyDegrees)
                    || Math.abs(txDegrees) >= 89 || Math.abs(tyDegrees) >= 89
                    || !Double.isFinite(area) || area <= 0 || area > 100) rejection = "Missing/invalid optical angles or area";
            else if (confidence < minimumConfidence) rejection = "Low confidence";
            else if (!filter.accepts(classId)) rejection = "Class filtered";
            else rejection = "";
            position = rejection.isEmpty() && geometry != null ? geometry.project(txDegrees, tyDegrees) : null;
        }
    }

    private final int pipeline, requiredFrames;
    private final double minimumConfidence, confidenceDivisor, maximumAgeMs, associationGate;
    private final HiveBallGeometry geometry;
    private final VisionResultGate gate;
    private final LongSupplier clock;
    private Filter filter = Filter.YELLOW;
    private List<Detection> detections = Collections.emptyList();
    private Detection candidate;
    private double processedTimestamp = Double.NaN;
    private long lastDistinctFrameNanos, distinctFrames, trackId;
    private int streak;
    private String status = "Waiting for camera";

    public HiveBallTracker(int pipeline) { this(pipeline, System::nanoTime); }

    HiveBallTracker(int pipeline, LongSupplier clock) {
        this.pipeline = pipeline;
        this.clock = clock;
        minimumConfidence = HiveVisionConfig.minimumConfidence;
        confidenceDivisor = HiveVisionConfig.confidenceIsPercent ? 100 : 1;
        maximumAgeMs = HiveVisionConfig.maximumFrameAgeMs;
        requiredFrames = HiveVisionConfig.confirmationFrames;
        associationGate = HiveVisionConfig.associationGateDegrees;
        if (pipeline < 0 || pipeline > 9 || !Double.isFinite(minimumConfidence)
                || minimumConfidence < 0 || minimumConfidence > 1 || !Double.isFinite(maximumAgeMs)
                || maximumAgeMs <= 0 || requiredFrames < 1 || !Double.isFinite(associationGate)
                || associationGate <= 0 || associationGate > 90) {
            throw new IllegalArgumentException("Invalid Hive Vision pipeline, confidence, age, or confirmation settings");
        }
        geometry = HiveVisionConfig.geometryVerified ? new HiveBallGeometry(
                HiveVisionConfig.cameraHeightInches, HiveVisionConfig.ballCenterHeightInches,
                HiveVisionConfig.cameraPitchDownDegrees, HiveVisionConfig.cameraYawLeftDegrees,
                HiveVisionConfig.cameraForwardInches, HiveVisionConfig.cameraLeftInches,
                HiveVisionConfig.maximumCameraRangeInches) : null;
        gate = new VisionResultGate(pipeline, clock);
    }

    /** Main thread only. Pass the same snapshot used by the OpMode's raw telemetry. */
    public void update(LLResult frame) {
        gate.observe(frame);
        if (frame == null) { rejectFrame("Waiting for camera"); return; }
        if (frame.getPipelineIndex() != pipeline) { rejectFrame("Waiting for detector pipeline " + pipeline); return; }
        if (!frame.isValid()) { rejectFrame("No valid targets"); return; }
        if (gate.getFresh(maximumAgeMs) == null) { rejectFrame("Stale frame or invalid timing"); return; }
        double timestamp = frame.getTimestamp();
        // A faster HTTP poll cannot add confirmation frames or rejuvenate a frozen target.
        if (Double.compare(timestamp, processedTimestamp) == 0) return;
        long now = clock.getAsLong();
        double gapMs = (now - lastDistinctFrameNanos) / 1e6;
        if (timestamp < processedTimestamp || gapMs < 0 || gapMs > maximumAgeMs) clearTrack();
        processedTimestamp = timestamp;
        lastDistinctFrameNanos = now;
        distinctFrames++;
        List<Detection> rows = new ArrayList<>();
        List<DetectorResult> raw = frame.getDetectorResults();
        JSONArray payload = null;
        try { payload = new JSONObject(frame.toString()).optJSONArray("Detector"); }
        catch (JSONException invalidJson) { /* Mark rows unusable below. */ }
        if (raw != null) for (int i = 0; i < raw.size(); i++) {
            DetectorResult result = raw.get(i);
            if (result != null) rows.add(new Detection(result, confidenceDivisor, minimumConfidence, filter, geometry,
                    payload == null ? null : payload.optJSONObject(i)));
        }
        detections = Collections.unmodifiableList(rows);
        Detection lowest = null, match = null;
        double smallestJump = Double.POSITIVE_INFINITY;
        for (Detection detection : rows) {
            if (!detection.rejection.isEmpty()) continue;
            // Hive Vision's nearest-first heuristic: lowest in the image for a level-roll floor camera.
            if (lowest == null || detection.tyDegrees < lowest.tyDegrees) lowest = detection;
            if (candidate != null && detection.classId == candidate.classId) {
                double jump = Math.hypot(detection.txDegrees - candidate.txDegrees,
                        detection.tyDegrees - candidate.tyDegrees);
                if (jump <= associationGate && jump < smallestJump) { match = detection; smallestJump = jump; }
            }
        }
        if (match != null) {
            candidate = match;
            streak = Math.min(requiredFrames, streak + 1);
        } else {
            candidate = lowest;
            streak = lowest == null ? 0 : 1;
            if (lowest != null) trackId++;
        }
        status = candidate == null ? "No eligible balls" : streak < requiredFrames ? "Confirming" : "Confirmed target";
    }

    public void setFilter(Filter filter) {
        if (filter == null) throw new IllegalArgumentException("Choose a class filter");
        if (this.filter != filter) { this.filter = filter; reset(); }
    }

    /** Clears selection; reacquisition begins on the next distinct camera frame. */
    public void reset() {
        clearTrack();
        detections = Collections.emptyList();
        status = "Waiting for a new frame";
    }

    public void stop() { reset(); gate.clear(); status = "Stopped"; }
    public Filter getFilter() { return filter; }
    public String getStatus() { return status; }
    public int getConfirmationStreak() { return streak; }
    public int getRequiredFrames() { return requiredFrames; }
    public long getDistinctFrames() { return distinctFrames; }
    /** Changes on reacquisition, including a same-class jump; not a persistent physical object ID. */
    public long getTrackId() { return candidate == null ? 0 : trackId; }
    public double getFrameAgeMs() { return gate.getAgeMs(); }
    public boolean hasGeometry() { return geometry != null; }
    public List<Detection> getDetections() { return detections; }
    public Detection getTarget() {
        return streak >= requiredFrames && gate.getFresh(maximumAgeMs) != null ? candidate : null;
    }

    private void clearTrack() { candidate = null; streak = 0; }
    private void rejectFrame(String reason) { reset(); status = reason; }
    public static String label(int classId) {
        switch (classId) {
            case 0: return "yellow_pollen";
            case 1: return "red_nectar";
            case 2: return "blue_nectar";
            default: return "unknown";
        }
    }
}
