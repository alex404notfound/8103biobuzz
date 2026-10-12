package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class HiveBallTrackerTest {
    private final Map<Field, Object> saved = new HashMap<>();
    private final AtomicLong now = new AtomicLong();
    private HiveBallTracker tracker;

    @Before public void setUp() throws Exception {
        for (Field field : HiveVisionConfig.class.getFields()) saved.put(field, field.get(null));
        HiveVisionConfig.minimumConfidence = 0.44;
        HiveVisionConfig.confidenceIsPercent = false;
        HiveVisionConfig.maximumFrameAgeMs = 150;
        HiveVisionConfig.confirmationFrames = 4;
        HiveVisionConfig.associationGateDegrees = 12;
        HiveVisionConfig.geometryVerified = false;
        tracker = new HiveBallTracker(1, now::get);
    }

    @After public void tearDown() throws Exception {
        for (Map.Entry<Field, Object> value : saved.entrySet()) value.getKey().set(null, value.getValue());
    }

    @Test public void confirmsOnlyDistinctCameraFramesWithoutTruncatingFractionalTimestamps() {
        DetectorResult ball = detection(0, 0, -10, 0.9);
        tracker.update(frame(1.1, ball));
        for (int i = 0; i < 20; i++) tracker.update(frame(1.1, ball));
        assertEquals(1, tracker.getConfirmationStreak());
        assertEquals(1, tracker.getDistinctFrames());
        tracker.update(frame(1.2, ball));
        tracker.update(frame(1.3, ball));
        assertNull(tracker.getTarget());
        tracker.update(frame(1.4, ball));
        assertNotNull(tracker.getTarget());
        assertEquals(4, tracker.getDistinctFrames());
    }

    @Test public void staleRepeatedHttpResponsesLoseTargetAndCannotRefreshItsAge() {
        confirm(detection(0, 0, 0, 0.9));
        now.set(TimeUnit.MILLISECONDS.toNanos(151));
        tracker.update(frame(4, detection(0, 0, 0, 0.9)));
        assertNull(tracker.getTarget());
        assertEquals(0, tracker.getConfirmationStreak());
        tracker.update(frame(5, detection(0, 0, 0, 0.9)));
        assertEquals(1, tracker.getConfirmationStreak());
    }

    @Test public void targetReadRechecksAgeEvenWithoutAnotherUpdate() {
        confirm(detection(0, 0, 0, 0.9));
        now.set(TimeUnit.MILLISECONDS.toNanos(151));
        assertNull(tracker.getTarget());
    }

    @Test public void captureProcessingAndParseLatencyCountTowardExposureAge() {
        LLResult frame = frame(1, detection(0, 0, 0, 0.9));
        when(frame.getCaptureLatency()).thenReturn(70.0);
        when(frame.getTargetingLatency()).thenReturn(70.0);
        when(frame.getParseLatency()).thenReturn(11.0);
        tracker.update(frame);
        assertEquals(151, tracker.getFrameAgeMs(), 1e-9);
        assertEquals(0, tracker.getConfirmationStreak());
    }

    @Test public void missingInvalidAndWrongPipelineFramesClearConfirmedTargets() {
        confirm(detection(0, 0, 0, 0.9));
        tracker.update(null);
        assertNull(tracker.getTarget());
        LLResult wrong = frame(5, detection(0, 0, 0, 0.9));
        when(wrong.getPipelineIndex()).thenReturn(0);
        tracker.update(wrong);
        assertTrue(tracker.getStatus().contains("pipeline 1"));
        LLResult invalid = frame(6, detection(0, 0, 0, 0.9));
        when(invalid.isValid()).thenReturn(false);
        tracker.update(invalid);
        assertEquals(0, tracker.getConfirmationStreak());
        assertTrue(tracker.getDetections().isEmpty());
    }

    @Test public void noDetectionsOrOnlyFilteredDetectionsImmediatelyLoseTargetWithoutPrediction() {
        confirm(detection(0, 0, 0, 0.9));
        tracker.update(frame(5));
        assertNull(tracker.getTarget());
        tracker.update(frame(6, detection(1, 0, 0, 0.9)));
        assertEquals(0, tracker.getConfirmationStreak());
        assertEquals("Class filtered", tracker.getDetections().get(0).rejection);
    }

    @Test public void staysWithConfirmedBallWhenAnotherBallMovesLowerInImage() {
        confirm(detection(0, -5, 0, 0.9));
        tracker.update(frame(5, detection(0, 30, -20, 0.99), detection(0, -6, -1, 0.9)));
        assertEquals(-6, tracker.getTarget().txDegrees, 1e-9);
    }

    @Test public void jumpBeyondGateStartsFreshConfirmation() {
        confirm(detection(0, -30, 0, 0.9));
        tracker.update(frame(5, detection(0, 30, 0, 0.9)));
        assertNull(tracker.getTarget());
        assertEquals(1, tracker.getConfirmationStreak());
    }

    @Test public void colorChangeCannotInheritAnotherColorsConfirmation() {
        tracker.setFilter(HiveBallTracker.Filter.ALL);
        confirm(detection(1, 0, 0, 0.9));
        tracker.update(frame(5, detection(2, 0, 0, 0.9)));
        assertNull(tracker.getTarget());
        assertEquals(1, tracker.getConfirmationStreak());
    }

    @Test public void changingFilterOrResettingWaitsForNextDistinctFrame() {
        confirm(detection(0, 0, 0, 0.9));
        tracker.setFilter(HiveBallTracker.Filter.ALL);
        tracker.update(frame(4, detection(0, 0, 0, 0.9)));
        assertEquals(0, tracker.getConfirmationStreak());
        tracker.update(frame(5, detection(2, 0, 0, 0.9)));
        assertEquals(1, tracker.getConfirmationStreak());
        tracker.reset();
        tracker.update(frame(5, detection(2, 0, 0, 0.9)));
        assertEquals(0, tracker.getConfirmationStreak());
    }

    @Test public void rebootsAndLongObservationGapsRestartConfirmation() {
        confirm(detection(0, 0, 0, 0.9));
        tracker.update(frame(0.5, detection(0, 0, 0, 0.9)));
        assertEquals(1, tracker.getConfirmationStreak());
        now.set(TimeUnit.MILLISECONDS.toNanos(151));
        tracker.update(frame(0.6, detection(0, 0, 0, 0.9)));
        assertEquals(1, tracker.getConfirmationStreak());
        assertNull(tracker.getTarget());
    }

    @Test public void filtersUnknownClassesMislabeledOutputsBadNumbersAndLowConfidence() {
        DetectorResult mislabeled = detection(0, 0, 0, 0.9);
        when(mislabeled.getClassName()).thenReturn("blue_nectar");
        DetectorResult badArea = detection(0, 0, 0, 0.9);
        when(badArea.getTargetArea()).thenReturn(-1.0);
        tracker.update(frame(1, detection(7, 0, 0, 0.9), mislabeled,
                detection(0, 0, 0, Double.NaN), detection(0, Double.NaN, 0, 0.9),
                detection(0, 0, 0, 0.2), badArea, detection(0, 0, 0, 90)));
        assertEquals(7, tracker.getDetections().size());
        assertTrue(tracker.getDetections().stream().noneMatch(d -> d.rejection.isEmpty()));
        assertEquals(0, tracker.getConfirmationStreak());
    }

    @Test public void percentConfidenceRequiresExplicitConfigurationAndIsNormalized() {
        HiveVisionConfig.confidenceIsPercent = true;
        tracker = new HiveBallTracker(1, now::get);
        confirm(detection(0, 0, 0, 90));
        assertEquals(0.9, tracker.getTarget().confidence, 1e-9);
        assertEquals(90, tracker.getTarget().rawConfidence, 1e-9);
    }

    @Test public void uncalibratedTargetHasAnglesButNeverPlausibleMadeUpDistance() {
        confirm(detection(0, 5, -10, 0.9));
        assertNotNull(tracker.getTarget());
        assertNull(tracker.getTarget().position);
        assertFalse(tracker.hasGeometry());
    }

    @Test public void measuredGeometryUsesOpticalAnglesRatherThanAimingCrosshair() {
        HiveVisionConfig.geometryVerified = true;
        HiveVisionConfig.cameraHeightInches = 15;
        HiveVisionConfig.ballCenterHeightInches = 3;
        HiveVisionConfig.cameraPitchDownDegrees = 45;
        HiveVisionConfig.cameraYawLeftDegrees = HiveVisionConfig.cameraForwardInches = HiveVisionConfig.cameraLeftInches = 0;
        tracker = new HiveBallTracker(1, now::get);
        DetectorResult ball = detection(0, 0, 0, 0.9);
        when(ball.getTargetXDegrees()).thenReturn(20.0);
        when(ball.getTargetYDegrees()).thenReturn(20.0);
        confirm(ball);
        assertEquals(12, tracker.getTarget().position.forwardInches, 1e-9);
        assertEquals(0, tracker.getTarget().position.leftInches, 1e-9);
    }

    @Test public void invalidConfigurationFailsBeforeCameraAccessAndSettingsAreCapturedAtInit() {
        HiveVisionConfig.minimumConfidence = Double.NaN;
        assertThrows(IllegalArgumentException.class, () -> new HiveBallTracker(1, now::get));
        confirm(detection(0, 0, 0, 0.9));
        assertNotNull(tracker.getTarget());
        HiveVisionConfig.minimumConfidence = 0.44;
        HiveVisionConfig.confirmationFrames = 0;
        assertThrows(IllegalArgumentException.class, () -> new HiveBallTracker(1, now::get));
        HiveVisionConfig.confirmationFrames = 4;
        assertThrows(IllegalArgumentException.class, () -> new HiveBallTracker(10, now::get));
    }

    @Test public void stopClearsAllObservations() {
        confirm(detection(0, 0, 0, 0.9));
        tracker.stop();
        assertNull(tracker.getTarget());
        assertTrue(tracker.getDetections().isEmpty());
        assertEquals("Stopped", tracker.getStatus());
    }

    @Test public void missingOpticalFieldsCannotBecomeFabricatedZeroAngleTarget() {
        LLResult result = frame(1, detection(0, 0, 0, 0.9));
        when(result.toString()).thenReturn("{\"Detector\":[{\"tx\":12,\"ty\":-8}]}");
        tracker.update(result);
        assertNull(tracker.getTarget());
        assertTrue(tracker.getDetections().get(0).rejection.contains("optical angles"));
    }

    @Test public void targetIdentityChangesOnSameColorJumpAndReacquisition() {
        confirm(detection(0, 0, 0, 0.9));
        long first = tracker.getTrackId();
        tracker.update(frame(5, detection(0, 1, 0, 0.9)));
        assertEquals(first, tracker.getTrackId());
        tracker.update(frame(6, detection(0, 30, 0, 0.9)));
        assertNotEquals(first, tracker.getTrackId());
        tracker.reset();
        assertEquals(0, tracker.getTrackId());
    }

    private void confirm(DetectorResult detection) {
        for (int i = 1; i <= 4; i++) tracker.update(frame(i, detection));
        assertNotNull(tracker.getTarget());
    }

    private static LLResult frame(double timestamp, DetectorResult... detections) {
        LLResult result = mock(LLResult.class);
        when(result.getPipelineIndex()).thenReturn(1);
        when(result.isValid()).thenReturn(true);
        when(result.getTimestamp()).thenReturn(timestamp);
        when(result.getDetectorResults()).thenReturn(Arrays.asList(detections));
        try {
            JSONArray payload = new JSONArray();
            for (DetectorResult detection : detections) {
                JSONObject row = new JSONObject();
                if (Double.isFinite(detection.getTargetXDegreesNoCrosshair())) row.put("tx_nocross", detection.getTargetXDegreesNoCrosshair());
                if (Double.isFinite(detection.getTargetYDegreesNoCrosshair())) row.put("ty_nocross", detection.getTargetYDegreesNoCrosshair());
                payload.put(row);
            }
            when(result.toString()).thenReturn(new JSONObject().put("Detector", payload).toString());
        } catch (JSONException invalidFixture) { throw new AssertionError(invalidFixture); }
        return result;
    }

    private static DetectorResult detection(int classId, double tx, double ty, double confidence) {
        DetectorResult result = mock(DetectorResult.class);
        when(result.getClassId()).thenReturn(classId);
        when(result.getClassName()).thenReturn(HiveBallTracker.label(classId));
        when(result.getTargetXDegreesNoCrosshair()).thenReturn(tx);
        when(result.getTargetYDegreesNoCrosshair()).thenReturn(ty);
        when(result.getConfidence()).thenReturn(confidence);
        when(result.getTargetArea()).thenReturn(2.0);
        return result;
    }
}
