package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.math.HiveAimGeometry;
import org.firstinspires.ftc.teamcode.math.HiveAimGeometry.Vec;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class HiveAutoAimTest {
    private final AtomicLong now = new AtomicLong();
    private final Map<Field, Object> originalSettings = new LinkedHashMap<>();
    private HiveAutoAim aim;

    @Before public void setUp() throws IllegalAccessException, NoSuchFieldException {
        for (Field field : HiveAutoAim.class.getFields()) {
            if (Modifier.isStatic(field.getModifiers()) && !Modifier.isFinal(field.getModifiers()))
                originalSettings.put(field, field.get(null));
        }
        for (String name : new String[]{"limitTravel", "minDegrees", "maxDegrees"}) {
            Field field = AxonTurret.class.getField(name);
            originalSettings.put(field, field.get(null));
        }
        HiveAutoAim.cameraPoseVerified = HiveAutoAim.openingGeometryVerified = HiveAutoAim.cellOrientationVerified = true;
        HiveAutoAim.cameraForward = HiveAutoAim.cameraLeft = HiveAutoAim.cameraUp = 0;
        HiveAutoAim.cameraRollDegrees = HiveAutoAim.cameraPitchDegrees = HiveAutoAim.cameraYawDegrees = 0;
        HiveAutoAim.pivotForward = HiveAutoAim.pivotLeft = HiveAutoAim.turretZeroYawDegrees = 0;
        HiveAutoAim.shooterForward = HiveAutoAim.shooterLeft = HiveAutoAim.shooterYawDegrees = 0;
        // A measured example offset, not a robot-specific field calibration.
        HiveAutoAim.openingTagX = 0; HiveAutoAim.openingTagY = -7.188; HiveAutoAim.openingTagZ = 6;
        HiveAutoAim.openingNormalTagX = 0; HiveAutoAim.openingNormalTagY = -1; HiveAutoAim.openingNormalTagZ = 0;
        HiveAutoAim.upPitchDegrees = 30; HiveAutoAim.downPitchDegrees = -30; HiveAutoAim.pitchToleranceDegrees = 8;
        HiveAutoAim.pitchAgreementDegrees = 4; HiveAutoAim.centerAgreementInches = 2; HiveAutoAim.normalAgreementDegrees = 8;
        HiveAutoAim.minimumTagArea = 0.02; HiveAutoAim.maxFrameAgeMs = 150; HiveAutoAim.extraLatencyMs = 0;
        HiveAutoAim.stableDwellMs = 200; HiveAutoAim.stablePitchDegrees = 3; HiveAutoAim.stablePositionInches = 2;
        HiveAutoAim.maximumPredictionAgeMs = 0;
        HiveAutoAim.minimumRangeInches = 12; HiveAutoAim.maximumRangeInches = 180; HiveAutoAim.minimumApproachDegrees = 30;
        AxonTurret.limitTravel = true; AxonTurret.minDegrees = -180; AxonTurret.maxDegrees = 180;
        aim = new HiveAutoAim(2, now::get);
    }

    @After public void tearDown() throws IllegalAccessException {
        for (Map.Entry<Field, Object> entry : originalSettings.entrySet()) entry.getKey().set(null, entry.getValue());
    }

    @Test public void twoMeterUnitTagPosesRecoverOpeningAndRequireDistinctFramesAndDwell() {
        sample(0, standardFrame(1));
        sample(50, standardFrame(1));
        sample(100, standardFrame(2));
        assertFalse(aim.hasAim());
        sample(200, standardFrame(3));
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(HiveAutoAim.CellState.UP, aim.getCellState());
        assertEquals(30, aim.getPitchDegrees(), 1e-8);
        assertEquals(2, aim.getTagCount());
        assertEquals(100, aim.getRangeInches(), 1e-8);
        assertEquals(0, aim.getTurretGoalDegrees(), 1e-8);
        assertEquals(90, aim.getApproachDegrees(), 1e-6);
        assertFalse(aim.isPredicted());
    }

    @Test public void signedTagUpDistinguishesDownAndTransitionImmediately() {
        acquire();
        sample(220, pair(4, 120, new Vec(0, -40, 100)));
        assertEquals(-30, aim.getPitchDegrees(), 1e-8);
        assertEquals(HiveAutoAim.CellState.DOWN, aim.getCellState());
        assertFalse(aim.hasAim());
        sample(240, pair(5, 90, new Vec(0, -40, 100)));
        assertEquals(HiveAutoAim.CellState.TRANSITION, aim.getCellState());
        assertFalse(aim.hasAim());
        assertTrue(Double.isNaN(aim.getTurretGoalDegrees()));
    }

    @Test public void unrelatedTagsCannotSubstituteForSelectedCell() {
        Vec opening = new Vec(0, -40, 100);
        sample(0, frame(1, tag(38, opening, 60, 0, 0), tag(39, opening, 60, 0, 0)));
        assertFalse(aim.hasAim()); assertEquals(0, aim.getTagCount());
        sample(100, frame(2, tag(30, opening, 60, 0, 0), tag(31, opening, 60, 0, 0)));
        assertFalse(aim.hasAim());
        sample(200, frame(3, tag(34, opening, 60, 0, 0), tag(38, opening, 60, 0, 0)));
        assertFalse(aim.hasAim());
        now.set(300_000_000L);
        aim.update(standardFrame(4), 0, true, 0, 0, 0, true, 0, Alliance.BLUE, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim()); assertEquals(HiveAutoAim.CellState.UNKNOWN, aim.getCellState());
    }

    @Test public void duplicatesAndInconsistentTagSolutionsFailClosed() {
        Vec opening = new Vec(0, -40, 100);
        sample(0, frame(1, tag(34, opening, 60, 0, 0), tag(34, opening, 60, 0, 0)));
        assertTrue(aim.getStatus().contains("Duplicate"));
        sample(100, frame(2, tag(34, opening, 60, 0, 0), tag(37, opening, 120, 0, 0)));
        assertTrue(aim.getStatus().contains("orientations disagree"));
        sample(200, frame(3, tag(34, opening, 60, 0, 0), tag(37, opening.plus(new Vec(10, 0, 0)), 60, 0, 0)));
        assertTrue(aim.getStatus().contains("openings disagree"));
        assertFalse(aim.hasAim());
    }

    @Test public void tinyMissingOrBehindCameraPosesDoNotCountAsValidTags() {
        FiducialResult good = tag(34, new Vec(0, -40, 100), 60, 0, 0);
        FiducialResult tiny = tag(37, new Vec(0, -40, 100), 60, 0, 0);
        when(tiny.getTargetArea()).thenReturn(0.001);
        sample(0, frame(1, good, tiny));
        assertFalse(aim.hasAim());
        FiducialResult missing = mock(FiducialResult.class);
        when(missing.getFiducialId()).thenReturn(37); when(missing.getTargetArea()).thenReturn(1.0);
        sample(100, frame(2, good, missing));
        assertFalse(aim.hasAim());
        sample(200, frame(3, good, tag(37, new Vec(0, 0, -100), 60, 0, 0)));
        assertFalse(aim.hasAim());
    }

    @Test public void noHistoryBeforeExposurePreventsAiming() {
        now.set(100_000_000L);
        aim.update(standardFrame(1), 50, true, 0, 0, 0, true, 0, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        assertTrue(aim.getStatus().contains("bracketing"));
        now.set(150_000_000L);
        aim.update(standardFrame(2), 25, true, 0, 0, 0, true, 0, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertEquals(HiveAutoAim.CellState.UP, aim.getCellState());
        assertFalse(aim.hasAim()); // One observation is still insufficient.
    }

    @Test public void turretAndRobotMotionUseCapturePoseRatherThanCurrentPose() {
        sample(0, null);
        for (int step = 1; step <= 5; step++) {
            long ms = step * 50L;
            double captureTurret = (step - 1) * 10;
            double captureY = step - 1;
            Vec targetAtCapture = new Vec(100, -captureY, 40);
            Vec relative = HiveAimGeometry.rotate(targetAtCapture, 0, 0, -captureTurret);
            Vec opticalOpening = new Vec(-relative.y, -relative.z, relative.x);
            LLResult result = frame(step, tag(34, opticalOpening, 60, captureTurret, 0),
                    tag(37, opticalOpening, 60, captureTurret, 0));
            now.set(ms * 1_000_000L);
            aim.update(result, 50, true, 0, step, 0, true, step * 10,
                    Alliance.RED, HiveCell.RED_AUDIENCE);
        }
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(Math.toDegrees(Math.atan2(-5, 100)), aim.getTurretGoalDegrees(), 1e-8);
        assertEquals(Math.hypot(100, 5), aim.getRangeInches(), 1e-8);
        assertEquals(50, aim.getExposureAgeMs(), 1e-8);
    }

    @Test public void calibratedCameraTranslationAndPivotOffsetRemoveParallax() {
        HiveAutoAim.cameraForward = 3; HiveAutoAim.cameraLeft = 4; HiveAutoAim.cameraUp = 8;
        HiveAutoAim.pivotForward = 2; HiveAutoAim.pivotLeft = 1;
        Vec openingFromCamera = new Vec(-15, -32, 95); // World opening (100,20,40), camera (5,5,8).
        for (int i = 0; i < 3; i++) sample(i * 100L, pair(i + 1, 60, openingFromCamera));
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(Math.toDegrees(Math.atan2(19, 98)), aim.getTurretGoalDegrees(), 1e-8);
        assertEquals(Math.hypot(98, 19), aim.getRangeInches(), 1e-8);
    }

    @Test public void frozenFrameCannotRefreshExposureOrAccumulateConfirmation() {
        sample(0, standardFrame(1));
        sample(50, standardFrame(1));
        sample(100, standardFrame(1));
        assertFalse(aim.hasAim());
        sample(151, standardFrame(1));
        assertFalse(aim.hasAim());
        sample(200, standardFrame(2)); sample(300, standardFrame(3)); sample(400, standardFrame(4));
        assertTrue(aim.getStatus(), aim.hasAim());
        sample(500, standardFrame(4));
        assertTrue(aim.hasAim());
        sample(551, standardFrame(4));
        assertFalse(aim.hasAim());
    }

    @Test public void ageAndLiveConfigurationAreRecheckedAtConsumption() {
        acquire();
        now.set(351_000_000L);
        assertFalse(aim.hasAim());
        assertTrue(Double.isNaN(aim.getTurretGoalDegrees()));
        aim = new HiveAutoAim(2, now::get);
        acquire();
        HiveAutoAim.cameraPoseVerified = false;
        assertFalse(aim.hasAim());
        HiveAutoAim.cameraPoseVerified = true;
        HiveAutoAim.cameraYawDegrees = 1;
        assertFalse(aim.hasAim());
    }

    @Test public void configurationChangesRequireFreshStableObservationWindow() {
        acquire();
        HiveAutoAim.shooterLeft = 2;
        assertFalse(aim.hasAim());
        sample(220, standardFrame(4));
        assertFalse(aim.hasAim());
        sample(320, standardFrame(5)); sample(420, standardFrame(6));
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(Math.toDegrees(Math.asin(-0.02)), aim.getTurretGoalDegrees(), 1e-8);
    }

    @Test public void unverifiedCalibrationsMayDisplayPitchButCannotProduceAim() {
        HiveAutoAim.cellOrientationVerified = false;
        sample(0, standardFrame(1)); sample(100, standardFrame(2)); sample(200, standardFrame(3));
        assertEquals(30, aim.getPitchDegrees(), 1e-8);
        assertFalse(aim.hasAim());
        assertTrue(Double.isNaN(aim.getTurretGoalDegrees()));
        assertEquals(0, aim.getCandidateGoalDegrees(), 1e-8);
        assertEquals(100, aim.getRangeInches(), 1e-8);
        HiveAutoAim.cellOrientationVerified = true; HiveAutoAim.openingGeometryVerified = false;
        sample(220, standardFrame(4));
        assertFalse(aim.hasAim());
        HiveAutoAim.openingGeometryVerified = true; HiveAutoAim.cameraPoseVerified = false;
        sample(240, standardFrame(5));
        assertFalse(aim.hasAim());
    }

    @Test public void lossOrWrongPipelineStopsUnlessExplicitShortPredictionIsEnabled() {
        acquire();
        sample(220, null);
        assertFalse(aim.hasAim());
        aim = new HiveAutoAim(2, now::get);
        HiveAutoAim.maximumPredictionAgeMs = 300;
        acquire();
        sample(250, null);
        assertTrue(aim.hasAim()); assertTrue(aim.isPredicted());
        LLResult wrong = standardFrame(4); when(wrong.getPipelineIndex()).thenReturn(3);
        sample(350, wrong);
        assertTrue(aim.hasAim()); assertTrue(aim.isPredicted());
        sample(501, null);
        assertFalse(aim.hasAim()); // Total age since the final exposure, not time since last loss.
    }

    @Test public void newDownObservationCancelsPredictionImmediately() {
        HiveAutoAim.maximumPredictionAgeMs = 500;
        acquire();
        sample(220, null);
        assertTrue(aim.isPredicted());
        sample(240, pair(4, 120, new Vec(0, -40, 100)));
        assertFalse(aim.hasAim()); assertFalse(aim.isPredicted());
        assertEquals(HiveAutoAim.CellState.DOWN, aim.getCellState());
    }

    @Test public void healthOrTurretReferenceLossInvalidatesHistoryAndTarget() {
        acquire();
        now.set(220_000_000L);
        aim.update(standardFrame(4), 0, false, 0, 0, 0, true, 0, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        now.set(240_000_000L);
        aim.update(standardFrame(5), 50, true, 0, 0, 0, true, 0, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        assertTrue(aim.getStatus().contains("bracketing"));
        now.set(260_000_000L);
        aim.update(standardFrame(6), 0, true, 0, 0, 0, false, 0, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
    }

    @Test public void currentTurretUsesEquivalentUnwrappedAngleAndRespectTravelInterval() {
        AxonTurret.limitTravel = false;
        for (int i = 0; i < 3; i++) {
            now.set(i * 100_000_000L);
            aim.update(standardFrame(i + 1), 0, true, 0, 0, 0, true, 360,
                    Alliance.RED, HiveCell.RED_AUDIENCE);
        }
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(360, aim.getTurretGoalDegrees(), 1e-8);
        AxonTurret.limitTravel = true; AxonTurret.minDegrees = 20; AxonTurret.maxDegrees = 160;
        for (int i = 0; i < 3; i++) {
            now.set((300 + i * 100) * 1_000_000L);
            aim.update(standardFrame(i + 4), 0, true, 0, 0, 0, true, 360,
                    Alliance.RED, HiveCell.RED_AUDIENCE);
        }
        assertFalse(aim.hasAim());
        assertTrue(Double.isNaN(aim.getTurretGoalDegrees()));
    }

    @Test public void shallowOrBacksideApproachDoesNotBecomeAimReady() {
        HiveAutoAim.minimumApproachDegrees = 60;
        Vec oblique = new Vec(-100, -40, 50);
        for (int i = 0; i < 3; i++) sample(i * 100L, pair(i + 1, 60, oblique));
        assertFalse(aim.hasAim());
        assertEquals(Math.toDegrees(Math.asin(50 / Math.hypot(50, 100))), aim.getApproachDegrees(), 1e-6);
        HiveAutoAim.openingNormalTagY = 1;
        for (int i = 0; i < 3; i++) sample(300 + i * 100L, standardFrame(i + 4));
        assertFalse(aim.hasAim());
        assertEquals(-90, aim.getApproachDegrees(), 1e-6);
    }

    @Test public void impossibleMuzzleGeometryAndNonfiniteSettingsFailWithoutThrowing() {
        HiveAutoAim.shooterLeft = 150;
        sample(0, standardFrame(1)); sample(100, standardFrame(2)); sample(200, standardFrame(3));
        assertFalse(aim.hasAim());
        HiveAutoAim.shooterLeft = 0; HiveAutoAim.cameraRollDegrees = Double.NaN;
        sample(220, standardFrame(4));
        assertFalse(aim.hasAim());
    }

    private void acquire() {
        sample(0, standardFrame(1)); sample(100, standardFrame(2)); sample(200, standardFrame(3));
        assertTrue(aim.getStatus(), aim.hasAim());
    }

    private void sample(long ms, LLResult frame) {
        now.set(ms * 1_000_000L);
        aim.update(frame, 0, true, 0, 0, 0, true, 0, Alliance.RED, HiveCell.RED_AUDIENCE);
    }

    private static LLResult standardFrame(double timestamp) { return pair(timestamp, 60, new Vec(0, -40, 100)); }

    private static LLResult pair(double timestamp, double opticalRx, Vec opening) {
        return frame(timestamp, tag(34, opening, opticalRx, 0, 0), tag(37, opening, opticalRx, 0, 0));
    }

    private static LLResult frame(double timestamp, FiducialResult... tags) {
        LLResult frame = VisionResultGateTest.result(timestamp, 2);
        when(frame.getFiducialResults()).thenReturn(Arrays.asList(tags));
        return frame;
    }

    /** Synthesize camera-space poses from an opening, sticker offset, and optical Euler angles. */
    private static FiducialResult tag(int id, Vec opticalOpening, double rx, double ry, double rz) {
        double[] rowOffsets = {-6.5, -2.75, 2.75, 6.5};
        Vec openingOffset = HiveAimGeometry.rotate(new Vec(HiveAutoAim.openingTagX,
                HiveAutoAim.openingTagY, HiveAutoAim.openingTagZ), rx, ry, rz);
        Vec rowOffset = HiveAimGeometry.rotate(new Vec(rowOffsets[(id - 30) % 4], 0, 0), rx, ry, rz);
        Vec position = opticalOpening.minus(openingOffset).plus(rowOffset);
        Pose3D pose = new Pose3D(new Position(DistanceUnit.METER, position.x * 0.0254,
                position.y * 0.0254, position.z * 0.0254, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, rz, ry, rx, 0));
        FiducialResult tag = mock(FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetArea()).thenReturn(0.5);
        when(tag.getTargetPoseCameraSpace()).thenReturn(pose);
        return tag;
    }
}
