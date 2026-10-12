package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import java.lang.reflect.Field;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class HivePedroControllerTest {
    private final Map<Field, Object> saved = new HashMap<>();
    private final AtomicLong now = new AtomicLong();
    private final AtomicBoolean pathDone = new AtomicBoolean();
    private final Drivetrain drive = mock(Drivetrain.class);
    private HiveBallTracker tracker;
    private HivePedroController control;
    private int frameId;

    @Before public void setUp() throws Exception {
        Scheduler.reset();
        for (Class<?> type : new Class<?>[]{HiveVisionConfig.class, HivePedroConfig.class})
            for (Field field : type.getFields()) saved.put(field, field.get(null));
        HiveVisionConfig.geometryVerified = true;
        HiveVisionConfig.cameraHeightInches = 15;
        HiveVisionConfig.ballCenterHeightInches = 3;
        HiveVisionConfig.cameraPitchDownDegrees = 45;
        HiveVisionConfig.cameraYawLeftDegrees = HiveVisionConfig.cameraForwardInches = HiveVisionConfig.cameraLeftInches = 0;
        HiveVisionConfig.confirmationFrames = 4;
        when(drive.isLocalizationReady()).thenReturn(true);
        when(drive.isPathFollowingConfigured()).thenReturn(true);
        when(drive.getPose()).thenReturn(Pose.zero());
        when(drive.getVelocity()).thenReturn(Velocity.zero());
        when(drive.getMotorPowerLimit()).thenReturn(1.0);
        when(drive.followPath(any())).thenAnswer(call -> Command.build().requiring(drive).setDone(pathDone::get));
        create();
    }

    private void create() {
        tracker = new HiveBallTracker(1, now::get);
        control = new HivePedroController(drive, tracker, now::get);
        control.start();
    }

    @After public void tearDown() throws Exception {
        control.stop();
        Scheduler.reset();
        for (Map.Entry<Field, Object> value : saved.entrySet()) value.getKey().set(null, value.getValue());
    }

    @Test public void heldThroughStartCannotMoveUntilReleasedAndPressedAgain() {
        for (int i = 0; i < 7; i++) tick(50, 0, true);
        verify(drive, never()).followPath(any());
        control.beforeDrive(false, false);
        control.afterSensors();
        control.beforeDrive(true, false);
        control.afterSensors();
        assertEquals(HivePedroController.Phase.TRAVEL, control.getPhase());
    }

    @Test public void projectedFieldBallProducesPedroStandoffWithCurrentHeadingAndOwnsDrive() {
        when(drive.getPose()).thenReturn(new Pose(10, 20, Math.PI / 2));
        begin(50, 0);
        ArgumentCaptor<Path> path = ArgumentCaptor.forClass(Path.class);
        verify(drive).followPath(path.capture());
        assertEquals(10, path.getValue().get(0).x(), 1e-8);
        assertEquals(20, path.getValue().get(0).y(), 1e-8);
        assertEquals(10, path.getValue().endPose().x(), 1e-8);
        assertEquals(40, path.getValue().endPose().y(), 1e-8);
        assertEquals(Math.PI / 2, path.getValue().endPose().heading(), 1e-8);
        verify(drive).setMotorPowerLimit(0.25);
        // A new Ivy drive owner must interrupt the whole approach, including the delegated path.
        Command.build().requiring(drive).schedule();
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        verify(drive).setMotorPowerLimit(1.0);
        verify(drive, atLeastOnce()).stop();
    }

    @Test public void nearBallSkipsPathAndCorrectsWithLinearForwardAndCounterclockwiseCommands() {
        begin(24, 6);
        assertEquals(HivePedroController.Phase.CHASE, control.getPhase());
        verify(drive, never()).followPath(any());
        Scheduler.execute();
        verify(drive).driveRobotRelative(doubleThat(x -> x > 0 && x <= .25), eq(0.0), doubleThat(x -> x > 0 && x <= .25));
    }

    @Test public void largeBearingOnlyTurnsAndNeverDrivesForwardBlindly() {
        begin(20, -10);
        Scheduler.execute();
        verify(drive).driveRobotRelative(eq(0.0), eq(0.0), doubleThat(x -> x < 0 && x >= -.25));
    }

    @Test public void pedroCompletionStopsEndHoldBeforeCameraControlTakesOver() {
        begin(50, 0);
        pathDone.set(true);
        Scheduler.execute();
        assertEquals(HivePedroController.Phase.CHASE, control.getPhase());
        verify(drive).stopFollowing();
        Scheduler.execute();
        verify(drive).driveRobotRelative(eq(.25), eq(0.0), eq(0.0));
    }

    @Test public void reachStopsAndHoldingTriggerCannotLaunchAnotherAttempt() {
        begin(18, 0);
        Scheduler.execute();
        assertEquals(HivePedroController.Phase.REACHED, control.getPhase());
        verify(drive).setMotorPowerLimit(1.0);
        clearInvocations(drive);
        for (int i = 0; i < 10; i++) tick(18, 0, true);
        verify(drive, never()).setMotorPowerLimit(anyDouble());
        verify(drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
    }

    @Test public void triggerReleaseAndCancelStopImmediatelyAndRestoreEarlierPowerCap() {
        when(drive.getMotorPowerLimit()).thenReturn(.2);
        begin(50, 0);
        verify(drive).setMotorPowerLimit(.2);
        control.beforeDrive(false, false);
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        verify(drive, times(2)).setMotorPowerLimit(.2);
        verify(drive, atLeastOnce()).stop();
        clearInvocations(drive);
        control.beforeDrive(true, true);
        control.afterSensors();
        verify(drive, never()).followPath(any());
    }

    @Test public void missingTargetAbortsBeforeNextDriveTickAndCannotAutoResume() {
        begin(50, 0);
        tracker.update(null);
        control.beforeDrive(true, false);
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        verify(drive, atLeastOnce()).stop();
        clearInvocations(drive);
        for (int i = 0; i < 8; i++) tick(50, 0, true);
        verify(drive, never()).followPath(any());
    }

    @Test public void frozenCameraFrameAbortsDespiteContinuedHttpResponses() {
        begin(50, 0);
        now.addAndGet(151_000_000L);
        control.beforeDrive(true, false);
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        assertTrue(control.getStatus().contains("stale"));
    }

    @Test public void currentLoopLocalizationFaultCannotBecomeNaturalPathCompletion() {
        begin(50, 0);
        pathDone.set(true);
        when(drive.isLocalizationReady()).thenReturn(false);
        control.afterSensors();
        Scheduler.execute();
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        verify(drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
    }

    @Test public void movementBudgetAndTimeoutEndAnAttempt() {
        begin(50, 0);
        now.addAndGet(8_000_000_000L);
        control.beforeDrive(true, false);
        assertEquals("Attempt timed out", control.getStatus());
        create();
        begin(50, 0);
        when(drive.getPose()).thenReturn(new Pose(49, 0, 0));
        control.afterSensors();
        assertEquals("Travel budget reached", control.getStatus());
    }

    @Test public void aDifferentTrackCannotTakeOverWhileTheTriggerIsHeld() {
        HiveVisionConfig.confirmationFrames = 1;
        create();
        begin(50, 0);
        tracker.reset();
        tracker.update(frame(++frameId, 50, 0));
        control.beforeDrive(true, false);
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        assertTrue(control.getStatus().contains("Target changed"));
    }

    @Test public void ballMovingInFieldAbortsEvenIfAngularAssociationKeepsItsTrack() {
        begin(50, 0);
        tracker.update(frame(++frameId, 38, 0));
        control.beforeDrive(true, false);
        assertEquals(HivePedroController.Phase.ABORTED, control.getPhase());
        assertTrue(control.getStatus().contains("position shifted"));
    }

    @Test public void uncalibratedCameraOrUntunedPedroCannotBeginMotion() {
        HiveVisionConfig.geometryVerified = false;
        create();
        begin(50, 0);
        verify(drive, never()).followPath(any());
        assertTrue(control.getStatus().contains("geometry"));
        when(drive.isPathFollowingConfigured()).thenReturn(false);
        control.afterSensors();
        assertTrue(control.getStatus().contains("AutoTune"));
    }

    @Test public void sparseStationarySamplesDoNotCountAsASettledRobot() {
        for (int i = 0; i < 4; i++) {
            now.addAndGet(200_000_000L);
            tracker.update(frame(++frameId, 50, 0));
            control.beforeDrive(false, false);
            control.afterSensors();
        }
        control.beforeDrive(true, false);
        control.afterSensors();
        verify(drive, never()).followPath(any());
        assertTrue(control.getStatus().contains("250 ms"));
    }

    @Test public void oversizedPowerAndInvalidDistancesAreRejectedAtConstruction() {
        HivePedroConfig.maximumMotorPower = .8;
        assertThrows(IllegalArgumentException.class, () -> new HivePedroController(drive, tracker, now::get));
        HivePedroConfig.maximumMotorPower = .25;
        HivePedroConfig.approachDistanceInches = 10;
        assertThrows(IllegalArgumentException.class, () -> new HivePedroController(drive, tracker, now::get));
    }

    @Test public void failedPathStartStillZerosOutputsAndRestoresPowerLimit() {
        when(drive.followPath(any())).thenReturn(Command.build().setStart(() -> { throw new IllegalStateException("failed path"); }));
        assertThrows(IllegalStateException.class, () -> begin(50, 0));
        verify(drive).setMotorPowerLimit(1);
        verify(drive, atLeastOnce()).stop();
    }

    private void begin(double forward, double left) {
        for (int i = 0; i < 7; i++) tick(forward, left, false);
        control.beforeDrive(true, false);
        control.afterSensors();
    }

    private void tick(double forward, double left, boolean held) {
        now.addAndGet(50_000_000L);
        tracker.update(frame(++frameId, forward, left));
        control.beforeDrive(held, false);
        control.afterSensors();
    }

    /** Inverts the measured 15/3-inch, 45-degree camera so test inputs are physical robot coordinates. */
    private static LLResult frame(int id, double forward, double left) {
        double tx = Math.toDegrees(Math.atan(-left / ((forward + 12) / Math.sqrt(2))));
        double ty = Math.toDegrees(Math.atan((forward - 12) / (forward + 12)));
        DetectorResult detection = mock(DetectorResult.class);
        when(detection.getClassId()).thenReturn(0);
        when(detection.getClassName()).thenReturn("yellow_pollen");
        when(detection.getConfidence()).thenReturn(.9);
        when(detection.getTargetArea()).thenReturn(2.0);
        LLResult frame = mock(LLResult.class);
        when(frame.getPipelineIndex()).thenReturn(1);
        when(frame.isValid()).thenReturn(true);
        when(frame.getTimestamp()).thenReturn((double) id);
        when(frame.getDetectorResults()).thenReturn(Collections.singletonList(detection));
        try {
            when(frame.toString()).thenReturn(new JSONObject().put("Detector", new JSONArray().put(
                    new JSONObject().put("tx_nocross", tx).put("ty_nocross", ty))).toString());
        } catch (JSONException invalidFixture) { throw new AssertionError(invalidFixture); }
        return frame;
    }
}
