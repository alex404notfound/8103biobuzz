package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class LimelightTest {
    private final AtomicLong now = new AtomicLong();
    private final Limelight3A camera = mock(Limelight3A.class);
    private final Drivetrain drive = mock(Drivetrain.class);
    private final ScheduledExecutorService executor = mock(ScheduledExecutorService.class);
    private Limelight vision;
    private Runnable writeHeading;
    private LLResult result;

    @Before public void setUp() {
        Scheduler.reset();
        Limelight.enableRelocalization = false;
        Limelight.frameOriginXInches = 0;
        Limelight.frameOriginYInches = 0;
        Limelight.frameRotationDegrees = 0;
        Limelight.maxStalenessMs = 100;
        OrientationPublisher publisher = new OrientationPublisher(() -> true, heading -> true, executor, now::get);
        ArgumentCaptor<Runnable> task = ArgumentCaptor.forClass(Runnable.class);
        verify(executor).scheduleWithFixedDelay(task.capture(), anyLong(), anyLong(), eq(TimeUnit.MILLISECONDS));
        writeHeading = task.getValue();
        writeHeading.run();
        vision = new Limelight(camera, drive, mock(Telemetry.class), 0, publisher, now::get);
        result = VisionResultGateTest.result(1, 0);
        when(result.getBotposeTagCount()).thenReturn(2);
        when(result.toString()).thenReturn("{\"botpose_orb\":[0.1,0.2,0,0,0,30]}");
        when(camera.getLatestResult()).thenReturn(result);
        when(drive.isLocalizationReady()).thenReturn(true);
        when(drive.getPose()).thenReturn(new Pose(0, 0, 0.7));
        when(drive.getVelocity()).thenReturn(new Pose());
        vision.periodic();
        writeHeading.run();
        sampleAt(50, 2);
        sampleAt(100, 3);
    }

    @After public void tearDown() { vision.stop(); Scheduler.reset(); Limelight.enableRelocalization = false; }

    @Test public void correctionIsDisabledUntilFieldCalibrationIsConfirmed() {
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void validStationaryCorrectionUsesMetersAndKeepsOdometryHeading() {
        Limelight.enableRelocalization = true;
        vision.relocalize().schedule();
        ArgumentCaptor<Pose> pose = ArgumentCaptor.forClass(Pose.class);
        verify(drive).applyVisionTranslation(pose.capture());
        assertEquals(0.1 * 100 / 2.54, pose.getValue().getX(), 1e-9);
        assertEquals(0.2 * 100 / 2.54, pose.getValue().getY(), 1e-9);
        assertEquals(0.7, pose.getValue().getHeading(), 1e-9);
    }

    @Test public void absentMegaTag2PayloadCannotBecomeAFalseFieldCenterPose() {
        Limelight.enableRelocalization = true;
        when(result.toString()).thenReturn("{}");
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void freshCameraFrameCannotUseAStaleOrientationTransmission() {
        Limelight.enableRelocalization = true;
        now.set(TimeUnit.MILLISECONDS.toNanos(300));
        when(result.getTimestamp()).thenReturn(2.0);
        vision.periodic();
        vision.relocalize().schedule();
        assertEquals("Heading feed is stale", vision.getLastCorrection());
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void justStoppedRobotRejectsAStillFreshFrameFromBeforeTheStop() {
        Limelight.enableRelocalization = true;
        when(drive.getVelocity()).thenReturn(new Pose(40, 0, 0));
        vision.periodic();
        now.set(TimeUnit.MILLISECONDS.toNanos(150));
        when(drive.getVelocity()).thenReturn(new Pose());
        vision.periodic();
        writeHeading.run();
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
        sampleAt(200, 4);
        sampleAt(250, 5);
        vision.relocalize().schedule();
        verify(drive).applyVisionTranslation(any());
    }

    @Test public void movingOrUnhealthyRobotRejectsCorrection() {
        Limelight.enableRelocalization = true;
        when(drive.getVelocity()).thenReturn(new Pose(5, 0, 0));
        vision.relocalize().schedule();
        when(drive.getVelocity()).thenReturn(new Pose());
        when(drive.isLocalizationReady()).thenReturn(false);
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void missingTagsLargeJumpsAndMalformedNumbersAreRejected() {
        Limelight.enableRelocalization = true;
        when(result.getBotposeTagCount()).thenReturn(0);
        vision.relocalize().schedule();
        when(result.getBotposeTagCount()).thenReturn(2);
        when(result.toString()).thenReturn("{\"botpose_orb\":[4,0,0,0,0,0]}");
        vision.relocalize().schedule();
        when(result.toString()).thenReturn("{\"botpose_orb\":[1e999,0,0,0,0,0]}");
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void unhealthySampleRestartsTheStationaryDwell() {
        now.set(TimeUnit.MILLISECONDS.toNanos(125));
        when(drive.isLocalizationReady()).thenReturn(false);
        vision.periodic();
        when(drive.isLocalizationReady()).thenReturn(true);
        sampleAt(150, 4);
        Limelight.enableRelocalization = true;
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void nonfiniteOdometryRestartsTheStationaryDwell() {
        when(drive.getVelocity()).thenReturn(new Pose(0, 0, Double.NaN));
        vision.periodic();
        when(drive.getVelocity()).thenReturn(new Pose());
        sampleAt(150, 4);
        Limelight.enableRelocalization = true;
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void longLoopGapCannotCountAsObservedStationaryTime() {
        sampleAt(251, 4);
        Limelight.enableRelocalization = true;
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void consumerRechecksMotionAndDiscardsTheOldStationaryWindow() {
        Limelight.enableRelocalization = true;
        when(drive.getVelocity()).thenReturn(new Pose(5, 0, 0));
        vision.relocalize().schedule();
        when(drive.getVelocity()).thenReturn(new Pose());
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    @Test public void timePassingWithoutNewStationarySamplesDoesNotCompleteDwell() {
        when(drive.getVelocity()).thenReturn(new Pose(5, 0, 0));
        vision.periodic();
        when(drive.getVelocity()).thenReturn(new Pose());
        sampleAt(150, 4);
        now.set(TimeUnit.MILLISECONDS.toNanos(250));
        Limelight.enableRelocalization = true;
        vision.relocalize().schedule();
        verify(drive, never()).applyVisionTranslation(any());
    }

    private void sampleAt(long milliseconds, double timestamp) {
        now.set(TimeUnit.MILLISECONDS.toNanos(milliseconds));
        when(result.getTimestamp()).thenReturn(timestamp);
        vision.periodic();
        writeHeading.run();
    }

}
