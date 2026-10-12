package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import dev.nextftc.robot.triggers.Trigger;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.subsystems.HiveVisionConfig;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class HivePedroOpModeTest {
    private final TestRobot robot = new TestRobot();
    private final Limelight3A camera = mock(Limelight3A.class);
    private NextOpModeHarness<TestOp> runner;

    @Before public void setUp() {
        when(robot.map.get(Limelight3A.class, HiveVisionConfig.hardwareName)).thenReturn(camera);
        when(camera.pipelineSwitch(HiveVisionConfig.pipelineIndex)).thenReturn(true);
        when(robot.drive.getPose()).thenReturn(Pose.zero());
        when(robot.drive.getVelocity()).thenReturn(Velocity.zero());
        when(robot.drive.isLocalizationReady()).thenReturn(true);
        runner = new NextOpModeHarness<>(() -> new TestOp(robot));
    }

    @After public void tearDown() {
        runner.close();
        Scheduler.reset();
        Trigger.Companion.getDefaultEventLoop().clear();
    }

    @Test public void detectorHasOneOwnerAndInitDoesNotRequireOtherMechanismsOrMove() {
        runner.init();
        assertEquals(Robot.HardwareProfile.DRIVE_ONLY, robot.profile);
        assertNull(robot.limelight);
        assertNull(robot.exampleSubsystem);
        verify(robot.map).getAll(LynxModule.class);
        verify(robot.map).get(Limelight3A.class, HiveVisionConfig.hardwareName);
        verifyNoMoreInteractions(robot.map);
        runner.initLoop();
        verify(camera).getLatestResult();
        verify(robot.drive, never()).followPath(any());
        verify(robot.drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
    }

    @Test public void activeLoopReadsCameraBeforeUpdatingPedro() {
        runner.init();
        runner.start();
        runner.loop();
        org.mockito.InOrder order = inOrder(camera, robot.drive);
        order.verify(camera).getLatestResult();
        order.verify(robot.drive).periodic();
    }

    @Test public void stopInInitDoesNotStartDriveAndClosesTheCamera() {
        runner.init();
        runner.opMode.startAllowed = false;
        runner.start();
        runner.loop();
        verify(camera).stop();
        verify(robot.drive, never()).followPath(any());
        verify(robot.drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
        assertTrue(robot.getMechanisms().isEmpty());
    }

    @Test public void failedPipelineInitUsesNextFtcExceptionalCleanup() {
        when(camera.pipelineSwitch(HiveVisionConfig.pipelineIndex)).thenReturn(false);
        assertThrows(IllegalStateException.class, runner::init);
        runner.cleanupAfterFailure();
        verify(camera).stop();
        verify(camera, never()).start();
        verify(robot.drive, atLeastOnce()).stop();
    }

    @Test public void cameraFailureDuringAnActiveLoopStillStopsAndCleansUp() {
        runner.init();
        runner.start();
        when(camera.getLatestResult()).thenThrow(new IllegalStateException("disconnected"));
        assertThrows(IllegalStateException.class, runner::loop);
        runner.cleanupAfterFailure();
        verify(camera).stop();
        verify(robot.drive, atLeastOnce()).stop();
    }

    private static final class TestOp extends HivePedroTest {
        boolean startAllowed = true;
        TestOp(Robot robot) { super(robot); }
        @Override protected boolean isStartAllowed() { return startAllowed; }
    }
}
