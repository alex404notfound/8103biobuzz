package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.robot.triggers.Trigger;
import org.firstinspires.ftc.teamcode.pedroPathing.MarrowConfig;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.subsystems.HiveVisionConfig;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class HiveMarrowOpModeTest {
    private final Map<Field, Object> saved = new HashMap<>();
    private final TestRobot robot = new TestRobot();
    private final Limelight3A camera = mock(Limelight3A.class);
    private NextOpModeHarness<TestOp> runner;

    @Before public void setUp() throws Exception {
        for (Field field : MarrowConfig.class.getFields()) saved.put(field, field.get(null));
        MarrowConfig.geometryAndMapVerified = true;
        MarrowConfig.obstacleEnabled = false;
        MarrowConfig.robotLengthInches = MarrowConfig.robotWidthInches = 18;
        MarrowConfig.clearanceInches = 4;
        MarrowConfig.startX = 72;
        MarrowConfig.startY = 24;
        MarrowConfig.startHeadingDegrees = 90;
        when(robot.map.get(Limelight3A.class, HiveVisionConfig.hardwareName)).thenReturn(camera);
        when(camera.pipelineSwitch(HiveVisionConfig.pipelineIndex)).thenReturn(true);
        when(robot.drive.getPose()).thenReturn(new Pose(72, 24, Math.PI / 2));
        when(robot.drive.getVelocity()).thenReturn(Velocity.zero());
        when(robot.drive.isLocalizationReady()).thenReturn(true);
        when(robot.drive.isPathFollowingConfigured()).thenReturn(true);
        runner = new NextOpModeHarness<>(() -> new TestOp(robot));
    }

    @After public void tearDown() throws Exception {
        runner.close();
        Scheduler.reset();
        Trigger.Companion.getDefaultEventLoop().clear();
        for (Map.Entry<Field, Object> value : saved.entrySet()) value.getKey().set(null, value.getValue());
    }

    @Test public void usesMeasuredFieldPoseAndOneCameraWithoutAcquiringLauncherHardware() {
        runner.init();
        ArgumentCaptor<Pose> start = ArgumentCaptor.forClass(Pose.class);
        verify(robot.drive).setStartingPose(start.capture());
        assertEquals(72, start.getValue().x(), 0);
        assertEquals(24, start.getValue().y(), 0);
        assertEquals(Math.PI / 2, start.getValue().heading(), 1e-8);
        assertEquals(Robot.HardwareProfile.DRIVE_ONLY, robot.profile);
        assertNull(robot.limelight);
        assertNull(robot.exampleSubsystem);
        verify(robot.map).get(Limelight3A.class, HiveVisionConfig.hardwareName);
        runner.initLoop();
        verify(robot.drive, never()).followPath(any());
        verify(robot.drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
    }

    @Test public void unverifiedMapRefusesInitBeforeStartingTheCameraOrChangingPose() {
        MarrowConfig.geometryAndMapVerified = false;
        assertThrows(IllegalArgumentException.class, runner::init);
        runner.cleanupAfterFailure();
        verify(camera, never()).start();
        verify(robot.drive, never()).setStartingPose(any());
        verify(robot.drive, never()).followPath(any());
        verify(robot.drive, atLeastOnce()).stop();
    }

    @Test public void startTooCloseToWallAndInvalidObstacleAreRejectedBeforeCameraStart() {
        MarrowConfig.startX = 2;
        assertThrows(IllegalArgumentException.class, runner::init);
        verify(camera, never()).start();
        MarrowConfig.startX = 72;
        MarrowConfig.obstacleEnabled = true;
        MarrowConfig.obstacleRadiusInches = Double.NaN;
        assertThrows(IllegalArgumentException.class, MarrowConfig::createPlanner);
    }

    @Test public void holdingThroughStartWithoutAFreshConfirmedBallNeverMoves() {
        runner.init();
        runner.opMode.gamepad1.right_trigger = 1;
        runner.start();
        for (int i = 0; i < 5; i++) runner.loop();
        verify(robot.drive, never()).followPath(any());
        verify(robot.drive, never()).setMotorPowerLimit(anyDouble());
    }

    @Test public void stoppingInInitClosesCameraAndRemovesThePlanOverlay() {
        runner.init();
        runner.opMode.startAllowed = false;
        runner.start();
        verify(camera).stop();
        verify(robot.drive).setFieldOverlay(isNull());
        verify(robot.drive, never()).followPath(any());
    }

    private static final class TestOp extends HiveMarrowTest {
        boolean startAllowed = true;
        TestOp(Robot robot) { super(robot); }
        @Override protected boolean isStartAllowed() { return startAllowed; }
    }
}
