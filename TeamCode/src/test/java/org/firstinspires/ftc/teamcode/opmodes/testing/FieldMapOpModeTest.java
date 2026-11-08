package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.robot.triggers.Trigger;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class FieldMapOpModeTest {
    private final TestRobot robot = new TestRobot();
    private final Gamepad input = new Gamepad();
    private NextOpModeHarness<TestOp> runner;

    @Before public void setUp() {
        defaults();
        when(robot.drive.isLocalizationReady()).thenReturn(true);
        runner = new NextOpModeHarness<>(() -> new TestOp(robot));
        runner.init();
    }

    @After public void tearDown() {
        runner.close();
        Scheduler.reset();
        Trigger.Companion.getDefaultEventLoop().clear();
        defaults();
    }

    @Test public void panelsEditsOnlyApplyOnInitButtonPress() {
        FieldMap.startX = 30;
        FieldMap.startY = 42;
        FieldMap.startHeadingDegrees = 180;
        runner.initLoop();
        verify(robot.drive, never()).setStartingPose(any());
        input.a = true;
        runner.opMode.gamepad1.copy(input);
        runner.initLoop();
        ArgumentCaptor<Pose> pose = ArgumentCaptor.forClass(Pose.class);
        verify(robot.drive).setStartingPose(pose.capture());
        assertEquals(30, pose.getValue().x(), 0);
        assertEquals(42, pose.getValue().y(), 0);
        assertEquals(Math.PI, pose.getValue().heading(), 1e-8);
        runner.initLoop(); // held button does not repeatedly reset odometry
        verify(robot.drive, times(1)).setStartingPose(any());
        runner.start();
        FieldMap.startX = 80;
        input.x = true;
        runner.opMode.gamepad1.copy(input);
        runner.loop();
        verify(robot.drive, times(1)).setStartingPose(any());
        verify(robot.drive, never()).recalibrateLocalization();
    }

    @Test public void invalidOrUnreadyStartIsRejected() {
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1, 145}) {
            FieldMap.startX = invalid;
            pressA();
        }
        defaults();
        FieldMap.startHeadingDegrees = Double.NaN;
        pressA();
        defaults();
        when(robot.drive.isLocalizationReady()).thenReturn(false);
        pressA();
        verify(robot.drive, never()).setStartingPose(any());
    }

    @Test public void calibrationWinsOverSimultaneousPoseResetAndModeNeverDrives() {
        input.x = true;
        input.a = true;
        runner.opMode.gamepad1.copy(input);
        runner.initLoop();
        verify(robot.drive).recalibrateLocalization();
        verify(robot.drive, never()).setStartingPose(any());
        runner.start();
        runner.opMode.gamepad1.left_stick_y = -1;
        runner.opMode.gamepad1.right_stick_x = 1;
        runner.loop();
        verify(robot.drive).stop();
        verify(robot.drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
        verify(robot.drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
        verify(robot.drive, never()).followPath(any());
        assertEquals(Robot.HardwareProfile.DRIVE_ONLY, robot.profile);
    }

    private void pressA() {
        // copy() updates SDK edge detection just like a Driver Station packet.
        input.a = false;
        runner.opMode.gamepad1.copy(input);
        runner.initLoop();
        input.a = true;
        runner.opMode.gamepad1.copy(input);
        runner.initLoop();
    }

    private static void defaults() {
        FieldMap.startX = 72;
        FieldMap.startY = 24;
        FieldMap.startHeadingDegrees = 90;
    }

    private static final class TestOp extends FieldMap {
        TestOp(Robot robot) { super(robot); }
        @Override protected boolean isStartAllowed() { return true; }
    }
}
