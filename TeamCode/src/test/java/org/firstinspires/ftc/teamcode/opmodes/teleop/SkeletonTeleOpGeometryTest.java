package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.After;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class SkeletonTeleOpGeometryTest {
    private final Alliance previousAlliance = Alliance.current;
    private final boolean previousConfigured = FieldConstants.START_POSE_CONFIGURED;

    @After public void restoreConfiguration() {
        Alliance.current = previousAlliance;
        FieldConstants.START_POSE_CONFIGURED = previousConfigured;
    }

    @Test public void headingLockRotatesForBlueAndHeldButtonOnlyFiresOnce() {
        TestRobot robot = new TestRobot();
        try (NextOpModeHarness<Harness> runner = new NextOpModeHarness<>(() -> new Harness(robot))) {
            runner.init(); runner.start();
            Alliance.current = Alliance.RED;
            runner.opMode.gamepad1.square = true;
            runner.loop(); runner.loop();
            verify(robot.drive, times(1)).lockHeading(90);
            runner.opMode.gamepad1.square = false;
            runner.loop();
            Alliance.current = Alliance.BLUE;
            runner.opMode.gamepad1.square = true;
            runner.loop();
            verify(robot.drive).lockHeading(270);
        }
    }

    @Test public void operatorCannotInjectUnmeasuredStartPose() {
        TestRobot robot = new TestRobot();
        FieldConstants.START_POSE_CONFIGURED = false;
        try (NextOpModeHarness<Harness> runner = new NextOpModeHarness<>(() -> new Harness(robot))) {
            runner.init(); runner.start();
            runner.opMode.gamepad2.cross = true;
            runner.loop();
            verify(robot.drive, never()).setPose(any());
            verify(robot.output).addLine(contains("Corner reset disabled"));
        }
    }

    @Test public void configuredBlueResetUsesRotatedPositionAndHeading() {
        TestRobot robot = new TestRobot();
        FieldConstants.START_POSE_CONFIGURED = true;
        Alliance.current = Alliance.BLUE;
        try (NextOpModeHarness<Harness> runner = new NextOpModeHarness<>(() -> new Harness(robot))) {
            runner.init(); runner.start();
            runner.opMode.gamepad2.cross = true;
            runner.loop();
            ArgumentCaptor<Pose> pose = ArgumentCaptor.forClass(Pose.class);
            verify(robot.drive).setPose(pose.capture());
            assertEquals(144 - FieldConstants.RED_CORNER_START.x(), pose.getValue().x(), 1e-9);
            assertEquals(144 - FieldConstants.RED_CORNER_START.y(), pose.getValue().y(), 1e-9);
            assertEquals(3 * Math.PI / 2, pose.getValue().heading(), 1e-9);
        }
    }

    @Test public void buttonsHeldAtStartRequireReleaseBeforeActivating() {
        TestRobot robot = new TestRobot();
        try (NextOpModeHarness<Harness> runner = new NextOpModeHarness<>(() -> new Harness(robot))) {
            runner.init();
            runner.opMode.gamepad1.square = true;
            runner.initLoop();
            runner.start(); runner.loop();
            verify(robot.drive, never()).lockHeading(anyDouble());
            runner.opMode.gamepad1.square = false; runner.loop();
            runner.opMode.gamepad1.square = true; runner.loop();
            verify(robot.drive).lockHeading(anyDouble());
        }
    }

    private static class Harness extends SkeletonTeleOp {
        Harness(TestRobot robot) { super(robot); }
        @Override protected boolean isStartAllowed() { return true; }
    }
}
