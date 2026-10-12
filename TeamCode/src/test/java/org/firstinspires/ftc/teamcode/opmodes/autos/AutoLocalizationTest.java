package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.After;
import org.junit.Test;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class AutoLocalizationTest {
    @After public void reset() { Scheduler.reset(); }

    @Test public void startWithoutLocalizationSampleNeverBuildsSequence() {
        TestRobot robot = configuredRobot();
        try (NextOpModeHarness<TestAuto> runner = new NextOpModeHarness<>(() -> new TestAuto(robot))) {
            runner.init();
            runner.start();
            assertEquals(0, runner.opMode.builds);
            verify(robot.drive).stop();
        }
    }

    @Test public void untunedForesightDoesNotBuildOrScheduleAutonomous() {
        TestRobot robot = configuredRobot();
        when(robot.drive.isLocalizationReady()).thenReturn(true);
        when(robot.drive.isPathFollowingConfigured()).thenReturn(false);
        try (NextOpModeHarness<TestAuto> runner = new NextOpModeHarness<>(() -> new TestAuto(robot))) {
            runner.init();
            runner.start();
            assertEquals(0, runner.opMode.builds);
            assertFalse(runner.opMode.action.isScheduled());
            verify(robot.drive).stop();
        }
    }

    @Test public void knownFaultCancelsBeforeSequenceAdvancesAndRecoveryDoesNotRestart() {
        TestRobot robot = configuredRobot();
        when(robot.drive.isLocalizationReady()).thenReturn(true);
        try (NextOpModeHarness<TestAuto> runner = new NextOpModeHarness<>(() -> new TestAuto(robot))) {
            runner.init();
            runner.start();
            assertEquals(1, runner.opMode.builds);
            assertTrue(runner.opMode.action.isScheduled());
            when(robot.drive.isLocalizationReady()).thenReturn(false);
            runner.opMode.firstFinished = true;
            runner.loop();
            assertFalse(runner.opMode.action.isScheduled());
            assertEquals(0, runner.opMode.nextActions);
            verify(robot.drive).stop();
            when(robot.drive.isLocalizationReady()).thenReturn(true);
            runner.loop();
            assertEquals(1, runner.opMode.builds);
            assertEquals(0, runner.opMode.nextActions);
        }
    }

    @Test public void faultDiscoveredDuringNextFtcMechanismUpdateCannotAdvanceSequenceThatTick() {
        TestRobot robot = configuredRobot();
        when(robot.drive.isLocalizationReady()).thenReturn(true);
        try (NextOpModeHarness<TestAuto> runner = new NextOpModeHarness<>(() -> new TestAuto(robot))) {
            runner.init();
            runner.start();
            doAnswer(call -> {
                when(robot.drive.isLocalizationReady()).thenReturn(false);
                runner.opMode.firstFinished = true; // stopping the follower can look like completion
                return null;
            }).when(robot.drive).periodic();
            runner.loop();
            assertFalse(runner.opMode.action.isScheduled());
            assertEquals(0, runner.opMode.nextActions);
            verify(robot.drive).stop();
        }
    }

    private TestRobot configuredRobot() {
        TestRobot robot = new TestRobot();
        when(robot.drive.isPathFollowingConfigured()).thenReturn(true);
        return robot;
    }

    private static class TestAuto extends AutoOpMode {
        int builds;
        boolean firstFinished;
        int nextActions;
        final Command action = sequential(
                infinite(() -> { }).setDone(() -> firstFinished),
                instant(() -> nextActions++));
        TestAuto(TestRobot robot) { super(robot, Alliance.RED); }
        @Override protected boolean isStartAllowed() { return true; }
        @Override protected Pose startingPose() { return Pose.zero(); }
        @Override protected Command buildSequence() { builds++; return action; }
    }
}
