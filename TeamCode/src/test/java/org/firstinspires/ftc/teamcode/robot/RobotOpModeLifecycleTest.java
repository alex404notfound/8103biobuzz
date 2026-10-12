package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import dev.nextftc.robot.triggers.Trigger;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.After;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class RobotOpModeLifecycleTest {
    @After public void resetScheduler() {
        Scheduler.reset();
        Trigger.Companion.getDefaultEventLoop().clear();
    }

    @Test public void initSamplesWithoutRunningCommandsAndActiveLoopTicksEachOnce() {
        List<String> events = new ArrayList<>();
        TestRobot robot = robot(events);
        try (NextOpModeHarness<TestOpMode> runner = new NextOpModeHarness<>(() -> new TestOpMode(robot, events))) {
            runner.init();
            events.clear();
            runner.initLoop();
            assertEquals(Arrays.asList("clear", "init-input", "mechanism"), events);
            events.clear();
            runner.start();
            runner.loop();
            assertEquals(Arrays.asList("start", "clear", "input", "mechanism", "guard", "command"), events);
        }
        verify(robot.drive).stop();
    }

    @Test public void stopStillStopsHardwareAndClearsSchedulerWhenHookThrows() {
        TestRobot robot = robot(new ArrayList<>());
        NextOpModeHarness<TestOpMode> runner = new NextOpModeHarness<>(() -> new TestOpMode(robot, new ArrayList<>()));
        runner.init();
        runner.opMode.failOnStop = true;
        assertThrows(IllegalStateException.class, runner::close);
        verify(robot.drive).stop();
        assertFalse(runner.opMode.action.isScheduled());
    }

    @Test public void exceptionalLoopCleanupDoesNotDependOnEndBeingCalled() {
        TestRobot robot = robot(new ArrayList<>());
        NextOpModeHarness<TestOpMode> runner = new NextOpModeHarness<>(() -> new TestOpMode(robot, new ArrayList<>()));
        runner.init();
        runner.start();
        doThrow(new IllegalStateException("sensor failure")).when(robot.drive).periodic();
        assertThrows(IllegalStateException.class, runner::loop);
        runner.cleanupAfterFailure();
        verify(robot.drive).stop();
        assertFalse(runner.opMode.action.isScheduled());
        assertTrue(robot.getMechanisms().isEmpty());
    }

    @Test public void initializationRemovesCommandsAndBindingsLeftByAnEarlierOpMode() {
        Command previous = infinite(() -> fail("Old command executed"));
        previous.schedule();
        Trigger.Companion.getDefaultEventLoop().bind(() -> fail("Old binding executed"));
        TestRobot robot = robot(new ArrayList<>());
        try (NextOpModeHarness<TestOpMode> runner = new NextOpModeHarness<>(() -> new TestOpMode(robot, new ArrayList<>()))) {
            runner.init();
            runner.start();
            runner.loop();
            assertFalse(previous.isScheduled());
        }
    }

    @Test public void stopInInitCannotStartDefaultCommandsOrAutonomous() {
        TestRobot robot = robot(new ArrayList<>());
        List<String> events = new ArrayList<>();
        try (NextOpModeHarness<TestOpMode> runner = new NextOpModeHarness<>(() -> new TestOpMode(robot, events))) {
            runner.init();
            runner.opMode.startAllowed = false;
            runner.start(); // NextFTC 0.2.1 also reaches this when STOP ends INIT.
            assertFalse(events.contains("start"));
            verify(robot.drive, never()).getDefaultCommand();
            verify(robot.drive).stop();
            assertFalse(runner.opMode.action.isScheduled());
        }
    }

    @Test public void sharedRobotCanInitializeAgainWithoutKeepingStoppedStateOrOldBindings() {
        TestRobot robot = robot(new ArrayList<>());
        for (int i = 0; i < 2; i++) {
            try (NextOpModeHarness<TestOpMode> runner = new NextOpModeHarness<>(() -> new TestOpMode(robot, new ArrayList<>()))) {
                runner.init();
                assertFalse(robot.getMechanisms().isEmpty());
                runner.start();
                runner.loop();
            }
        }
        verify(robot.drive, times(2)).periodic();
        verify(robot.drive, times(2)).stop();
    }

    private TestRobot robot(List<String> events) {
        TestRobot robot = spy(new TestRobot());
        doAnswer(call -> { events.add("clear"); return null; }).when(robot).clearBulkCache();
        doAnswer(call -> { events.add("mechanism"); return null; }).when(robot.drive).periodic();
        return robot;
    }

    private static class TestOpMode extends RobotOpMode {
        final List<String> events;
        Command action;
        boolean failOnStop;
        boolean startAllowed = true;

        TestOpMode(Robot robot, List<String> events) { super(robot); this.events = events; }
        @Override protected boolean isStartAllowed() { return startAllowed; }
        @Override protected void onInit() {
            action = infinite(() -> events.add("command"));
            action.schedule();
        }
        @Override protected void onInitLoop() { events.add("init-input"); }
        @Override protected void onStart() { events.add("start"); }
        @Override protected void onLoop() { events.add("input"); }
        @Override protected void onAfterMechanisms() { events.add("guard"); }
        @Override protected void onStop() {
            if (failOnStop) throw new IllegalStateException("stop hook failed");
        }
    }
}
