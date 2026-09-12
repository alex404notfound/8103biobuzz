package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.junit.After;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.*;

public class RobotOpModeLifecycleTest {
    @After public void resetScheduler() { Scheduler.reset(); }

    @Test public void eachCycleClearsSensorsBeforeInputAndFlushesAfterControl() {
        List<String> events = new ArrayList<>();
        TestOpMode opMode = new TestOpMode(events);
        opMode.init();
        events.clear();

        opMode.init_loop();
        assertEquals(Arrays.asList("clear", "init-input", "command", "periodic", "telemetry"), events);
        events.clear();

        opMode.start();
        opMode.loop();
        assertEquals(Arrays.asList("start", "clear", "input", "command", "periodic", "telemetry"), events);
        opMode.stop();
    }

    @Test public void stopStillStopsHardwareAndClearsSchedulerWhenHookThrows() {
        List<String> events = new ArrayList<>();
        TestOpMode opMode = new TestOpMode(events);
        opMode.init();
        opMode.failOnStop = true;

        assertThrows(IllegalStateException.class, opMode::stop);
        verify(opMode.testRobot).stop();
        assertFalse(opMode.action.isScheduled());
    }

    @Test public void initializationRemovesCommandsLeftByAnEarlierOpMode() {
        Command previous = infinite(() -> fail("Old command executed in new OpMode"));
        previous.schedule();
        TestOpMode opMode = new TestOpMode(new ArrayList<>());
        opMode.init();
        opMode.init_loop();
        assertFalse(previous.isScheduled());
        opMode.stop();
    }

    private static class TestOpMode extends RobotOpMode {
        final Robot testRobot = mock(Robot.class);
        final List<String> events;
        Command action;
        boolean failOnStop;

        TestOpMode(List<String> events) {
            this.events = events;
            doAnswer(call -> { events.add("clear"); return null; }).when(testRobot).clearBulkCache();
            doAnswer(call -> { events.add("periodic"); return null; }).when(testRobot).periodic();
            doAnswer(call -> { events.add("telemetry"); return null; }).when(testRobot).updateTelemetry(anyDouble());
        }

        @Override protected Robot createRobot() { return testRobot; }
        @Override protected void onInit() {
            action = infinite(() -> events.add("command"));
            action.schedule();
        }
        @Override protected void onInitLoop() { events.add("init-input"); }
        @Override protected void onStart() { events.add("start"); }
        @Override protected void onLoop() { events.add("input"); }
        @Override protected void onStop() {
            if (failOnStop) throw new IllegalStateException("stop hook failed");
        }
    }
}
