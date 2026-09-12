package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.junit.After;
import org.junit.Test;
import java.lang.reflect.Field;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class AutoLocalizationTest {
    @After public void reset() { Scheduler.reset(); }

    @Test public void startWithoutLocalizationSampleNeverBuildsSequence() throws Exception {
        TestAuto auto = new TestAuto();
        auto.init();
        auto.start();
        assertEquals(0, auto.builds);
        verify(auto.testRobot).stop();
    }

    @Test public void faultCancelsSequenceBeforeItCanAdvanceAndRecoveryDoesNotRestart() throws Exception {
        TestAuto auto = new TestAuto();
        when(auto.drive.isLocalizationReady()).thenReturn(true);
        auto.init();
        auto.start();
        assertEquals(1, auto.builds);
        assertTrue(auto.action.isScheduled());
        when(auto.drive.isLocalizationReady()).thenReturn(false);
        auto.firstFinished = true;
        auto.loop();
        assertFalse(auto.action.isScheduled());
        assertEquals(0, auto.nextActions);
        verify(auto.testRobot).stop();
        when(auto.drive.isLocalizationReady()).thenReturn(true);
        auto.loop();
        assertEquals(1, auto.builds);
        assertEquals(0, auto.nextActions);
    }

    private static class TestAuto extends AutoOpMode {
        final Robot testRobot = mock(Robot.class);
        final Drivetrain drive = mock(Drivetrain.class);
        int builds;
        boolean firstFinished;
        int nextActions;
        final Command action = sequential(
                infinite(() -> { }).setDone(() -> firstFinished),
                instant(() -> nextActions++));
        TestAuto() throws Exception {
            super(Alliance.RED);
            setField("drivetrain", drive);
            setField("telemetry", mock(Telemetry.class));
        }
        private void setField(String name, Object value) throws Exception {
            Field field = Robot.class.getField(name);
            field.setAccessible(true);
            field.set(testRobot, value);
        }
        @Override protected Robot createRobot() { return testRobot; }
        @Override protected Pose startingPose() { return new Pose(); }
        @Override protected Command buildSequence() { builds++; return action; }
    }
}
