package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

/** Launcher facade tests; encoder, voltage, PID and coupled-motor behavior live in ShooterFlywheelTest. */
public class LauncherTest {
    private final ShooterFlywheel flywheel = mock(ShooterFlywheel.class);
    private final Servo hood = mock(Servo.class);
    private final AtomicLong now = new AtomicLong();
    private Launcher launcher;

    @Before public void setUp() {
        defaults();
        when(flywheel.getMode()).thenReturn(ShooterFlywheel.Mode.OFF);
        when(flywheel.hasFreshSample()).thenReturn(true);
        launcher = new Launcher(flywheel, hood, mock(Telemetry.class), now::get);
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        Launcher.smallBallRpm = Launcher.largeBallRpm = 1000;
        Launcher.hoodMin = Launcher.hoodMax = Launcher.smallBallHood = Launcher.largeBallHood = Launcher.calibrationHood = -1;
        Launcher.hoodSettleMs = 300;
    }

    @Test public void initReadsSensorsWithoutEnablingMotionOrWritingHood() {
        calibratedHood();
        launcher.requestVelocity();
        launcher.requestMotorTest(true);
        assertFalse(launcher.applyPresetHood());
        launcher.periodic();
        verify(flywheel).periodic();
        verify(flywheel, never()).enable();
        verify(flywheel, never()).requestRpm(anyDouble());
        verify(flywheel, never()).requestMotorTest(anyBoolean());
        verifyNoInteractions(hood);
    }

    @Test public void motorRequestsDelegateToTheSameControllerUsedForTuning() {
        launcher.enable();
        verify(flywheel).enable();
        launcher.requestMotorTest(true);
        launcher.requestMotorTest(false);
        verify(flywheel).requestMotorTest(true);
        verify(flywheel).requestMotorTest(false);
        launcher.requestVelocity();
        verify(flywheel).requestRpm(1000);
        launcher.idle();
        verify(flywheel).idle();
        when(flywheel.getMode()).thenReturn(ShooterFlywheel.Mode.FAULT);
        when(flywheel.getStatus()).thenReturn("Encoder disagreement");
        assertEquals(Launcher.Mode.FAULT, launcher.getMode());
        assertEquals("Encoder disagreement", launcher.getStatus());
    }

    @Test public void runningPresetEditsRefreshTargetAndImmediatelyInvalidateOldReady() {
        launcher.enable();
        when(flywheel.getMode()).thenReturn(ShooterFlywheel.Mode.VELOCITY);
        when(flywheel.getTargetRpm()).thenReturn(1000.0);
        when(flywheel.isAtSpeed()).thenReturn(true);
        assertTrue(launcher.isAtSpeed());
        Launcher.smallBallRpm = 2500;
        assertFalse(launcher.isAtSpeed());
        launcher.periodic();
        verify(flywheel).requestRpm(2500);
        when(flywheel.getTargetRpm()).thenReturn(2500.0);
        assertTrue(launcher.isAtSpeed());
        when(flywheel.isAtSpeed()).thenReturn(false);
        assertFalse(launcher.isAtSpeed());
    }

    @Test public void readyRequiresFlywheelReadyCorrectHoodAndSettlingTime() {
        calibratedHood();
        launcher.enable();
        assertTrue(launcher.applyPresetHood());
        when(flywheel.getMode()).thenReturn(ShooterFlywheel.Mode.VELOCITY);
        when(flywheel.getTargetRpm()).thenReturn(1000.0);
        when(flywheel.isAtSpeed()).thenReturn(true);
        assertFalse(launcher.isReady());
        now.set(300_000_000L);
        assertTrue(launcher.isReady());
        Launcher.smallBallHood = 0.5;
        assertFalse(launcher.isReady());
        Launcher.smallBallHood = 0.4;
        Launcher.hoodSettleMs = Double.NaN;
        assertFalse(launcher.isReady());
    }

    @Test public void selectingAnotherBallStopsFlywheelAndRequiresItsHoodPreset() {
        calibratedHood();
        Launcher.largeBallRpm = 2000;
        launcher.enable();
        assertTrue(launcher.applyPresetHood());
        launcher.selectBall(Launcher.Ball.LARGE_NECTAR);
        verify(flywheel).idle();
        assertEquals(2000, launcher.getTargetRpm(), 0);
        assertEquals(0.4, launcher.getCommandedHood(), 0);
        assertFalse(launcher.isReady());
        assertTrue(launcher.applyPresetHood());
        verify(hood).setPosition(0.6);
        launcher.requestVelocity();
        verify(flywheel).requestRpm(2000);
        launcher.adjustRpm(10000);
        assertEquals(6000, launcher.getTargetRpm(), 0);
        launcher.adjustRpm(-10000);
        assertEquals(0, launcher.getTargetRpm(), 0);
    }

    @Test public void hoodRejectsUnmeasuredOutOfRangeMovingStaleAndNonfiniteStates() {
        launcher.enable();
        assertFalse(launcher.applyPresetHood());
        calibratedHood();
        assertFalse(launcher.positionHood(0.9));
        assertFalse(launcher.positionHood(Double.NaN));
        when(flywheel.getLeftRpm()).thenReturn(151.0);
        assertFalse(launcher.applyPresetHood());
        when(flywheel.getLeftRpm()).thenReturn(0.0);
        when(flywheel.getRightRpm()).thenReturn(Double.NaN);
        assertFalse(launcher.applyPresetHood());
        when(flywheel.getRightRpm()).thenReturn(0.0);
        when(flywheel.hasFreshSample()).thenReturn(false);
        assertFalse(launcher.applyPresetHood());
        when(flywheel.hasFreshSample()).thenReturn(true);
        when(flywheel.getMode()).thenReturn(ShooterFlywheel.Mode.VOLTAGE);
        assertFalse(launcher.applyPresetHood());
        verifyNoInteractions(hood);
        when(flywheel.getMode()).thenReturn(ShooterFlywheel.Mode.OFF);
        assertTrue(launcher.applyPresetHood());
        assertTrue(launcher.jogHood(1));
        verify(hood).setPosition(0.4);
        verify(hood).setPosition(0.7);
    }

    @Test public void stopIsTerminalEvenIfMotorCleanupThrowsAndNeverRepositionsHood() {
        launcher.enable();
        doThrow(new IllegalStateException("disconnected")).when(flywheel).stop();
        assertThrows(IllegalStateException.class, launcher::stop);
        clearInvocations(flywheel);
        launcher.enable();
        launcher.requestVelocity();
        launcher.requestMotorTest(true);
        launcher.periodic();
        launcher.idle();
        assertFalse(launcher.applyPresetHood());
        assertEquals(Launcher.Mode.OFF, launcher.getMode());
        verifyNoInteractions(flywheel, hood);
    }

    private void calibratedHood() {
        Launcher.hoodMin = 0.3; Launcher.hoodMax = 0.7;
        Launcher.smallBallHood = 0.4; Launcher.largeBallHood = 0.6;
    }
}
