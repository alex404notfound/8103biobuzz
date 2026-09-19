package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.*;

public class AxonTurretOpModeTest {
    private final AxonTurret turret = mock(AxonTurret.class);
    private AxonTurretTest op;

    @Before public void setUp() {
        defaults();
        op = new AxonTurretTest(turret, mock(Telemetry.class));
        op.gamepad1 = mock(Gamepad.class);
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        AxonTurretTest.targetDegrees = 0; AxonTurretTest.targetStepDegrees = 5;
    }

    @Test public void initOnlyReadsAndHeldStartControlsRequireRelease() {
        op.gamepad1.right_trigger = 1;
        op.init_loop(); op.loop();
        verify(turret, never()).enable();
        verify(turret, never()).requestPosition(anyDouble());
        op.start(); op.loop();
        verify(turret, never()).requestPosition(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        AxonTurretTest.targetDegrees = 15;
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).requestPosition(15);
    }

    @Test public void cancelStopsImmediatelyAndOldHeldTriggerCannotResume() {
        arm();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).requestPosition(0);
        clearInvocations(turret);
        op.gamepad1.x = true; op.loop();
        verify(turret).idle();
        op.gamepad1.x = false; op.loop();
        verify(turret, never()).requestPosition(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).requestPosition(0);
    }

    @Test public void dashboardTargetEditsApplyWhileTheSameTriggerRemainsHeld() {
        arm();
        op.gamepad1.right_trigger = 1;
        AxonTurretTest.targetDegrees = 10;
        op.loop();
        verify(turret).requestPosition(10);
        clearInvocations(turret);

        AxonTurretTest.targetDegrees = -15;
        op.loop();
        verify(turret).requestPosition(-15);
        verify(turret, never()).requestPosition(10);
        verify(turret, never()).idle();
    }

    @Test public void manualOverridesPidAndReferenceOrFaultClearNeedsMotionReleased() {
        arm();
        // SDK edges are consumed once. A disallowed press must not queue a later action.
        when(op.gamepad1.bWasPressed()).thenReturn(true, false);
        when(op.gamepad1.yWasPressed()).thenReturn(true, false);
        op.gamepad1.left_bumper = true; op.gamepad1.left_stick_x = 0.5f;
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).requestManual(0.5);
        verify(turret, never()).requestPosition(anyDouble());
        verify(turret, never()).zeroHere(); verify(turret, never()).clearFault();
        op.gamepad1.left_bumper = false; op.gamepad1.left_stick_x = 0;
        op.gamepad1.right_trigger = 0; op.loop();
        verify(turret, never()).zeroHere(); verify(turret, never()).clearFault();
        when(op.gamepad1.bWasPressed()).thenReturn(true);
        when(op.gamepad1.yWasPressed()).thenReturn(true);
        op.loop();
        verify(turret).clearFault(); verify(turret).zeroHere();
    }

    @Test public void sensorFailureStopsAndStopCannotBeRestarted() {
        arm();
        doThrow(new IllegalStateException("encoder disconnected")).when(turret).periodic();
        assertThrows(IllegalStateException.class, op::loop);
        verify(turret).stop();
        clearInvocations(turret);
        op.start(); op.loop(); op.stop(); op.init_loop();
        verifyNoInteractions(turret);
    }

    private void arm() { op.start(); op.loop(); clearInvocations(turret); }
}
