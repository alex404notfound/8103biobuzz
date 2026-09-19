package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.*;

public class ShooterFlywheelTuningTest {
    private final ShooterFlywheel flywheel = mock(ShooterFlywheel.class);
    private ShooterFlywheelTuning op;

    @Before public void setUp() {
        defaults();
        op = new ShooterFlywheelTuning(flywheel, mock(Telemetry.class));
        op.gamepad1 = mock(Gamepad.class);
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        ShooterFlywheelTuning.targetRpm = 1000;
        ShooterFlywheelTuning.characterizationVoltage = 1;
        ShooterFlywheelTuning.rpmStep = 100;
        ShooterFlywheelTuning.voltageStep = 0.1;
    }

    @Test public void heldStartControlsCannotRunUntilReleasedAndDashboardTargetStaysLive() {
        op.gamepad1.right_trigger = 1;
        op.init_loop(); op.loop();
        verify(flywheel, never()).enable();
        op.start(); op.loop();
        verify(flywheel, never()).requestRpm(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(flywheel).requestRpm(1000);
        ShooterFlywheelTuning.targetRpm = 1500;
        op.loop();
        verify(flywheel).requestRpm(1500);
    }

    @Test public void cancelRequiresReleaseBeforeOldHeldRequestCanResume() {
        arm();
        op.gamepad1.right_trigger = 1; op.loop();
        clearInvocations(flywheel);
        op.gamepad1.x = true; op.loop();
        verify(flywheel).idle();
        op.gamepad1.x = false; op.loop();
        verify(flywheel, never()).requestRpm(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(flywheel).requestRpm(1000);
    }

    @Test public void individualAndVoltageModesAreExclusiveAndReleaseCoasts() {
        arm();
        op.gamepad1.right_trigger = 1; // LB modes take priority over RPM.
        op.gamepad1.left_bumper = true;
        op.gamepad1.x = true; op.loop();
        verify(flywheel).requestMotorTest(true);
        op.gamepad1.x = false; op.gamepad1.y = true; op.loop();
        verify(flywheel).requestMotorTest(false);
        op.gamepad1.y = false; op.gamepad1.a = true; op.loop();
        verify(flywheel).requestVoltage(1);
        ShooterFlywheelTuning.characterizationVoltage = 2;
        op.loop(); verify(flywheel).requestVoltage(2);
        op.gamepad1.x = true; op.loop(); // Conflicting LB+A+X is idle.
        verify(flywheel).idle();
        verify(flywheel, never()).requestRpm(anyDouble());
        op.gamepad1.x = op.gamepad1.a = op.gamepad1.left_bumper = false;
        op.gamepad1.right_trigger = 0; op.loop();
        verify(flywheel, times(2)).idle();
    }

    @Test public void canceledDpadEdgesCannotApplyAfterRelease() {
        arm();
        when(op.gamepad1.dpadUpWasPressed()).thenReturn(true, false);
        op.gamepad1.x = true; op.loop();
        op.gamepad1.x = false; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(flywheel).requestRpm(1000);
    }

    @Test public void hardwareFailureStopsAndCannotRestartClosedOpMode() {
        arm();
        doThrow(new IllegalStateException("disconnected")).when(flywheel).periodic();
        assertThrows(IllegalStateException.class, op::loop);
        verify(flywheel).stop();
        clearInvocations(flywheel);
        op.start(); op.loop(); op.init_loop(); op.stop();
        verifyNoInteractions(flywheel);
    }

    private void arm() { op.start(); op.loop(); clearInvocations(flywheel); }
}
