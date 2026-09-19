package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HiveTagAim;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class LauncherPrototypeTest {
    private final Launcher launcher = mock(Launcher.class);
    private final Limelight camera = mock(Limelight.class);
    private final HiveTagAim aim = mock(HiveTagAim.class);
    private final Drivetrain drive = mock(Drivetrain.class);
    private LauncherPrototype op;

    @Before public void setUp() {
        LauncherPrototype.targetTagId = 34;
        op = new LauncherPrototype(launcher, camera, aim, drive, mock(Telemetry.class), Alliance.RED);
        op.gamepad1 = mock(Gamepad.class);
        op.gamepad2 = mock(Gamepad.class);
    }

    @After public void tearDown() { LauncherPrototype.targetTagId = 34; }

    @Test public void heldControlsInInitDoNotEnableLauncherOrDrive() {
        op.gamepad1.right_trigger = 1;
        op.gamepad2.left_trigger = 1;
        op.init_loop();
        op.loop();
        verify(launcher, never()).enable();
        verify(launcher, never()).requestVelocity();
        verify(launcher, never()).applyPresetHood();
        verify(drive, never()).turnInPlace(anyDouble());
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void releaseStopsRunAndCancelRequiresReleasingPreviousRunRequest() {
        op.start();
        op.gamepad1.right_trigger = 1;
        op.loop();
        verify(launcher).requestVelocity();
        op.gamepad1.x = true;
        op.loop();
        verify(drive).stop();
        op.gamepad1.x = false;
        op.loop();
        verify(launcher, times(1)).requestVelocity();
        op.gamepad1.right_trigger = 0;
        op.loop();
        op.gamepad1.right_trigger = 1;
        op.loop();
        verify(launcher, times(2)).requestVelocity();
        op.gamepad1.right_trigger = 0;
        op.loop();
        verify(launcher, atLeastOnce()).idle();
    }

    @Test public void manualTurnOverridesAimAndNoTargetCommandsZeroTurn() {
        op.start();
        op.gamepad2.left_trigger = 1;
        when(aim.getTurnPower()).thenReturn(0.17);
        op.loop();
        verify(drive).turnInPlace(0.17);
        op.gamepad2.right_stick_x = 0.5f;
        op.gamepad2.left_stick_y = -0.25f;
        op.loop();
        verify(drive).arcadeDrive(0.25, 0, 0.5, Alliance.RED);
        op.gamepad2.right_stick_x = 0;
        when(aim.getTurnPower()).thenReturn(0.0);
        op.loop();
        verify(drive).turnInPlace(0);
    }

    @Test public void benchLoopDoesNotRequireAnyDriveHardware() {
        LauncherPrototype bench = new LauncherPrototype(launcher, camera, aim, null, mock(Telemetry.class), Alliance.RED);
        bench.gamepad1 = mock(Gamepad.class); bench.gamepad2 = mock(Gamepad.class);
        bench.init_loop(); bench.start(); bench.loop(); bench.stop();
        verify(launcher).stop();
        verify(camera).stop();
        verifyNoInteractions(drive);
    }

    @Test public void stopCleansUpOtherDevicesEvenWhenLauncherCleanupFailsAndCannotRestart() {
        op.start();
        doThrow(new IllegalStateException("motor disconnected")).when(launcher).stop();
        assertThrows(IllegalStateException.class, op::stop);
        verify(drive).stop(); verify(camera).stop(); verify(aim).clear();
        clearInvocations(launcher, drive, camera);
        op.start(); op.loop(); op.stop();
        verifyNoInteractions(launcher, drive, camera);
    }
}
