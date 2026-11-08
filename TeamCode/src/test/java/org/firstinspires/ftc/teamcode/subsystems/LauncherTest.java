package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class LauncherTest {
    private final ShooterFlywheel big = mock(ShooterFlywheel.class), small = mock(ShooterFlywheel.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private Launcher launcher;

    @Before public void setUp() {
        defaults();
        launcher = new Launcher(big, small, telemetry);
    }
    @After public void tearDown() { defaults(); }
    private static void defaults() {
        Launcher.smallBallRpm = Launcher.largeBallRpm = 1000;
        Launcher.big = new ShooterFlywheel.Settings("launcherLeft");
        Launcher.small = new ShooterFlywheel.Settings("launcherRight");
    }

    @Test public void initReadsBothSensorsWithoutEnablingEitherMotor() {
        launcher.requestRpm(1000); launcher.requestMotorTest(); launcher.periodic();
        verify(big).periodic(); verify(small).periodic();
        verify(big, never()).enable(); verify(small, never()).enable();
        verify(big, never()).requestRpm(anyDouble()); verify(small, never()).requestRpm(anyDouble());
        verify(big, never()).requestMotorTest(); verify(small, never()).requestMotorTest();
    }

    @Test public void selectionStopsBothBeforeRoutingToTheOtherMotor() {
        launcher.enable();
        launcher.requestRpm(1500);
        launcher.selectBall(Launcher.Ball.LARGE_NECTAR);
        launcher.requestRpm(2500); launcher.requestMotorTest();
        InOrder order = inOrder(big, small);
        order.verify(small).requestRpm(1500);
        order.verify(big).idle(); order.verify(small).idle();
        order.verify(big).requestRpm(2500); order.verify(big).requestMotorTest();
        verify(big, never()).requestRpm(1500); verify(small, never()).requestRpm(2500);
        verify(small, never()).requestMotorTest();
    }

    @Test public void presetsAndRpmAdjustmentsRemainIndependentAndBounded() {
        launcher.adjustRpm(200);
        assertEquals(1200, Launcher.smallBallRpm, 0);
        launcher.selectBall(Launcher.Ball.LARGE_NECTAR);
        assertEquals(1000, launcher.getManualRpm(), 0);
        launcher.adjustRpm(10000);
        assertEquals(6000, Launcher.largeBallRpm, 0);
        launcher.adjustRpm(-10000);
        assertEquals(0, Launcher.largeBallRpm, 0);
        launcher.adjustRpm(Double.NaN);
        assertEquals(0, Launcher.largeBallRpm, 0);
        assertEquals(1200, Launcher.smallBallRpm, 0);
    }

    @Test public void readinessAndTelemetryReferToEachShootersOwnFeedback() {
        when(big.isAtSpeed()).thenReturn(true); when(small.isAtSpeed()).thenReturn(false);
        when(big.getMeasuredRpm()).thenReturn(2500.0);
        when(small.getMeasuredRpm()).thenReturn(1200.0);
        assertFalse(launcher.isAtSpeed());
        launcher.enable(); assertFalse(launcher.isAtSpeed());
        launcher.selectBall(Launcher.Ball.LARGE_NECTAR); assertTrue(launcher.isAtSpeed());
        assertEquals(2500, launcher.getMeasuredRpm(), 0);
        launcher.periodic();
        verify(telemetry).addData("shooter.big.measuredRpm", 2500.0);
        verify(telemetry).addData("shooter.small.measuredRpm", 1200.0);
    }

    @Test public void stopAndIdleAttemptBothMotorsWhenOneThrows() {
        launcher.enable();
        doThrow(new IllegalStateException("disconnected")).when(big).idle();
        assertThrows(IllegalStateException.class, launcher::idle);
        verify(small).idle();
        doThrow(new IllegalStateException("disconnected")).when(big).stop();
        assertThrows(IllegalStateException.class, launcher::stop);
        verify(small).stop();
        clearInvocations(big, small);
        launcher.enable(); launcher.requestRpm(1000); launcher.requestMotorTest();
        launcher.periodic(); launcher.idle(); launcher.stop();
        assertFalse(launcher.isAtSpeed());
        verifyNoInteractions(big, small);
    }

    @Test public void sensorExceptionShutsDownBothControllers() {
        launcher.enable();
        doThrow(new IllegalStateException("encoder disconnected")).when(small).periodic();
        assertThrows(IllegalStateException.class, launcher::periodic);
        verify(big).stop(); verify(small).stop();
    }

    @Test public void hardwareMappingRequiresTwoDistinctMotorsAndNoAdditionalMechanism() {
        HardwareMap map = mock(HardwareMap.class);
        DcMotorEx bigMotor = mock(DcMotorEx.class), smallMotor = mock(DcMotorEx.class);
        when(map.get(DcMotorEx.class, "launcherLeft")).thenReturn(bigMotor);
        when(map.get(DcMotorEx.class, "launcherRight")).thenReturn(smallMotor);
        new Launcher(map, telemetry).stop();
        verify(map).get(DcMotorEx.class, "launcherLeft");
        verify(map).get(DcMotorEx.class, "launcherRight");
        verifyNoMoreInteractions(map);
        when(map.get(DcMotorEx.class, "launcherRight")).thenReturn(bigMotor);
        assertThrows(IllegalArgumentException.class, () -> new Launcher(map, telemetry));
    }

    @Test public void realControllersUseSeparateEncodersGainsAndMotorOutputs() {
        DcMotorEx bigMotor = mock(DcMotorEx.class), smallMotor = mock(DcMotorEx.class);
        AtomicLong now = new AtomicLong();
        Launcher.big.directionVerified = Launcher.small.directionVerified = true;
        Launcher.big.kV = .001; Launcher.small.kV = .002;
        when(bigMotor.getVelocity()).thenReturn(2000.0 * 28 / 60);
        when(smallMotor.getVelocity()).thenReturn(1000.0 * 28 / 60);
        Launcher real = new Launcher(
                new ShooterFlywheel(bigMotor, Launcher.big, () -> 12, now::get),
                new ShooterFlywheel(smallMotor, Launcher.small, () -> 12, now::get), telemetry);
        real.enable(); real.requestRpm(1000); real.periodic();
        verify(smallMotor).setPower(2.0 / 12);
        verify(bigMotor, never()).setPower(doubleThat(p -> p > 0));
        real.selectBall(Launcher.Ball.LARGE_NECTAR);
        clearInvocations(bigMotor, smallMotor);
        real.requestRpm(2000); now.set(20_000_000L); real.periodic();
        verify(bigMotor).setPower(2.0 / 12);
        verify(smallMotor, never()).setPower(doubleThat(p -> p > 0));
        assertEquals(2000, real.getMeasuredRpm(), 1e-9);
    }
}
