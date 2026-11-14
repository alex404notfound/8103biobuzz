package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class LauncherTest {
    private final ShooterFlywheel flywheel = mock(ShooterFlywheel.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private final ShooterFlywheel.Settings originalSettings = Launcher.shooter;
    private final double originalSmallRpm = Launcher.smallBallRpm, originalLargeRpm = Launcher.largeBallRpm;
    private Launcher launcher;

    @Before public void setUp() {
        Launcher.smallBallRpm = Launcher.largeBallRpm = 1000;
        Launcher.shooter = new ShooterFlywheel.Settings(Constants.SHOOTER_MOTOR_NAME);
        launcher = new Launcher(flywheel, telemetry);
    }

    @After public void tearDown() {
        Launcher.shooter = originalSettings;
        Launcher.smallBallRpm = originalSmallRpm;
        Launcher.largeBallRpm = originalLargeRpm;
    }

    @Test public void initReadsTheEncoderWithoutEnablingTheMotor() {
        launcher.requestRpm(1000); launcher.requestMotorTest(); launcher.periodic();
        verify(flywheel).periodic();
        verify(flywheel, never()).enable();
        verify(flywheel, never()).requestRpm(anyDouble());
        verify(flywheel, never()).requestMotorTest();
    }

    @Test public void ballSelectionStopsTheSameMotorBeforeUsingTheOtherPreset() {
        launcher.enable();
        launcher.requestRpm(1500);
        ShooterFlywheel pollenMotor = launcher.selected();
        launcher.selectBall(Launcher.Ball.LARGE_NECTAR);
        assertSame(pollenMotor, launcher.selected());
        launcher.requestRpm(2500); launcher.requestMotorTest();
        InOrder order = inOrder(flywheel);
        order.verify(flywheel).requestRpm(1500);
        order.verify(flywheel).idle();
        order.verify(flywheel).requestRpm(2500);
        order.verify(flywheel).requestMotorTest();
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

    @Test public void bothBallProfilesReadTheSingleEncoderAndTelemetryStream() {
        when(flywheel.isAtSpeed()).thenReturn(true);
        when(flywheel.getMeasuredRpm()).thenReturn(2500.0);
        assertFalse(launcher.isAtSpeed());
        launcher.enable(); assertTrue(launcher.isAtSpeed());
        for (Launcher.Ball ball : Launcher.Ball.values()) {
            launcher.selectBall(ball);
            assertEquals(2500, launcher.getMeasuredRpm(), 0);
            launcher.periodic();
        }
        verify(telemetry, times(2)).addData("shooter.measuredRpm", 2500.0);
    }

    @Test public void failedStopStillClosesTheLauncherAgainstLaterRequests() {
        launcher.enable();
        doThrow(new IllegalStateException("disconnected")).when(flywheel).stop();
        assertThrows(IllegalStateException.class, launcher::stop);
        clearInvocations(flywheel);
        launcher.enable(); launcher.requestRpm(1000); launcher.requestMotorTest();
        launcher.periodic(); launcher.idle(); launcher.stop();
        assertFalse(launcher.isAtSpeed());
        verifyNoInteractions(flywheel);
    }

    @Test public void sensorExceptionShutsDownTheController() {
        launcher.enable();
        doThrow(new IllegalStateException("encoder disconnected")).when(flywheel).periodic();
        assertThrows(IllegalStateException.class, launcher::periodic);
        verify(flywheel).stop();
        assertFalse(launcher.isAtSpeed());
    }

    @Test public void hardwareMappingRequiresOnlyOneShooterMotor() {
        HardwareMap map = mock(HardwareMap.class);
        DcMotorEx motor = mock(DcMotorEx.class);
        when(map.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME)).thenReturn(motor);
        new Launcher(map, telemetry).stop();
        verify(map).get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        verifyNoMoreInteractions(map);
        verify(motor, atLeastOnce()).setPower(0);
        verify(motor, never()).setPower(doubleThat(p -> p != 0));
    }

    @Test public void realControllerUsesTheSameMotorAndGainsAndResetsReadinessOnBallChange() {
        DcMotorEx motor = mock(DcMotorEx.class);
        AtomicLong now = new AtomicLong();
        Launcher.shooter.directionVerified = true;
        Launcher.shooter.kV = .002;
        Launcher.shooter.speedDwellMs = 0;
        when(motor.getVelocity()).thenReturn(1000.0 * 28 / 60);
        Launcher real = new Launcher(
                new ShooterFlywheel(motor, Launcher.shooter, () -> 12, now::get), telemetry);
        real.enable(); real.requestRpm(1000); real.periodic();
        verify(motor).setPower(2.0 / 12);
        assertTrue(real.isAtSpeed());
        real.selectBall(Launcher.Ball.LARGE_NECTAR);
        verify(motor, atLeastOnce()).setPower(0);
        assertFalse(real.isAtSpeed());
        assertEquals(0, real.getTargetRpm(), 0);
        clearInvocations(motor);
        when(motor.getVelocity()).thenReturn(1500.0 * 28 / 60);
        real.requestRpm(1500); now.set(20_000_000L); real.periodic();
        verify(motor).setPower(3.0 / 12);
        assertEquals(1500, real.getMeasuredRpm(), 1e-9);
        assertTrue(real.isAtSpeed());
    }
}
