package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class TimedParkAutoTest {
    private final AtomicLong now = new AtomicLong();
    private final HardwareMap map = mock(HardwareMap.class);
    private final TimedParkAuto auto = new TimedParkAuto(now::get);
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final int originalDuration = TimedParkAuto.moveMs;
    private final double originalForward = TimedParkAuto.forwardPower, originalLeft = TimedParkAuto.leftPower;

    @Before public void setUp() {
        TimedParkAuto.moveMs = 500;
        TimedParkAuto.forwardPower = 0.25;
        TimedParkAuto.leftPower = 0;
        String[] names = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (int i = 0; i < names.length; i++) {
            motors[i] = mock(DcMotorEx.class);
            when(map.get(DcMotorEx.class, names[i])).thenReturn(motors[i]);
        }
        auto.hardwareMap = map;
        auto.telemetry = mock(Telemetry.class);
    }

    @After public void restoreSettings() {
        TimedParkAuto.moveMs = originalDuration;
        TimedParkAuto.forwardPower = originalForward;
        TimedParkAuto.leftPower = originalLeft;
    }

    @Test public void oneMoveStartsOnStartAndBrakesAtTheInitConfiguredDeadline() {
        auto.init();
        for (DcMotorEx motor : motors) {
            verify(motor, never()).setPower(doubleThat(power -> power != 0));
            verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        TimedParkAuto.moveMs = 1000;
        TimedParkAuto.forwardPower = 0.7;
        auto.start();
        for (DcMotorEx motor : motors) verify(motor).setPower(0.25);
        clearInvocations((Object[]) motors);

        now.set(499_000_000L);
        auto.loop();
        verifyNoInteractions((Object[]) motors);
        now.set(500_000_000L);
        auto.loop();
        for (DcMotorEx motor : motors) {
            verify(motor).setPower(0);
            verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verify(motor, never()).getCurrentPosition();
        }
        verify(map, never()).get(eq(GoBildaPinpointDriver.class), anyString());
        clearInvocations((Object[]) motors);
        auto.loop();
        auto.start();
        verifyNoInteractions((Object[]) motors);
    }

    @Test public void stopBrakesAllWheelsBeforeTheMoveFinishes() {
        auto.init();
        auto.start();
        clearInvocations((Object[]) motors);
        auto.stop();
        auto.loop();
        for (DcMotorEx motor : motors) {
            verify(motor).setPower(0);
            verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Test public void invalidTimingOrPowerLeavesTheDriveStill() {
        TimedParkAuto.moveMs = 5001;
        auto.init();
        auto.start();
        auto.loop();
        TimedParkAuto.moveMs = 500;
        TimedParkAuto.leftPower = Double.NaN;
        auto.init();
        auto.start();
        auto.loop();
        for (DcMotorEx motor : motors) verify(motor, never()).setPower(doubleThat(power -> power != 0));
    }
}
