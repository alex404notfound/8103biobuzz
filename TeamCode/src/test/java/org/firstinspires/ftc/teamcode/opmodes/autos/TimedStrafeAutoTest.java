package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.mockito.Mockito.*;

public class TimedStrafeAutoTest {
    private final AtomicLong now = new AtomicLong();

    @Test public void strafesLeftAtHalfPowerForTwoSeconds() {
        verifyStrafe(new StrafeLeftTestAuto(now::get), -0.5);
    }

    @Test public void strafesRightAtHalfPowerForTwoSeconds() {
        verifyStrafe(new StrafeRightTestAuto(now::get), 0.5);
    }

    private void verifyStrafe(TimedDriveAuto auto, double frontLeftPower) {
        HardwareMap map = mock(HardwareMap.class);
        MecanumConfig config = Constants.drivetrainConfig;
        String[] names = {config.frontLeftName.get(), config.frontRightName.get(),
                config.backLeftName.get(), config.backRightName.get()};
        DcMotorEx[] motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            motors[i] = mock(DcMotorEx.class);
            when(map.get(DcMotorEx.class, names[i])).thenReturn(motors[i]);
        }
        auto.hardwareMap = map;
        auto.telemetry = mock(Telemetry.class);
        auto.init();
        for (DcMotorEx motor : motors) verify(motor, never()).setPower(doubleThat(power -> power != 0));

        now.set(1_000_000_000L);
        auto.start();
        verify(motors[0]).setPower(frontLeftPower);
        verify(motors[1]).setPower(-frontLeftPower);
        verify(motors[2]).setPower(-frontLeftPower);
        verify(motors[3]).setPower(frontLeftPower);
        clearInvocations((Object[]) motors);

        now.set(2_999_000_000L);
        auto.loop();
        verifyNoInteractions((Object[]) motors);

        now.set(3_000_000_000L);
        auto.loop();
        for (DcMotorEx motor : motors) {
            verify(motor).setPower(0);
            verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
