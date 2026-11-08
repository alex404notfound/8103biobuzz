package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class MotorSafetyTest {
    @Test public void powerAndBrakeFailuresStillStopEveryRemainingMotor() {
        DcMotorEx first = mock(DcMotorEx.class), second = mock(DcMotorEx.class), third = mock(DcMotorEx.class);
        RuntimeException powerFailure = new IllegalStateException("First motor disconnected");
        RuntimeException brakeFailure = new IllegalStateException("Second motor brake failed");
        doThrow(powerFailure).when(first).setPower(0);
        doThrow(brakeFailure).when(second).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        assertSame(powerFailure, assertThrows(RuntimeException.class,
                () -> MotorSafety.stopAndBrake(first, second, null, third)));

        for (DcMotorEx motor : new DcMotorEx[]{first, second, third}) {
            verify(motor).setPower(0);
            verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        assertArrayEquals(new Throwable[]{brakeFailure}, powerFailure.getSuppressed());
    }
}
