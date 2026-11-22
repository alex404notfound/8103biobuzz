package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.FixedShotTeleOp;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.*;

public class HardwareTuningTest {
    @Test public void panelsControlsDriveTheConfiguredLeftWheelsAndBothFlywheels() {
        HardwareTuning op = newTuner();
        DcMotorEx frontLeft = motor(op.hardwareMap, "frontRight");
        DcMotorEx backLeft = motor(op.hardwareMap, "frontLeft");
        DcMotorEx shooter = motor(op.hardwareMap, "smallShooter");
        DcMotorEx shooter2 = mock(DcMotorEx.class);
        when(op.hardwareMap.tryGet(DcMotorEx.class, "shooter2")).thenReturn(shooter2);

        HardwareTuning.shooter2Power = 0.8;
        try {
            op.init();
            verify(shooter2).setDirection(DcMotorSimple.Direction.FORWARD);
            assertEquals(0, HardwareTuning.shooter2Power, 0);
            HardwareTuning.frontLeftPower = 0.1;
            HardwareTuning.backLeftPower = 0.2;
            HardwareTuning.shooterPower = 0.3;
            HardwareTuning.shooter2Power = 0.4;
            op.loop();

            verify(frontLeft).setPower(0.1);
            verify(backLeft).setPower(0.2);
            verify(shooter).setPower(0.3);
            verify(shooter2).setPower(0.4);
            clearInvocations(shooter2);
        } finally {
            op.stop();
        }
        verify(shooter2).setPower(0);
        assertEquals(0, HardwareTuning.shooter2Power, 0);
    }

    @Test public void positionalTurretMovesOnlyWithinTheConfiguredTravelLimits() {
        HardwareTuning op = newTuner();
        motor(op.hardwareMap, "frontRight");
        motor(op.hardwareMap, "frontLeft");
        motor(op.hardwareMap, "smallShooter");
        Servo turret = mock(Servo.class);
        when(op.hardwareMap.tryGet(Servo.class, "turret")).thenReturn(turret);
        double originalMin = FixedShotTeleOp.turretMinPosition;
        double originalMax = FixedShotTeleOp.turretMaxPosition;
        FixedShotTeleOp.turretMinPosition = 0.4;
        FixedShotTeleOp.turretMaxPosition = 0.6;
        try {
            op.init();
            op.loop();
            HardwareTuning.turretPosition = 0.8;
            op.loop();
            verify(turret, never()).setPosition(anyDouble());

            HardwareTuning.turretPosition = 0.5;
            op.loop();
            verify(turret).setPosition(0.5);
        } finally {
            op.stop();
            HardwareTuning.turretPosition = -1;
            FixedShotTeleOp.turretMinPosition = originalMin;
            FixedShotTeleOp.turretMaxPosition = originalMax;
        }
    }

    private static HardwareTuning newTuner() {
        HardwareTuning op = new HardwareTuning();
        op.hardwareMap = mock(HardwareMap.class);
        op.telemetry = mock(Telemetry.class);
        motor(op.hardwareMap, "backLeft");
        motor(op.hardwareMap, "backRight");
        motor(op.hardwareMap, "intake");
        return op;
    }

    private static DcMotorEx motor(HardwareMap hardwareMap, String name) {
        DcMotorEx motor = mock(DcMotorEx.class);
        when(hardwareMap.get(DcMotorEx.class, name)).thenReturn(motor);
        return motor;
    }
}
