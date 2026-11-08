package org.firstinspires.ftc.robotcontroller.external.utilities;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;
import org.mockito.InOrder;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class UtilityTestHardwareTest {
    @Test public void motorStopsBeforeNextMotorRunsInEitherDirectionIncludingWrap() {
        for (boolean right : new boolean[]{false, true}) {
            DcMotor first = mock(DcMotor.class), second = mock(DcMotor.class);
            UtilityTestHardware op = utility(first, second);
            Gamepad packet = runningPacket();
            tick(op, packet);
            clearInvocations(first, second);
            select(packet, right);
            tick(op, packet);
            InOrder order = inOrder(first, second);
            order.verify(first).setPower(0);
            order.verify(second).setPower(.5);
            assertEquals(1, op.numDevice);

            packet.dpad_left = packet.dpad_right = false;
            tick(op, packet);
            clearInvocations(first, second);
            select(packet, right);
            tick(op, packet);
            order = inOrder(second, first);
            order.verify(second).setPower(0);
            order.verify(first).setPower(.5);
            assertEquals(0, op.numDevice);
        }
    }

    @Test public void crServoStopsWhenSwitchingWithRunControlReleased() {
        for (boolean right : new boolean[]{false, true}) {
            CRServo servo = mock(CRServo.class);
            DcMotor motor = mock(DcMotor.class);
            UtilityTestHardware op = utility(servo, motor);
            Gamepad packet = runningPacket();
            tick(op, packet);
            verify(servo).setPower(.5);
            clearInvocations(servo, motor);
            packet.right_bumper = false;
            select(packet, right);
            tick(op, packet);
            verify(servo).setPower(0);
            verify(motor, never()).setPower(anyDouble());
            assertEquals(1, op.numDevice);
        }
    }

    @Test public void switchingAwayFromPositionalServoDoesNotMoveIt() {
        Servo servo = mock(Servo.class);
        UtilityTestHardware op = utility(servo, mock(DcMotor.class));
        Gamepad packet = new Gamepad();
        select(packet, true);
        tick(op, packet);
        verifyNoInteractions(servo);
        assertEquals(1, op.numDevice);
    }

    @Test public void failedStopDoesNotSelectOrPowerTheNextActuator() {
        DcMotor first = mock(DcMotor.class), second = mock(DcMotor.class);
        UtilityTestHardware op = utility(first, second);
        Gamepad packet = runningPacket();
        tick(op, packet);
        doThrow(new IllegalStateException("stop failed")).when(first).setPower(0);
        select(packet, true);
        assertThrows(IllegalStateException.class, () -> tick(op, packet));
        assertEquals(0, op.numDevice);
        verify(second, never()).setPower(anyDouble());
    }

    private static UtilityTestHardware utility(HardwareDevice first, HardwareDevice second) {
        UtilityTestHardware op = new UtilityTestHardware();
        op.telemetry = mock(Telemetry.class);
        op.gamepad1 = new Gamepad();
        op.deviceList.add(new UtilityTestHardware.Device("first", first));
        op.deviceList.add(new UtilityTestHardware.Device("second", second));
        return op;
    }

    private static Gamepad runningPacket() {
        Gamepad packet = new Gamepad();
        packet.right_bumper = true;
        packet.left_stick_y = -.5f;
        return packet;
    }

    private static void select(Gamepad packet, boolean right) {
        packet.dpad_right = right;
        packet.dpad_left = !right;
    }

    private static void tick(UtilityTestHardware op, Gamepad packet) {
        op.gamepad1.copy(packet);
        op.loop();
    }
}
