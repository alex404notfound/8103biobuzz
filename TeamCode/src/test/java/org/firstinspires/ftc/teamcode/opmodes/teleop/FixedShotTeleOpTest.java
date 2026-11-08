package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import java.lang.reflect.Field;
import java.util.Collections;

import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class FixedShotTeleOpTest {
    private final HardwareMap map = mock(HardwareMap.class);
    private final Gamepad packet = new Gamepad();
    private final FixedShotTeleOp.WedgeSettings originalFront = FixedShotTeleOp.frontWedge;
    private final FixedShotTeleOp.WedgeSettings originalBack = FixedShotTeleOp.backWedge;
    private final double originalSetpoint = ShooterFlywheel.SMALL_SHOOTER.setpointRpm;
    private FixedShotTeleOp op;
    private DcMotorEx frontLeft, shooter, transfer, intake;
    private Servo stopper, frontWedge, backWedge;

    @Before public void setUp() throws ReflectiveOperationException {
        ShooterFlywheel.SMALL_SHOOTER.setpointRpm = 0;
        FixedShotTeleOp.frontWedge = wedge(0.2, 0.7);
        FixedShotTeleOp.backWedge = wedge(0.1, 0.9);
        frontLeft = device(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        device(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        device(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        device(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
        shooter = device(DcMotorEx.class, "smallShooter");
        transfer = device(DcMotorEx.class, Constants.TRANSFER_MOTOR_NAME);
        intake = device(DcMotorEx.class, "intake");
        stopper = device(Servo.class, "shotStopper");
        frontWedge = device(Servo.class, "frontWedge");
        backWedge = device(Servo.class, "backWedge");
        VoltageSensor battery = mock(VoltageSensor.class);
        when(battery.getVoltage()).thenReturn(12.0);
        @SuppressWarnings("unchecked")
        HardwareMap.DeviceMapping<VoltageSensor> sensors = mock(HardwareMap.DeviceMapping.class);
        when(sensors.iterator()).thenAnswer(call -> Collections.singletonList(battery).iterator());
        Field voltageSensor = HardwareMap.class.getField("voltageSensor");
        voltageSensor.setAccessible(true);
        voltageSensor.set(map, sensors);
        op = new FixedShotTeleOp();
        op.hardwareMap = map;
        op.telemetry = mock(Telemetry.class);
        op.gamepad1 = new Gamepad();
    }

    @After public void tearDown() {
        try { op.stop(); }
        finally {
            FixedShotTeleOp.frontWedge = originalFront;
            FixedShotTeleOp.backWedge = originalBack;
            ShooterFlywheel.SMALL_SHOOTER.setpointRpm = originalSetpoint;
        }
    }

    @Test public void closesStopperAndDrivesWithOnlyTheSmallBallHardware() {
        initAndStart();
        verify(stopper).setPosition(0.75);
        packet.left_stick_y = -0.4f;
        loop();
        verify(frontLeft).setPower(doubleThat(power -> power > 0));
        verify(map, never()).get(eq(GoBildaPinpointDriver.class), anyString());
        verify(map, never()).get(eq(DcMotorEx.class), eq(Constants.BIG_SHOOTER_MOTOR_NAME));
        verify(transfer, never()).setPower(doubleThat(power -> power != 0));
        verify(shooter, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void shotOpensAndFeedsOnlyWithinThreeHundredRpmWhileAHeld() {
        initAndStart();
        clearInvocations(shooter, transfer, stopper);
        packet.a = true;
        packet.left_stick_y = -0.4f;
        rpm(2699);
        loop();
        rpm(2700);
        loop();
        rpm(3301);
        loop();
        rpm(3300);
        loop();
        packet.a = false;
        loop();

        InOrder outputs = inOrder(stopper, transfer);
        outputs.verify(stopper).setPosition(0.75);
        outputs.verify(transfer).setPower(0);
        outputs.verify(stopper).setPosition(0.60);
        outputs.verify(transfer).setPower(FixedShotTeleOp.transferPower);
        outputs.verify(stopper).setPosition(0.75);
        outputs.verify(transfer).setPower(0);
        outputs.verify(stopper).setPosition(0.60);
        outputs.verify(transfer).setPower(FixedShotTeleOp.transferPower);
        outputs.verify(stopper).setPosition(0.75);
        outputs.verify(transfer).setPower(0);
        outputs.verifyNoMoreInteractions();
        verify(shooter, atLeastOnce()).setPower(doubleThat(power -> power > 0));
    }

    @Test public void cancelClosesTheStopperAndClearsTheShotAndMotorToggles() {
        initAndStart();
        packet.a = packet.b = packet.y = true;
        rpm(3000);
        loop();
        verify(stopper).setPosition(0.60);
        clearInvocations(intake, transfer, shooter, stopper);
        packet.b = packet.y = false;
        packet.x = true;
        loop();
        packet.x = false;
        loop();
        verify(stopper, times(2)).setPosition(0.75);
        verify(intake, times(2)).setPower(0);
        verify(transfer, times(2)).setPower(0);
        verify(shooter).setPower(0);
        verify(stopper, never()).setPosition(0.60);
    }

    @Test public void stopDuringShotStopsTheMotorsAndClosesTheStopper() {
        initAndStart();
        verify(shooter).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        packet.a = true;
        rpm(3000);
        loop();
        verify(transfer).setPower(FixedShotTeleOp.transferPower);
        clearInvocations(transfer, shooter, stopper, intake, frontLeft);
        op.stop();
        verify(transfer).setPower(0);
        verify(shooter).setPower(0);
        verify(shooter, never()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(intake).setPower(0);
        verify(frontLeft).setPower(0);
        verify(stopper).setPosition(0.75);
    }

    @Test public void bumpersAndMotorButtonsToggleIndependently() {
        packet.left_bumper = true;
        initAndStart();
        verify(frontWedge).setPosition(0.2);
        verify(backWedge).setPosition(0.1);
        verify(frontWedge, never()).setPosition(0.7);
        packet.left_bumper = false;
        loop();
        packet.left_bumper = true;
        packet.b = true;
        loop();
        verify(frontWedge).setPosition(0.7);
        verify(backWedge, never()).setPosition(0.9);
        verify(intake).setPower(FixedShotTeleOp.intakePower);
        verify(transfer, never()).setPower(doubleThat(power -> power != 0));
        clearInvocations(intake);
        packet.b = false;
        packet.y = true;
        packet.right_bumper = true;
        loop();
        verify(intake).setPower(FixedShotTeleOp.intakePower);
        verify(transfer).setPower(FixedShotTeleOp.transferPower);
        verify(backWedge).setPosition(0.9);
        verify(frontWedge).setPosition(0.7);
        packet.y = false;
        packet.left_bumper = false;
        loop();
        clearInvocations(intake, transfer);
        packet.b = packet.y = packet.left_bumper = true;
        loop();
        verify(intake).setPower(0);
        verify(transfer).setPower(0);
        verify(frontWedge, times(2)).setPosition(0.2);
        verify(backWedge).setPosition(0.1);
    }

    @Test public void anAliasedServoFailsInitBeforeMovingTheStopper() {
        when(map.get(Servo.class, "backWedge")).thenReturn(stopper);
        assertThrows(IllegalArgumentException.class, op::init);
        verifyNoInteractions(stopper);
    }

    private void initAndStart() {
        op.init();
        op.gamepad1.copy(packet);
        op.start();
    }

    private void loop() {
        op.gamepad1.copy(packet);
        op.loop();
    }

    private void rpm(double value) {
        when(shooter.getVelocity()).thenReturn(value * 28 / 60);
    }

    private <T> T device(Class<T> type, String name) {
        T device = mock(type);
        when(map.get(type, name)).thenReturn(device);
        return device;
    }

    private static FixedShotTeleOp.WedgeSettings wedge(double stowed, double deployed) {
        FixedShotTeleOp.WedgeSettings settings = new FixedShotTeleOp.WedgeSettings();
        settings.stowedPosition = stowed;
        settings.deployedPosition = deployed;
        return settings;
    }
}
