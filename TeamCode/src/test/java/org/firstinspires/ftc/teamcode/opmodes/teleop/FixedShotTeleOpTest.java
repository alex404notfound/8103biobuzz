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
    private final FixedShotTeleOp.WedgeSettings originalWedge = FixedShotTeleOp.wedge;
    private final double originalSetpoint = ShooterFlywheel.SHOOTER.setpointRpm;
    private final double originalTurretMinPosition = FixedShotTeleOp.turretMinPosition;
    private final double originalTurretMaxPosition = FixedShotTeleOp.turretMaxPosition;
    private final double originalTurretPositionPerSecond = FixedShotTeleOp.turretPositionPerSecond;
    private final boolean originalTurretReversed = FixedShotTeleOp.turretReversed;
    private FixedShotTeleOp op;
    private DcMotorEx frontLeft, shooter, intake;
    private Servo stopper, wedge, turret;
    private double runtime;

    @Before public void setUp() throws ReflectiveOperationException {
        ShooterFlywheel.SHOOTER.setpointRpm = 0;
        FixedShotTeleOp.wedge = wedge(0.2, 0.7);
        FixedShotTeleOp.turretMinPosition = 0.40;
        FixedShotTeleOp.turretMaxPosition = 0.60;
        FixedShotTeleOp.turretPositionPerSecond = 0.25;
        FixedShotTeleOp.turretReversed = false;
        frontLeft = device(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        device(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        device(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        device(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
        shooter = device(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
        intake = device(DcMotorEx.class, Constants.INTAKE_MOTOR_NAME);
        stopper = device(Servo.class, "shotStopper");
        wedge = device(Servo.class, Constants.WEDGE_SERVO_NAME);
        turret = device(Servo.class, Constants.TURRET_SERVO_NAME);
        when(turret.getPosition()).thenReturn(0.50);
        VoltageSensor battery = mock(VoltageSensor.class);
        when(battery.getVoltage()).thenReturn(12.0);
        @SuppressWarnings("unchecked")
        HardwareMap.DeviceMapping<VoltageSensor> sensors = mock(HardwareMap.DeviceMapping.class);
        when(sensors.iterator()).thenAnswer(call -> Collections.singletonList(battery).iterator());
        Field voltageSensor = HardwareMap.class.getField("voltageSensor");
        voltageSensor.setAccessible(true);
        voltageSensor.set(map, sensors);
        op = new FixedShotTeleOp() {
            @Override public double getRuntime() { return runtime; }
        };
        op.hardwareMap = map;
        op.telemetry = mock(Telemetry.class);
        op.gamepad1 = new Gamepad();
    }

    @After public void tearDown() {
        try { op.stop(); }
        finally {
            FixedShotTeleOp.wedge = originalWedge;
            ShooterFlywheel.SHOOTER.setpointRpm = originalSetpoint;
            FixedShotTeleOp.turretMinPosition = originalTurretMinPosition;
            FixedShotTeleOp.turretMaxPosition = originalTurretMaxPosition;
            FixedShotTeleOp.turretPositionPerSecond = originalTurretPositionPerSecond;
            FixedShotTeleOp.turretReversed = originalTurretReversed;
        }
    }

    @Test public void closesStopperAndDrivesWithOneShooterAndOneWedge() {
        initAndStart();
        verify(stopper).setPosition(0.75);
        packet.left_stick_y = -0.4f;
        loop();
        verify(frontLeft).setPower(doubleThat(power -> power > 0));
        verify(map, never()).get(eq(GoBildaPinpointDriver.class), anyString());
        verify(map, never()).tryGet(Servo.class, "backWedge");
        verify(intake, never()).setPower(doubleThat(power -> power != 0));
        verify(shooter, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void shotOpensAndFeedsOnlyWithinThreeHundredRpmWhileCrossHeld() {
        initAndStart();
        clearInvocations(shooter, intake, stopper);
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

        InOrder outputs = inOrder(stopper, intake);
        outputs.verify(stopper).setPosition(0.75);
        outputs.verify(intake).setPower(0);
        outputs.verify(stopper).setPosition(0.60);
        outputs.verify(intake).setPower(FixedShotTeleOp.intakePower);
        outputs.verify(stopper).setPosition(0.75);
        outputs.verify(intake).setPower(0);
        outputs.verify(stopper).setPosition(0.60);
        outputs.verify(intake).setPower(FixedShotTeleOp.intakePower);
        outputs.verify(stopper).setPosition(0.75);
        outputs.verify(intake).setPower(0);
        outputs.verifyNoMoreInteractions();
        verify(shooter, atLeastOnce()).setPower(doubleThat(power -> power > 0));
    }

    @Test public void manualIntakeYieldsToTheShotSpeedGateUntilCrossIsReleased() {
        initAndStart();
        clearInvocations(intake);
        packet.b = true;
        loop();

        packet.b = false;
        packet.a = true;
        rpm(2600);
        loop();
        rpm(3000);
        loop();
        rpm(2600);
        loop();
        packet.a = false;
        loop();

        InOrder output = inOrder(intake);
        output.verify(intake).setPower(FixedShotTeleOp.intakePower);
        output.verify(intake).setPower(0);
        output.verify(intake).setPower(FixedShotTeleOp.intakePower);
        output.verify(intake).setPower(0);
        output.verify(intake).setPower(FixedShotTeleOp.intakePower);
        output.verifyNoMoreInteractions();
    }

    @Test public void cancelClosesTheStopperAndClearsTheShotAndIntakeToggle() {
        initAndStart();
        packet.a = packet.b = true;
        rpm(3000);
        loop();
        verify(stopper).setPosition(0.60);
        clearInvocations(intake, shooter, stopper);
        packet.b = false;
        packet.x = true;
        loop();
        packet.x = false;
        loop();
        verify(stopper, times(2)).setPosition(0.75);
        verify(intake, times(2)).setPower(0);
        verify(shooter).setPower(0);
        verify(stopper, never()).setPosition(0.60);
    }

    @Test public void stopDuringShotStopsTheMotorsAndClosesTheStopper() {
        initAndStart();
        verify(shooter).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        packet.a = true;
        rpm(3000);
        loop();
        verify(intake).setPower(FixedShotTeleOp.intakePower);
        clearInvocations(intake, shooter, stopper, frontLeft);
        op.stop();
        verify(shooter).setPower(0);
        verify(shooter, never()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(intake).setPower(0);
        verify(frontLeft).setPower(0);
        verify(stopper).setPosition(0.75);
    }

    @Test public void leftBumperTogglesTheSingleWedgeIndependentlyOfMotorButtons() {
        packet.left_bumper = true;
        initAndStart();
        verify(wedge).setPosition(0.2);
        verify(wedge, never()).setPosition(0.7);
        packet.left_bumper = false;
        loop();
        packet.left_bumper = true;
        packet.b = true;
        loop();
        verify(wedge).setPosition(0.7);
        verify(intake).setPower(FixedShotTeleOp.intakePower);
        clearInvocations(intake);
        packet.b = false;
        packet.y = true;
        packet.right_bumper = true;
        loop();
        verify(intake).setPower(FixedShotTeleOp.intakePower);
        verify(wedge).setPosition(0.7);
        packet.y = false;
        packet.left_bumper = false;
        loop();
        clearInvocations(intake);
        packet.b = packet.left_bumper = true;
        loop();
        verify(intake).setPower(0);
        verify(wedge, times(2)).setPosition(0.2);
        verify(map, never()).tryGet(Servo.class, "backWedge");
    }

    @Test public void anAliasedServoFailsInitBeforeMovingTheStopper() {
        when(map.tryGet(Servo.class, Constants.WEDGE_SERVO_NAME)).thenReturn(stopper);
        assertThrows(IllegalArgumentException.class, op::init);
        verifyNoInteractions(stopper);
    }

    @Test public void slewsProportionallyAndHoldsWhenBothTriggersArePressedOrReleased() {
        initAndStart();
        verify(turret, never()).setPosition(anyDouble());
        clearInvocations(turret);

        packet.left_trigger = 0.5f;
        loop();
        packet.right_trigger = 1;
        loop();
        packet.left_trigger = 0;
        loop();
        packet.right_trigger = 0.01f; // Inside the deadband.
        loop();
        FixedShotTeleOp.turretReversed = true;
        packet.left_trigger = 0.5f;
        loop();

        InOrder positions = inOrder(turret);
        positions.verify(turret).setPosition(doubleThat(value -> Math.abs(value - 0.495) < 1e-9));
        positions.verify(turret).setPosition(doubleThat(value -> Math.abs(value - 0.505) < 1e-9));
        positions.verify(turret).setPosition(doubleThat(value -> Math.abs(value - 0.510) < 1e-9));
        positions.verifyNoMoreInteractions();
        verify(turret, times(3)).setPosition(anyDouble());
    }

    @Test public void turretRequiresTriggerReleaseAfterStartAndCancelAndKeepsItsTargetOnStop() {
        packet.right_trigger = 1;
        initAndStart();
        clearInvocations(turret);

        loop();
        packet.right_trigger = 0;
        loop();
        packet.right_trigger = 1;
        loop();
        packet.x = true;
        loop();
        packet.x = false;
        loop();
        packet.right_trigger = 0;
        loop();
        packet.right_trigger = 1;
        loop();
        op.stop();
        loop();

        InOrder positions = inOrder(turret);
        positions.verify(turret).setPosition(doubleThat(value -> Math.abs(value - 0.510) < 1e-9));
        positions.verify(turret).setPosition(doubleThat(value -> Math.abs(value - 0.520) < 1e-9));
        positions.verifyNoMoreInteractions();
        verify(turret, times(2)).setPosition(anyDouble());
    }

    @Test public void turretStartsFromTheRangeMidpointIfUnknownAndClampsTravel() {
        when(turret.getPosition()).thenReturn(Double.NaN);
        initAndStart();
        runtime = 1; // A delayed loop uses at most 50 ms of trigger input.
        packet.right_trigger = 1;
        loop();
        verify(turret).setPosition(doubleThat(value -> Math.abs(value - 0.5125) < 1e-9));

        for (int i = 0; i < 20; i++) loop();
        verify(turret).setPosition(0.60);
        packet.right_trigger = 0;
        packet.left_trigger = 1;
        for (int i = 0; i < 30; i++) loop();
        verify(turret).setPosition(0.40);
        verify(turret, never()).setPosition(doubleThat(value -> value < 0.40 || value > 0.60));
    }

    @Test public void drivesWithoutConfiguredServosAndLeavesTheLauncherOff() {
        when(map.tryGet(Servo.class, Constants.WEDGE_SERVO_NAME)).thenReturn(null);
        when(map.tryGet(Servo.class, "shotStopper")).thenReturn(null);
        when(map.tryGet(Servo.class, Constants.TURRET_SERVO_NAME)).thenReturn(null);
        initAndStart();

        packet.left_stick_y = -0.4f;
        packet.a = packet.b = packet.left_bumper = true;
        packet.right_trigger = 1;
        loop();

        verify(frontLeft).setPower(doubleThat(power -> power > 0));
        verify(shooter, never()).setPower(doubleThat(power -> power != 0));
        verify(intake, never()).setPower(doubleThat(power -> power != 0));
        verifyNoInteractions(stopper, wedge, turret);
    }

    private void initAndStart() {
        op.init();
        op.gamepad1.copy(packet);
        op.start();
    }

    private void loop() {
        runtime += 0.04;
        op.gamepad1.copy(packet);
        op.loop();
    }

    private void rpm(double value) {
        when(shooter.getVelocity()).thenReturn(value * 28 / 60);
    }

    private <T> T device(Class<T> type, String name) {
        T device = mock(type);
        if (Servo.class.equals(type)) when(map.tryGet(type, name)).thenReturn(device);
        else when(map.get(type, name)).thenReturn(device);
        return device;
    }

    private static FixedShotTeleOp.WedgeSettings wedge(double stowed, double deployed) {
        FixedShotTeleOp.WedgeSettings settings = new FixedShotTeleOp.WedgeSettings();
        settings.stowedPosition = stowed;
        settings.deployedPosition = deployed;
        return settings;
    }
}
