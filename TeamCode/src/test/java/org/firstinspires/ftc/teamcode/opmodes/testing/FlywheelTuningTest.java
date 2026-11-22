package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.Collections;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.doubleThat;
import static org.mockito.Mockito.*;

public class FlywheelTuningTest {
    private final FlywheelTuning op = new FlywheelTuning();
    private final Telemetry telemetry = mock(Telemetry.class);
    private final DcMotorEx shooter = mock(DcMotorEx.class);
    private final DcMotorEx shooter2 = mock(DcMotorEx.class);
    private final DcMotorEx intake = mock(DcMotorEx.class);
    private final FlywheelTuning.ControlMode originalMode = FlywheelTuning.controlMode;
    private final ShooterFlywheel.SpeedTuning originalSpeed = FlywheelTuning.speed;
    private final ShooterFlywheel.SpeedTuning originalSpeed2 = FlywheelTuning.speed2;
    private final double originalPower = FlywheelTuning.power;
    private final double originalPower2 = FlywheelTuning.power2;
    private final double originalIntake = FlywheelTuning.intakePower;
    private final int originalBlocked = FlywheelTuning.shotStopperBlocked;

    @Before public void setUp() throws ReflectiveOperationException {
        op.hardwareMap = mock(HardwareMap.class);
        op.telemetry = telemetry;
        when(op.hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME)).thenReturn(shooter);
        when(op.hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR_NAME)).thenReturn(intake);
        when(shooter.getVelocity()).thenReturn(700.0); // 1500 RPM at 28 ticks per revolution.
        VoltageSensor battery = mock(VoltageSensor.class);
        when(battery.getVoltage()).thenReturn(12.0);
        @SuppressWarnings("unchecked")
        HardwareMap.DeviceMapping<VoltageSensor> sensors = mock(HardwareMap.DeviceMapping.class);
        when(sensors.iterator()).thenAnswer(call -> Collections.singletonList(battery).iterator());
        Field voltageSensor = HardwareMap.class.getField("voltageSensor");
        voltageSensor.setAccessible(true);
        voltageSensor.set(op.hardwareMap, sensors);

        FlywheelTuning.speed = new ShooterFlywheel.SpeedTuning(0, 0.002, true);
        FlywheelTuning.speed2 = new ShooterFlywheel.SpeedTuning(0, 0.003, false);
        FlywheelTuning.power = 0.25;
        FlywheelTuning.power2 = 0;
    }

    @After public void tearDown() {
        try { op.stop(); }
        finally {
            FlywheelTuning.controlMode = originalMode;
            FlywheelTuning.speed = originalSpeed;
            FlywheelTuning.speed2 = originalSpeed2;
            FlywheelTuning.power = originalPower;
            FlywheelTuning.power2 = originalPower2;
            FlywheelTuning.intakePower = originalIntake;
            FlywheelTuning.shotStopperBlocked = originalBlocked;
        }
    }

    @Test public void publishesLivePowerAndRpmWithoutTheStopperAndKeepsFeedOff() {
        assertEquals(FlywheelTuning.ControlMode.RPM, FlywheelTuning.controlMode);
        op.init();
        FlywheelTuning.intakePower = 0.5;
        FlywheelTuning.shotStopperBlocked = 0;
        op.init_loop();
        verify(shooter, never()).setPower(doubleThat(power -> power != 0));
        verify(telemetry).addData("Target RPM", 0.0);
        verify(telemetry).addData("Current RPM", 1500.0);
        verify(telemetry, atLeastOnce()).addLine("Motor 2 unavailable. Configure shooter2 on Control Hub port 2.");

        FlywheelTuning.controlMode = FlywheelTuning.ControlMode.POWER;
        op.init_loop();
        verify(shooter).setPower(0.25);

        op.start();
        FlywheelTuning.controlMode = FlywheelTuning.ControlMode.RPM;
        FlywheelTuning.speed.setpointRpm = 2400;
        when(shooter.getVelocity()).thenReturn(840.0);
        op.loop();
        verify(telemetry).addData("Target RPM", 2400.0);
        verify(telemetry).addData("Current RPM", 1800.0);
        verify(shooter).setPower(doubleThat(power -> Math.abs(power - 0.4) < 1e-9));
        verify(intake, never()).setPower(doubleThat(power -> power != 0));
        clearInvocations(shooter);
        op.stop();
        verify(shooter).setPower(0);
    }

    @Test public void runsTheSharedIntakeAndStopperWhenTheServoIsConfigured() {
        Servo stopper = mock(Servo.class);
        when(op.hardwareMap.tryGet(Servo.class, "shotStopper")).thenReturn(stopper);
        op.init();
        verify(stopper).setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
        FlywheelTuning.intakePower = 0.2;
        FlywheelTuning.shotStopperBlocked = 0;
        op.init_loop();
        verify(intake).setPower(0.2);
        verify(stopper).setPosition(Constants.SHOT_STOPPER_UNBLOCKED_POSITION);
    }

    @Test public void initializesShooterTwoWithItsDefaultForwardDirection() {
        FlywheelTuning.speed2 = originalSpeed2;
        when(op.hardwareMap.tryGet(DcMotorEx.class, Constants.SECOND_SHOOTER_MOTOR_NAME)).thenReturn(shooter2);

        op.init();

        verify(shooter2).setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Test public void tunesBothMotorsWithSeparateTargetsGainsAndEncoderReadings() {
        assertNotSame(originalSpeed, originalSpeed2);
        when(op.hardwareMap.tryGet(DcMotorEx.class, Constants.SECOND_SHOOTER_MOTOR_NAME)).thenReturn(shooter2);
        when(shooter2.getVelocity()).thenReturn(560.0); // 1200 RPM.
        FlywheelTuning.speed.setpointRpm = 2400;
        FlywheelTuning.speed2.setpointRpm = 2000;
        op.init();
        op.start();
        op.loop();

        verify(telemetry).addData("Target RPM", 2400.0);
        verify(telemetry).addData("Current RPM", 1500.0);
        verify(telemetry).addData("Motor 2 Target RPM", 2000.0);
        verify(telemetry).addData("Motor 2 Current RPM", 1200.0);
        verify(shooter).setPower(doubleThat(power -> Math.abs(power - 0.4) < 1e-9));
        verify(shooter2).setPower(0.5);

        clearInvocations(shooter, shooter2);
        FlywheelTuning.speed2.kP = -1;
        op.loop();
        verify(shooter).setPower(doubleThat(power -> power > 0));
        verify(shooter2, never()).setPower(doubleThat(power -> power != 0));
        verify(telemetry).addData("shooter2.status", "Invalid flywheel gains, limits or watchdog settings");

        clearInvocations(shooter, shooter2);
        op.stop();
        verify(shooter).setPower(0);
        verify(shooter2).setPower(0);
    }

    @Test public void rejectsMappingBothControllersToOnePhysicalMotor() {
        when(op.hardwareMap.tryGet(DcMotorEx.class, Constants.SECOND_SHOOTER_MOTOR_NAME)).thenReturn(shooter);
        assertThrows(IllegalArgumentException.class, op::init);
        verifyNoInteractions(shooter, intake);
    }

    @Test public void stopsBothFlywheelsWhenTheSecondEncoderReadThrows() {
        when(op.hardwareMap.tryGet(DcMotorEx.class, Constants.SECOND_SHOOTER_MOTOR_NAME)).thenReturn(shooter2);
        FlywheelTuning.controlMode = FlywheelTuning.ControlMode.POWER;
        FlywheelTuning.power2 = 0.1;
        op.init();
        op.loop();
        verify(shooter).setPower(0.25);
        verify(shooter2).setPower(0.1);
        clearInvocations(shooter, shooter2);

        when(shooter2.getVelocity()).thenThrow(new IllegalStateException("encoder read failed"));
        assertThrows(IllegalStateException.class, op::loop);
        verify(shooter).setPower(0);
        verify(shooter2).setPower(0);
    }
}
