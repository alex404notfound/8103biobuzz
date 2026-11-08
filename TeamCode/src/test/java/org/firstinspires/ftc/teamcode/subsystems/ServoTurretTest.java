package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret.GearPreset;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class ServoTurretTest {
    private final HardwareMap hardware = mock(HardwareMap.class);
    private final Servo servo = mock(Servo.class);

    @Test public void eachGearRatioConvertsNinetyTurretDegreesToTheCorrectServoPosition() {
        checkPosition(GearPreset.BIG, "turret", 0.5 + 83.0 / 270);
        checkPosition(GearPreset.SMALL, "smallTurret", 0.5 + (90 * 78.0 / 85) / 270);
    }

    private void checkPosition(GearPreset gear, String name, double expected) {
        ServoTurret turret = create(calibratedSettings(name, gear));
        verify(hardware).get(Servo.class, name);
        verifyNoMoreInteractions(hardware);
        verifyNoInteractions(servo);

        turret.setAngleDegrees(90);

        assertEquals(expected, turret.getCommandedPosition(), 1e-9);
        verify(servo).setPosition(turret.getCommandedPosition());
        clearInvocations(servo, hardware);
    }

    @Test public void reversedTurretRespectsItsCalibratedZeroAndSafeEndpoints() {
        ServoTurret.Settings settings = calibratedSettings("turret", GearPreset.BIG);
        settings.reversed = true;
        settings.zeroPosition = 0.6;
        settings.minPosition = 0.3;
        settings.maxPosition = 0.8;
        ServoTurret turret = create(settings);

        turret.setAngleDegrees(0);
        verify(servo).setPosition(0.6);
        turret.setAngleDegrees(30);
        assertEquals(0.6 - (30 * 83.0 / 90) / 270, turret.getCommandedPosition(), 1e-9);
        turret.setAngleDegrees(turret.getMinDegrees());
        assertEquals(0.8, turret.getCommandedPosition(), 1e-9);

        clearInvocations(servo);
        turret.setAngleDegrees(turret.getMinDegrees() - 0.01);
        turret.setAngleDegrees(turret.getMaxDegrees() + 0.01);
        turret.setAngleDegrees(Double.NaN);
        verifyNoInteractions(servo);
        assertEquals(0.8, turret.getCommandedPosition(), 1e-9);
    }

    @Test public void uncalibratedServoReceivesNoCommands() {
        ServoTurret turret = create(new ServoTurret.Settings("turret", GearPreset.BIG));
        assertFalse(turret.isCalibrated());
        turret.setAngleDegrees(0);
        turret.nudgeDegrees(5);
        verifyNoInteractions(servo);
    }

    private ServoTurret create(ServoTurret.Settings settings) {
        when(hardware.get(Servo.class, settings.servoName)).thenReturn(servo);
        return new ServoTurret(hardware, settings);
    }

    private static ServoTurret.Settings calibratedSettings(String name, GearPreset gear) {
        ServoTurret.Settings settings = new ServoTurret.Settings(name, gear);
        settings.servoSweepDegrees = 270;
        settings.minPosition = 0.1;
        settings.maxPosition = 0.9;
        return settings;
    }
}
