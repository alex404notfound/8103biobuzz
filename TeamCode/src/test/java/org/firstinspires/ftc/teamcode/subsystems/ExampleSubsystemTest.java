package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.mockito.Mockito.*;

public class ExampleSubsystemTest {
    private final DcMotorEx motor = mock(DcMotorEx.class);
    private ExampleSubsystem subsystem;

    @Before public void setUp() {
        Scheduler.reset();
        ExampleSubsystem.onPower = 1;
        ExampleSubsystem.offPower = 0;
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(DcMotorEx.class, "example")).thenReturn(motor);
        subsystem = new ExampleSubsystem(hardwareMap, mock(Telemetry.class));
    }

    @After public void tearDown() { Scheduler.reset(); ExampleSubsystem.offPower = 0; }

    @Test public void cancelingLifetimeCommandTurnsMotorOffOnTheSameCycle() {
        Command action = subsystem.runWhileScheduled();
        action.schedule();
        subsystem.periodic();
        verify(motor).setPower(1);
        clearInvocations(motor);

        action.cancel();
        subsystem.periodic();
        verify(motor).setPower(0);
        assertFalse(action.isScheduled());
    }

    @Test public void offInterruptsLifetimeCommandThroughSharedRequirement() {
        Command action = subsystem.runWhileScheduled();
        action.schedule();
        subsystem.off().schedule();
        Scheduler.execute();
        subsystem.periodic();
        verify(motor).setPower(0);
        assertFalse(action.isScheduled());
    }

    @Test public void instantOnLeavesItsModeActiveAfterCommandFinishes() {
        Command action = subsystem.on();
        action.schedule();
        Scheduler.execute();
        assertFalse(action.isScheduled());
        subsystem.periodic();
        verify(motor).setPower(1);
    }

    @Test public void stopAlwaysWritesZeroEvenWhenConfiguredOffPowerIsNonzero() {
        ExampleSubsystem.offPower = .2;
        subsystem.on().schedule();
        subsystem.stop();
        verify(motor).setPower(0);
    }
}
