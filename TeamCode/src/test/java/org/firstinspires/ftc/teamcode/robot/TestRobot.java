package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import static org.mockito.Mockito.*;

/** Real NextRobot registration/order/cleanup with a mocked drivetrain at the hardware boundary. */
public class TestRobot extends Robot {
    public final Drivetrain drive = mock(Drivetrain.class);
    public final Telemetry output = mock(Telemetry.class);
    public final HardwareMap map = mock(HardwareMap.class);

    public TestRobot() {
        when(drive.getDefaultCommand()).thenAnswer(call -> Commands.infinite(() -> { }).requiring(drive));
    }

    @Override public void initialize(HardwareMap hardwareMap, Telemetry driverTelemetry, HardwareProfile profile) {
        initialize(map, output, profile, drive);
    }
}
