package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.junit.Test;
import static org.mockito.Mockito.*;

public class RobotHardwareProfileTest {
    @Test public void driveOnlyDoesNotRequireCameraOrExampleMotor() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        Robot robot = new Robot(hardwareMap, mock(Telemetry.class), Robot.HardwareProfile.DRIVE_ONLY, mock(Drivetrain.class));
        robot.periodic();
        robot.stop();
        verify(hardwareMap, never()).get(DcMotorEx.class, "example");
        verify(hardwareMap, never()).get(Limelight3A.class, "limelight");
    }

    @Test public void stoppedRobotCannotReactivateOutputsOnLaterPeriodicTicks() {
        Drivetrain drivetrain = mock(Drivetrain.class);
        Robot robot = new Robot(mock(HardwareMap.class), mock(Telemetry.class), Robot.HardwareProfile.DRIVE_ONLY, drivetrain);
        robot.stop();
        robot.periodic();
        robot.stop();
        verify(drivetrain).stop();
        verify(drivetrain, never()).periodic();
    }

    @Test public void explicitlyEnabledVisionStartsAndStopsWithoutAnExampleMotor() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        Limelight3A camera = mock(Limelight3A.class);
        when(hardwareMap.get(Limelight3A.class, "limelight")).thenReturn(camera);
        Robot robot = new Robot(hardwareMap, mock(Telemetry.class), Robot.HardwareProfile.DRIVE_AND_VISION, mock(Drivetrain.class));
        try { robot.periodic(); } finally { robot.stop(); }
        verify(camera).start();
        verify(camera).stop();
        verify(hardwareMap, never()).get(DcMotorEx.class, "example");
    }
}
