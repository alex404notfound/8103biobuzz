package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class RobotHardwareProfileTest {
    @Test public void driveOnlyDoesNotRequireCameraOrExampleMotor() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        Robot robot = new Robot(hardwareMap, mock(Telemetry.class), Robot.HardwareProfile.DRIVE_ONLY, mock(Drivetrain.class));
        robot.updateMechanismsInInit();
        robot.stop();
        verify(hardwareMap, never()).get(DcMotorEx.class, "example");
        verify(hardwareMap, never()).get(Limelight3A.class, "limelight");
    }

    @Test public void stoppedRobotCannotReactivateOutputsOnLaterPeriodicTicks() {
        Drivetrain drivetrain = mock(Drivetrain.class);
        Robot robot = new Robot(mock(HardwareMap.class), mock(Telemetry.class), Robot.HardwareProfile.DRIVE_ONLY, drivetrain);
        robot.stop();
        robot.updateMechanismsInInit();
        robot.stop();
        verify(drivetrain).stop();
        verify(drivetrain, never()).periodic();
    }

    @Test public void explicitlyEnabledVisionStartsAndStopsWithoutAnExampleMotor() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        Limelight3A camera = mock(Limelight3A.class);
        when(hardwareMap.get(Limelight3A.class, "limelight")).thenReturn(camera);
        Robot robot = new Robot(hardwareMap, mock(Telemetry.class), Robot.HardwareProfile.DRIVE_AND_VISION, mock(Drivetrain.class));
        try { robot.updateMechanismsInInit(); } finally { robot.stop(); }
        verify(camera).start();
        verify(camera).stop();
        verify(hardwareMap, never()).get(DcMotorEx.class, "example");
    }

    @Test public void sharedContainerReleasesOldHardwareWhenTheNextOpModeChangesProfile() {
        HardwareMap visionMap = mock(HardwareMap.class);
        Limelight3A camera = mock(Limelight3A.class);
        when(visionMap.get(Limelight3A.class, "limelight")).thenReturn(camera);
        Drivetrain firstDrive = mock(Drivetrain.class);
        Robot robot = new Robot(visionMap, mock(Telemetry.class), Robot.HardwareProfile.DRIVE_AND_VISION, firstDrive);
        Drivetrain nextDrive = mock(Drivetrain.class);
        HardwareMap driveOnlyMap = mock(HardwareMap.class);
        try {
            robot.initialize(driveOnlyMap, mock(Telemetry.class), Robot.HardwareProfile.DRIVE_ONLY, nextDrive);
            verify(firstDrive).stop();
            verify(camera).stop();
            assertSame(nextDrive, robot.drivetrain);
            assertNull(robot.limelight);
            robot.updateMechanismsInInit();
            verify(firstDrive, never()).periodic();
            verify(nextDrive).periodic();
            verify(driveOnlyMap, never()).get(Limelight3A.class, "limelight");
        } finally { robot.stop(); }
        verify(nextDrive).stop();
    }
}
