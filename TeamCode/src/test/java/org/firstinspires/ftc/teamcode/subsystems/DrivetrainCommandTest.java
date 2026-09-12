package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static org.junit.Assert.*;
import java.util.concurrent.atomic.AtomicLong;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class DrivetrainCommandTest {
    private final Localizer localizer = mock(Localizer.class);
    private final Follower follower = spy(new Follower(localizer,
            mock(com.pedropathing.drivetrain.Drivetrain.class), mock(Algorithm.class)));
    private boolean savedTuned;
    private final Path path = mock(Path.class);
    private Drivetrain drivetrain;
    private final AtomicLong clock = new AtomicLong();
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final DcMotorEx motor = mock(DcMotorEx.class);

    @Before public void setUp() {
        Scheduler.reset();
        savedTuned = Constants.foresightTuned;
        Constants.foresightTuned = true;
        doNothing().when(follower).follow(any());
        doNothing().when(follower).update();
        doReturn(false).when(follower).idle();
        doReturn(true).when(follower).holding();
        doReturn(Velocity.zero()).when(follower).velocity();
        when(follower.isBusy()).thenReturn(true);
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(eq(DcMotorEx.class), anyString())).thenReturn(motor);
        when(hardwareMap.get(eq(GoBildaPinpointDriver.class), anyString())).thenReturn(pinpoint);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        doReturn(Pose.zero()).when(follower).pose();
        drivetrain = new Drivetrain(follower, hardwareMap, mock(Telemetry.class), clock::get);
        clearInvocations(follower, motor);
    }

    @Test public void faultedPinpointPreventsFieldDriveWithRetainedFinitePose() {
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        drivetrain.periodic();
        drivetrain.arcadeDrive(1, 0, 0, org.firstinspires.ftc.teamcode.robot.Alliance.RED);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void nonFinitePosePreventsFieldDriveEvenWhenPinpointSaysReady() {
        doReturn(new Pose(Double.NaN, 0, 0)).when(follower).pose();
        drivetrain.periodic();
        drivetrain.arcadeDrive(1, 0, 0, org.firstinspires.ftc.teamcode.robot.Alliance.RED);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void readyRequiresAnActualRecentLocalizationSample() {
        assertFalse(drivetrain.isLocalizationReady());
        drivetrain.periodic();
        assertTrue(drivetrain.isLocalizationReady());
        clock.set(250_000_001L);
        assertFalse(drivetrain.isLocalizationReady());
        assertEquals("STALE_LOCALIZATION_SAMPLE", drivetrain.getLocalizationStatus());
    }

    @Test public void calibrationPreservesPoseAndWaitsForNewReadySample() {
        drivetrain.periodic();
        drivetrain.recalibrateLocalization();
        assertFalse(drivetrain.isLocalizationReady());
        verify(pinpoint).recalibrateIMU();
        verify(pinpoint, never()).resetPosAndIMU();
        verify(follower, never()).setPose(any());
        drivetrain.periodic(); // A cached READY flag cannot finish calibration immediately.
        assertFalse(drivetrain.isLocalizationReady());
        clock.set(300_000_000L);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.CALIBRATING);
        drivetrain.periodic();
        assertFalse(drivetrain.isLocalizationReady());
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        drivetrain.periodic();
        assertTrue(drivetrain.isLocalizationReady());
        verify(follower, never()).update();
    }

    @Test public void localizationFaultStopsFollowerAndLaterTicksOnlySamplePose() {
        drivetrain.followPath(path).schedule();
        doThrow(new org.firstinspires.ftc.teamcode.pedroPathing.Constants.LocalizationNotReady("FAULT_BAD_READ"))
                .when(follower).update();
        drivetrain.periodic();
        assertFalse(drivetrain.isLocalizationReady());
        verify(follower).stop();
        drivetrain.periodic();
        verify(follower, times(1)).update();
        verify(localizer).update();
    }

    @Test public void visionCorrectionBreaksHoldAndKeepsOdometryHeading() {
        doReturn(new Pose(0, 0, 1.2)).when(follower).pose();
        drivetrain.periodic();
        drivetrain.followPath(path).schedule();
        drivetrain.applyVisionTranslation(new Pose(20, 30, 2.7));
        org.mockito.ArgumentCaptor<Pose> pose = org.mockito.ArgumentCaptor.forClass(Pose.class);
        InOrder order = inOrder(follower);
        order.verify(follower).stop();
        order.verify(follower).setPose(pose.capture());
        assertEquals(20, pose.getValue().x(), 0);
        assertEquals(30, pose.getValue().y(), 0);
        assertEquals(1.2, pose.getValue().heading(), 0);
        drivetrain.periodic();
        verify(follower, never()).update();
    }

    @SuppressWarnings("unchecked")
    @Test public void realPedroGuardRejectsFaultsBeforeAnyDriveOutput() throws Exception {
        HardwareMap map = mock(HardwareMap.class);
        when(map.get(eq(DcMotorEx.class), anyString())).thenReturn(motor);
        when(map.get(eq(GoBildaPinpointDriver.class), anyString())).thenReturn(pinpoint);
        com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType type =
                new com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType();
        when(motor.getMotorType()).thenReturn(type);
        HardwareMap.DeviceMapping<com.qualcomm.robotcore.hardware.VoltageSensor> voltage = mock(HardwareMap.DeviceMapping.class);
        when(voltage.iterator()).thenReturn(java.util.Collections.singletonList(
                mock(com.qualcomm.robotcore.hardware.VoltageSensor.class)).iterator());
        java.lang.reflect.Field field = HardwareMap.class.getField("voltageSensor");
        field.setAccessible(true);
        field.set(map, voltage);
        when(pinpoint.getPosition()).thenReturn(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH, 0, 0,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS, 0));
        Follower real = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createGuardedFollower(map);
        real.follow(com.pedropathing.api.Paths.line(Pose.zero(), new Pose(20, 0)).constant(0));
        clearInvocations(motor);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        assertThrows(org.firstinspires.ftc.teamcode.pedroPathing.Constants.LocalizationNotReady.class, real::update);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        when(pinpoint.getVelX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)).thenReturn(Double.NaN);
        assertThrows(org.firstinspires.ftc.teamcode.pedroPathing.Constants.LocalizationNotReady.class, real::update);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
    }

    @After public void resetScheduler() { Scheduler.reset(); Constants.foresightTuned = savedTuned; }

    @Test public void cancelImmediatelyStopsTheFollower() {
        Command action = drivetrain.followPath(path);
        action.schedule();
        action.cancel();
        verify(follower).stop();
        assertFalse(action.isScheduled());
    }

    @Test public void timeoutStopsTheFollowerBeforeAnotherPeriodicTick() {
        Command action = race(drivetrain.followPath(path), waitMs(0));
        action.schedule();
        Scheduler.execute();
        verify(follower).stop();
        assertFalse(action.isScheduled());
    }

    @Test public void replacementPathInterruptsThePreviousDriveOwnerBeforeStarting() {
        Command previous = drivetrain.followPath(path);
        Path replacementPath = mock(Path.class);
        Command replacement = drivetrain.followPath(replacementPath);
        previous.schedule();
        replacement.schedule();

        assertFalse(previous.isScheduled());
        InOrder order = inOrder(follower);
        order.verify(follower).follow(eq(path));
        order.verify(follower).stop();
        order.verify(follower).follow(eq(replacementPath));
    }

    @Test public void naturalCompletionPreservesPedrosConfiguredEndHold() {
        Command action = drivetrain.followPath(path);
        action.schedule();
        when(follower.isBusy()).thenReturn(false);
        Scheduler.execute();
        assertFalse(action.isScheduled());
        verify(follower, never()).stop();
    }
}
