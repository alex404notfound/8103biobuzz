package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
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
    private final Follower follower = mock(Follower.class);
    private final PathChain path = mock(PathChain.class);
    private Drivetrain drivetrain;
    private final AtomicLong clock = new AtomicLong();
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final DcMotorEx motor = mock(DcMotorEx.class);

    @Before public void setUp() {
        Scheduler.reset();
        follower.constants = new FollowerConstants();
        when(follower.getMaxPowerScaling()).thenReturn(1.0);
        when(follower.isBusy()).thenReturn(true);
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(eq(DcMotorEx.class), anyString())).thenReturn(motor);
        when(hardwareMap.get(eq(GoBildaPinpointDriver.class), anyString())).thenReturn(pinpoint);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        when(follower.getPose()).thenReturn(new Pose());
        drivetrain = new Drivetrain(follower, hardwareMap, mock(Telemetry.class), clock::get);
    }

    @Test public void faultedPinpointPreventsFieldDriveWithRetainedFinitePose() {
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        drivetrain.periodic();
        drivetrain.arcadeDrive(1, 0, 0, org.firstinspires.ftc.teamcode.robot.Alliance.RED);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
    }

    @Test public void nonFinitePosePreventsFieldDriveEvenWhenPinpointSaysReady() {
        when(follower.getPose()).thenReturn(new Pose(Double.NaN, 0, 0));
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
        verify(follower).breakFollowing();
        drivetrain.periodic();
        verify(follower, times(1)).update();
        verify(follower).updatePose();
    }

    @Test public void visionCorrectionBreaksHoldAndKeepsOdometryHeading() {
        when(follower.getHeading()).thenReturn(1.2);
        drivetrain.periodic();
        drivetrain.followPath(path).schedule();
        drivetrain.applyVisionTranslation(new Pose(20, 30, 2.7));
        org.mockito.ArgumentCaptor<Pose> pose = org.mockito.ArgumentCaptor.forClass(Pose.class);
        InOrder order = inOrder(follower);
        order.verify(follower).breakFollowing();
        order.verify(follower).setPose(pose.capture());
        assertEquals(20, pose.getValue().getX(), 0);
        assertEquals(30, pose.getValue().getY(), 0);
        assertEquals(1.2, pose.getValue().getHeading(), 0);
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
        real.followPath(real.pathBuilder().addPath(new com.pedropathing.geometry.BezierLine(
                new Pose(), new Pose(20, 0))).build());
        clearInvocations(motor);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ);
        assertThrows(org.firstinspires.ftc.teamcode.pedroPathing.Constants.LocalizationNotReady.class, real::update);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        when(pinpoint.getVelX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)).thenReturn(Double.NaN);
        assertThrows(org.firstinspires.ftc.teamcode.pedroPathing.Constants.LocalizationNotReady.class, real::update);
        verify(motor, never()).setPower(doubleThat(power -> power != 0));
    }

    @After public void resetScheduler() { Scheduler.reset(); }

    @Test public void cancelImmediatelyStopsTheFollower() {
        Command action = drivetrain.followPath(path);
        action.schedule();
        action.cancel();
        verify(follower).breakFollowing();
        assertFalse(action.isScheduled());
    }

    @Test public void timeoutStopsTheFollowerBeforeAnotherPeriodicTick() {
        Command action = race(drivetrain.followPath(path), waitMs(0));
        action.schedule();
        Scheduler.execute();
        verify(follower).breakFollowing();
        assertFalse(action.isScheduled());
    }

    @Test public void replacementPathInterruptsThePreviousDriveOwnerBeforeStarting() {
        Command previous = drivetrain.followPath(path);
        PathChain replacementPath = mock(PathChain.class);
        Command replacement = drivetrain.followPath(replacementPath);
        previous.schedule();
        replacement.schedule();

        assertFalse(previous.isScheduled());
        InOrder order = inOrder(follower);
        order.verify(follower).followPath(eq(path), eq(1.0), anyBoolean());
        order.verify(follower).breakFollowing();
        order.verify(follower).followPath(eq(replacementPath), eq(1.0), anyBoolean());
    }

    @Test public void naturalCompletionPreservesPedrosConfiguredEndHold() {
        Command action = drivetrain.followPath(path);
        action.schedule();
        when(follower.isBusy()).thenReturn(false);
        Scheduler.execute();
        assertFalse(action.isScheduled());
        verify(follower, never()).breakFollowing();
    }
}
