package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static com.pedropathing.api.Paths.line;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

/** Characterizes v3 boundaries where a version-only upgrade can leave motors running. */
public class Pedro3DriveContractTest {
    private final HardwareMap map = mock(HardwareMap.class);
    private final GoBildaPinpointDriver pinpoint = mock(GoBildaPinpointDriver.class);
    private final Algorithm algorithm = mock(Algorithm.class);
    private final TestLocalizer localizer = new TestLocalizer();
    private final double[] powers = new double[4];
    private Follower follower;
    private Drivetrain drive;
    private boolean savedTuned;

    @Before public void setup() {
        Scheduler.reset();
        savedTuned = Constants.foresightTuned;
        Constants.foresightTuned = true;
        String[] names = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (int i = 0; i < names.length; i++) {
            final int index = i;
            DcMotorEx motor = mock(DcMotorEx.class);
            doAnswer(call -> { powers[index] = call.getArgument(0); return null; })
                    .when(motor).setPower(anyDouble());
            when(map.get(DcMotorEx.class, names[i])).thenReturn(motor);
        }
        when(map.get(GoBildaPinpointDriver.class, "pinpoint")).thenReturn(pinpoint);
        when(pinpoint.getDeviceStatus()).thenReturn(GoBildaPinpointDriver.DeviceStatus.READY);
        follower = new Follower(localizer, Constants.createDrivetrain(map), algorithm);
        drive = new Drivetrain(follower, map, mock(Telemetry.class), () -> 0L);
        drive.periodic();
    }

    @After public void cleanup() { Scheduler.reset(); Constants.foresightTuned = savedTuned; }

    @Test public void cancellationZerosRealPedroMotorOutputWithoutAnotherTick() {
        Command command = drive.followPath(line(Pose.zero(), new Pose(20, 0)).constant(0));
        command.schedule();
        when(algorithm.calculatePath(any(), any(), any(), anyDouble()))
                .thenReturn(new DrivePowers(0.6, 0, 0));
        drive.periodic();
        assertEquals(0.6, powers[0], 0);
        command.cancel();
        assertArrayEquals(new double[4], powers, 0);
        assertTrue(follower.idle());
    }

    @Test public void invalidControllerOutputStopsInsteadOfRetainingLastMotorPower() {
        Command command = drive.followPath(line(Pose.zero(), new Pose(20, 0)).constant(0));
        command.schedule();
        when(algorithm.calculatePath(any(), any(), any(), anyDouble()))
                .thenReturn(new DrivePowers(0.6, 0, 0));
        drive.periodic();
        assertEquals(0.6, powers[0], 0);
        when(algorithm.calculatePath(any(), any(), any(), anyDouble()))
                .thenReturn(new DrivePowers(Double.NaN, 0, 0));
        drive.periodic();
        assertArrayEquals(new double[4], powers, 0);
        assertFalse(drive.isLocalizationReady());
        assertTrue(follower.idle());
    }

    @Test public void localizationOnlyTickPreservesManualDriveAndStopClearsCachedPowers() {
        drive.arcadeDrive(0.5, 0, 0, Alliance.RED);
        drive.periodic();
        assertArrayEquals(new double[]{0.275, 0.275, 0.275, 0.275}, powers, 1e-9);
        drive.stop();
        assertArrayEquals(new double[4], powers, 0);
        drive.arcadeDrive(0.5, 0, 0, Alliance.RED);
        assertEquals(0.275, powers[0], 1e-9);
    }

    @Test public void parametricEndpointDoesNotAdvanceCommandsBeforeEndHoldSettles() {
        Command command = drive.followPath(line(Pose.zero(), new Pose(20, 0)).constant(0));
        command.schedule();
        when(algorithm.atParametricEnd()).thenReturn(true);
        when(algorithm.isBusy()).thenReturn(true);
        Scheduler.execute();
        assertTrue(command.isScheduled());
        follower.hold(new Pose(20, 0));
        Scheduler.execute();
        assertTrue(command.isScheduled());
        when(algorithm.isBusy()).thenReturn(false);
        Scheduler.execute();
        assertFalse(command.isScheduled());
        assertTrue(follower.holding());
    }

    @Test public void angularVelocityIsNotNormalizedLikeAnAngle() {
        localizer.velocity = new Velocity(0, 0, -2 * Math.PI);
        assertEquals(-2 * Math.PI, drive.getVelocity().omega, 0);
    }

    @Test public void renamedDriveMotorsUseTheSameConfigurationForDrivingAndStopping() {
        MecanumConfig original = Constants.drivetrainConfig;
        try {
            Constants.drivetrainConfig = new MecanumConfig(c -> {
                c.frontLeftName.set("leftFrontDrive");
                c.frontRightName.set("rightFrontDrive");
                c.backLeftName.set("leftRearDrive");
                c.backRightName.set("rightRearDrive");
                c.frontLeftDirection.set(original.frontLeftDirection.get());
                c.frontRightDirection.set(original.frontRightDirection.get());
                c.backLeftDirection.set(original.backLeftDirection.get());
                c.backRightDirection.set(original.backRightDirection.get());
            });
            HardwareMap renamed = mock(HardwareMap.class);
            String[] names = {"leftFrontDrive", "rightFrontDrive", "leftRearDrive", "rightRearDrive"};
            for (int i = 0; i < names.length; i++) {
                final int index = i;
                DcMotorEx motor = mock(DcMotorEx.class);
                doAnswer(call -> { powers[index] = call.getArgument(0); return null; })
                        .when(motor).setPower(anyDouble());
                when(renamed.get(DcMotorEx.class, names[i])).thenReturn(motor);
            }
            when(renamed.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()))
                    .thenReturn(pinpoint);
            Follower renamedFollower = new Follower(localizer, Constants.createDrivetrain(renamed), algorithm);
            Drivetrain renamedDrive = new Drivetrain(renamedFollower, renamed, mock(Telemetry.class), () -> 0L);
            renamedDrive.periodic();
            renamedDrive.arcadeDrive(0.5, 0, 0, Alliance.RED);
            assertArrayEquals(new double[]{0.275, 0.275, 0.275, 0.275}, powers, 1e-9);
            renamedDrive.stop();
            assertArrayEquals(new double[4], powers, 0);
        } finally {
            Constants.drivetrainConfig = original;
        }
    }

    @Test public void untunedForesightCannotStartAPathButManualDriveRemainsAvailable() {
        Constants.foresightTuned = false;
        Command command = drive.followPath(line(Pose.zero(), new Pose(20, 0)).constant(0));
        assertThrows(IllegalStateException.class, command::schedule);
        assertTrue(follower.idle());
        assertArrayEquals(new double[4], powers, 0);
        drive.arcadeDrive(0.5, 0, 0, Alliance.RED);
        assertEquals(0.275, powers[0], 1e-9);
    }

    private static class TestLocalizer implements Localizer {
        Pose pose = Pose.zero();
        Velocity velocity = Velocity.zero();
        public void setPose(Pose pose) { this.pose = pose; }
        public MotionState state() { return MotionState.ofVelocity(pose, velocity); }
        public void update() { }
        public void reset() { }
    }
}
