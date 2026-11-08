package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HiveAutoAim;
import org.firstinspires.ftc.teamcode.subsystems.ServoTurret;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.InOrder;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class HiveAutoAimOpModeTest {
    private final ServoTurret turret = mock(ServoTurret.class);
    private final Drivetrain drive = mock(Drivetrain.class);
    private final HiveAutoAim aim = mock(HiveAutoAim.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private HiveAutoAimTest op;

    @Before public void setUp() {
        defaults();
        when(drive.getPose()).thenReturn(new Pose(30, 40, .25));
        when(drive.getVelocity()).thenReturn(new Velocity(0, 0, 0));
        when(drive.isLocalizationReady()).thenReturn(true);
        when(turret.getCommandedDegrees()).thenReturn(15.0);
        when(turret.getMinDegrees()).thenReturn(-90.0);
        when(turret.getMaxDegrees()).thenReturn(90.0);
        when(aim.getTurretGoalDegrees()).thenReturn(35.0);
        op = new HiveAutoAimTest(turret, drive, aim, telemetry);
        op.gamepad1 = mock(Gamepad.class);
        op.gamepad2 = mock(Gamepad.class);
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        HiveAutoAimTest.alliance = Alliance.RED;
        HiveAutoAimTest.selectedCell = HiveCell.RED_AUDIENCE;
        HiveAutoAimTest.startPoseConfigured = false;
        HiveAutoAimTest.startX = HiveAutoAimTest.startY = HiveAutoAimTest.startHeadingDegrees = 0;
        HiveAutoAimTest.sideUncertaintyInches = 6;
        HiveAutoAimTest.maxAimChassisSpeed = 1;
        HiveAutoAimTest.maxAimChassisTurnDegrees = 5;
    }

    @Test public void initAndHeldStartTriggerCannotMoveUntilEveryDeadmanIsReleased() {
        seedPose();
        op.gamepad1.right_trigger = 1;
        op.init_loop(); op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
        op.start(); op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
        op.gamepad1.right_trigger = 0;
        op.gamepad2.left_bumper = true;
        op.loop();
        op.gamepad1.right_trigger = 1;
        op.gamepad2.left_bumper = false;
        op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).setAngleDegrees(35);
    }

    @Test public void holdAimUsesLatestGoalAndStopsTheChassisEvenWithDriveDeadmanHeld() {
        seedPose(); arm();
        op.gamepad2.left_bumper = true;
        op.gamepad2.left_stick_y = -1;
        op.gamepad1.right_trigger = 1;
        op.loop();
        InOrder order = inOrder(drive, aim, turret);
        order.verify(drive).periodic();
        order.verify(aim).update(true, true, 30, 40, .25, 15, -90, 90, Alliance.RED, HiveCell.RED_AUDIENCE);
        order.verify(turret).setAngleDegrees(35);
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
        when(aim.getTurretGoalDegrees()).thenReturn(-20.0);
        op.loop();
        verify(turret).setAngleDegrees(-20);
        clearInvocations(turret);
        op.gamepad1.right_trigger = 0;
        op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
    }

    @Test public void movingOrUnhealthyChassisPreventsAnOtherwiseAvailableAim() {
        seedPose();
        arm();
        op.gamepad1.right_trigger = 1;
        for (Velocity velocity : new Velocity[]{new Velocity(1.01, 0, 0), new Velocity(.8, .8, 0),
                new Velocity(0, 0, Math.toRadians(5.01)), new Velocity(0, 0, Math.toRadians(-5.01)),
                new Velocity(Double.NaN, 0, 0), new Velocity(0, 0, Double.NaN)}) {
            when(drive.getVelocity()).thenReturn(velocity);
            op.loop();
        }
        when(drive.getVelocity()).thenReturn(new Velocity(0, 0, 0));
        when(drive.isLocalizationReady()).thenReturn(false);
        op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
    }

    @Test public void invalidOrExcessiveStationarityThresholdsDisableAim() {
        seedPose();
        arm(); op.gamepad1.right_trigger = 1;
        for (double threshold : new double[]{-.1, 3.01, Double.NaN, Double.POSITIVE_INFINITY}) {
            HiveAutoAimTest.maxAimChassisSpeed = threshold;
            op.loop();
        }
        HiveAutoAimTest.maxAimChassisSpeed = 1;
        for (double threshold : new double[]{-.1, 15.01, Double.NaN, Double.POSITIVE_INFINITY}) {
            HiveAutoAimTest.maxAimChassisTurnDegrees = threshold;
            op.loop();
        }
        verify(turret, never()).setAngleDegrees(anyDouble());
    }

    @Test public void losingUsablePoseAimSendsNoNewServoPosition() {
        seedPose();
        arm(); op.gamepad1.right_trigger = 1;
        op.loop();
        verify(turret).setAngleDegrees(35);
        clearInvocations(turret, drive);
        when(aim.getTurretGoalDegrees()).thenReturn(Double.NaN);
        op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
        verify(drive, atLeastOnce()).stop();
    }

    @Test public void manualTurretNudgeOverridesAimOnlyWhileItsDeadmanIsHeld() {
        arm();
        op.gamepad1.right_trigger = 1;
        op.gamepad1.left_bumper = true;
        when(op.gamepad1.dpadLeftWasPressed()).thenReturn(true, false);
        op.loop();
        verify(turret).nudgeDegrees(-5);
        verify(turret, never()).setAngleDegrees(anyDouble());
        op.loop();
        op.gamepad1.left_bumper = false;
        op.gamepad1.right_trigger = 0;
        when(op.gamepad1.dpadRightWasPressed()).thenReturn(true);
        op.loop();
        verify(turret).nudgeDegrees(anyDouble());
        verify(turret, never()).setAngleDegrees(anyDouble());
    }

    @Test public void cancelStopsPositionUpdatesAndHeldTriggerCannotResume() {
        seedPose();
        arm(); op.gamepad1.right_trigger = 1; op.loop();
        clearInvocations(turret, drive, aim);
        op.gamepad1.x = true; op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(aim).clear();
        op.gamepad1.x = false; op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).setAngleDegrees(35);
    }

    @Test public void panelsConfiguredPoseAloneDoesNotAuthorizeFieldCentricDrive() {
        HiveAutoAimTest.startPoseConfigured = true;
        HiveAutoAimTest.startX = 30; HiveAutoAimTest.startY = 40;
        arm();
        op.gamepad2.left_bumper = true;
        op.gamepad2.left_stick_y = -.5f;
        op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
        verify(telemetry).addData("Field half (estimate)", "UNKNOWN / UNKNOWN");
    }

    @Test public void explicitValidInitSeedSetsRadiansClearsAimAndAllowsDeadmanDrive() {
        seedPose();
        ArgumentCaptor<Pose> pose = ArgumentCaptor.forClass(Pose.class);
        verify(drive).setStartingPose(pose.capture());
        assertEquals(30, pose.getValue().x(), 0);
        assertEquals(40, pose.getValue().y(), 0);
        assertEquals(Math.PI / 2, pose.getValue().heading(), 1e-9);
        InOrder order = inOrder(drive, aim);
        order.verify(drive).setStartingPose(any(Pose.class));
        order.verify(aim).clear();
        arm();
        op.gamepad2.left_bumper = true;
        op.gamepad2.left_stick_y = -.5f;
        op.gamepad2.left_stick_x = .25f;
        op.gamepad2.right_stick_x = -.25f;
        op.loop();
        verify(drive).arcadeDrive(.5, .25, -.25, Alliance.RED);
        clearInvocations(drive);
        op.gamepad2.left_bumper = false; op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void invalidInitSeedNeverChangesThePose() {
        when(op.gamepad2.aWasPressed()).thenReturn(true);
        op.init_loop(); // No measured-pose confirmation.
        HiveAutoAimTest.startPoseConfigured = true;
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -.1, 144.1}) {
            HiveAutoAimTest.startX = invalid; HiveAutoAimTest.startY = 40;
            op.init_loop();
            HiveAutoAimTest.startX = 30; HiveAutoAimTest.startY = invalid;
            op.init_loop();
        }
        HiveAutoAimTest.startX = 30; HiveAutoAimTest.startY = 40;
        HiveAutoAimTest.startHeadingDegrees = Double.NaN; op.init_loop();
        HiveAutoAimTest.startHeadingDegrees = 0;
        when(drive.isLocalizationReady()).thenReturn(false); op.init_loop();
        verify(drive, never()).setStartingPose(any());
        arm(); op.gamepad2.left_bumper = true; op.loop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void rejectedReseedRevokesPriorFieldDriveAuthorization() {
        seedPose();
        HiveAutoAimTest.startPoseConfigured = false;
        when(op.gamepad2.aWasPressed()).thenReturn(true, false);
        op.init_loop();
        arm(); op.gamepad2.left_bumper = true; op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void initCalibrationClearsHistoryAndActivePoseButtonsNeverResetOdometry() {
        when(op.gamepad2.xWasPressed()).thenReturn(true, false);
        op.init_loop();
        verify(drive).recalibrateLocalization();
        verify(aim).clear();
        arm();
        HiveAutoAimTest.startPoseConfigured = true;
        when(op.gamepad2.xWasPressed()).thenReturn(true, false);
        when(op.gamepad2.aWasPressed()).thenReturn(true, false);
        op.loop(); op.loop();
        verify(drive, never()).recalibrateLocalization();
        verify(drive, never()).setStartingPose(any());
        verify(op.gamepad2, atLeastOnce()).xWasPressed();
        verify(op.gamepad2, atLeastOnce()).aWasPressed();
    }

    @Test public void pinpointHealthRecoveryDoesNotRestoreFieldDriveWithoutAnotherInitSeed() {
        seedPose();
        when(drive.isLocalizationReady()).thenReturn(false);
        op.init_loop();
        when(drive.isLocalizationReady()).thenReturn(true);
        op.init_loop();
        arm();
        op.gamepad2.left_bumper = true;
        op.gamepad2.left_stick_y = -.5f;
        op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
        verify(telemetry).addData("Field half (estimate)", "UNKNOWN / UNKNOWN");
        // A driver-station health recovery and an active-mode A press cannot silently reseed.
        when(op.gamepad2.aWasPressed()).thenReturn(true, false);
        op.loop();
        verify(drive, never()).setStartingPose(any());
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void crossingFieldHalvesNeverChangesSelectedAllianceOrCell() {
        seedPose(); arm();
        when(drive.getPose()).thenReturn(new Pose(120, 120, .5));
        op.loop();
        verify(aim).update(true, true, 120, 120, .5, 15, -90, 90,
                Alliance.RED, HiveCell.RED_AUDIENCE);
        verify(telemetry).addData("Field half (estimate)", "BLUE_HALF / FAR_HALF");
        assertEquals(Alliance.RED, HiveAutoAimTest.alliance);
        assertEquals(HiveCell.RED_AUDIENCE, HiveAutoAimTest.selectedCell);
        HiveAutoAimTest.alliance = Alliance.BLUE;
        HiveAutoAimTest.selectedCell = HiveCell.BLUE_FAR;
        op.loop();
        verify(aim).update(true, true, 120, 120, .5, 15, -90, 90,
                Alliance.BLUE, HiveCell.BLUE_FAR);
    }

    @Test public void forwardsCurrentMeasuredPoseAndExplicitFieldReferenceWithoutACamera() {
        arm(); op.loop();
        verify(aim).update(false, true, 30, 40, .25, 15, -90, 90,
                Alliance.RED, HiveCell.RED_AUDIENCE);
    }

    @Test public void automaticAimRequiresAnAppliedFieldPoseEvenIfTheSolverReportsAvailable() {
        arm(); op.gamepad1.right_trigger = 1; op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
    }

    @Test public void driverSelectsAudienceOrFarEndForTheConfiguredAlliance() {
        seedPose(); arm();
        when(op.gamepad1.dpadUpWasPressed()).thenReturn(true, false);
        op.loop();
        assertEquals(HiveCell.RED_FAR, HiveAutoAimTest.selectedCell);
        verify(aim).update(true, true, 30, 40, .25, 15, -90, 90, Alliance.RED, HiveCell.RED_FAR);
        HiveAutoAimTest.alliance = Alliance.BLUE;
        when(op.gamepad1.dpadDownWasPressed()).thenReturn(true, false);
        op.loop();
        assertEquals(HiveCell.BLUE_AUDIENCE, HiveAutoAimTest.selectedCell);
        verify(aim).update(true, true, 30, 40, .25, 15, -90, 90, Alliance.BLUE, HiveCell.BLUE_AUDIENCE);
    }

    @Test public void localizationFaultStopsAimUntilAnotherInitSeed() {
        seedPose(); arm(); op.gamepad1.right_trigger = 1; op.loop();
        clearInvocations(turret, drive, aim);
        when(drive.isLocalizationReady()).thenReturn(false);
        op.loop();
        verify(drive).periodic();
        verify(turret, never()).setAngleDegrees(anyDouble());
        when(drive.isLocalizationReady()).thenReturn(true);
        op.loop();
        verify(turret, never()).setAngleDegrees(anyDouble());
    }

    @Test public void nullAllianceBlocksFieldDriveAfterAValidSeed() {
        seedPose(); arm();
        HiveAutoAimTest.alliance = null;
        op.gamepad2.left_bumper = true; op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void servoFailureStopsDriveAndClosedOpModeCannotRestart() {
        seedPose(); arm();
        op.gamepad1.right_trigger = 1;
        doThrow(new IllegalStateException("servo write failed")).when(turret).setAngleDegrees(35);
        assertThrows(IllegalStateException.class, op::loop);
        verify(drive, atLeastOnce()).stop(); verify(aim).clear();
        clearInvocations(turret, drive, aim);
        op.start(); op.loop(); op.init_loop(); op.stop();
        verifyNoInteractions(turret, drive, aim);
    }

    @Test public void cleanupStillClearsAimWhenDriveStopThrows() {
        arm();
        doThrow(new IllegalStateException("drive write failed")).when(drive).stop();
        assertThrows(IllegalStateException.class, op::stop);
        verify(aim).clear();
        clearInvocations(turret, drive, aim);
        op.stop(); op.start(); op.loop();
        verifyNoInteractions(turret, drive, aim);
    }

    private void seedPose() {
        HiveAutoAimTest.startPoseConfigured = true;
        HiveAutoAimTest.startX = 30; HiveAutoAimTest.startY = 40;
        HiveAutoAimTest.startHeadingDegrees = 90;
        when(op.gamepad2.aWasPressed()).thenReturn(true, false);
        op.init_loop();
    }

    private void arm() {
        op.start(); op.loop();
        clearInvocations(turret, drive, aim, telemetry);
    }
}
