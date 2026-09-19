package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HiveAutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.InOrder;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class HiveAutoAimOpModeTest {
    private final AxonTurret turret = mock(AxonTurret.class);
    private final Drivetrain drive = mock(Drivetrain.class);
    private final Limelight vision = mock(Limelight.class);
    private final HiveAutoAim aim = mock(HiveAutoAim.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private HiveAutoAimTest op;

    @Before public void setUp() {
        defaults();
        when(drive.getPose()).thenReturn(new Pose(30, 40, .25));
        when(drive.getVelocity()).thenReturn(new Velocity(0, 0, 0));
        when(drive.isLocalizationReady()).thenReturn(true);
        when(turret.hasReference()).thenReturn(true);
        when(turret.getPositionDegrees()).thenReturn(15.0);
        when(aim.hasAim()).thenReturn(true);
        when(aim.getTurretGoalDegrees()).thenReturn(35.0);
        op = new HiveAutoAimTest(turret, drive, vision, aim, telemetry);
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
        op.gamepad1.right_trigger = 1;
        op.init_loop(); op.loop();
        verify(turret, never()).enable();
        verify(turret, never()).requestPosition(anyDouble());
        op.start(); op.loop();
        verify(turret).enable();
        verify(turret, never()).requestPosition(anyDouble());
        op.gamepad1.right_trigger = 0;
        op.gamepad2.left_bumper = true;
        op.loop();
        op.gamepad1.right_trigger = 1;
        op.gamepad2.left_bumper = false;
        op.loop();
        verify(turret, never()).requestPosition(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).requestPosition(35);
    }

    @Test public void holdAimUsesLatestGoalAndStopsTheChassisEvenWithDriveDeadmanHeld() {
        seedPose(); arm();
        op.gamepad2.left_bumper = true;
        op.gamepad2.left_stick_y = -1;
        op.gamepad1.right_trigger = 1;
        op.loop();
        verify(turret).requestPosition(35);
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
        when(aim.getTurretGoalDegrees()).thenReturn(-20.0);
        op.loop();
        verify(turret).requestPosition(-20);
    }

    @Test public void movingOrUnhealthyChassisPreventsAnOtherwiseAvailableAim() {
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
        verify(turret, never()).requestPosition(anyDouble());
        verify(turret, atLeastOnce()).idle();
    }

    @Test public void invalidOrExcessiveStationarityThresholdsDisableAim() {
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
        verify(turret, never()).requestPosition(anyDouble());
        verify(turret, atLeastOnce()).idle();
    }

    @Test public void losingUsableVisionIdlesRatherThanReusingTheLastGoal() {
        arm(); op.gamepad1.right_trigger = 1;
        op.loop();
        verify(turret).requestPosition(35);
        clearInvocations(turret, drive);
        when(aim.hasAim()).thenReturn(false);
        op.loop();
        verify(turret, atLeastOnce()).idle();
        verify(turret, never()).requestPosition(anyDouble());
        verify(drive, atLeastOnce()).stop();
    }

    @Test public void manualTurretJogOverridesAutoAimWithItsOwnDeadmanAndDeadband() {
        arm();
        op.gamepad1.right_trigger = 1;
        op.gamepad1.left_bumper = true;
        op.gamepad1.left_stick_x = -.5f;
        op.loop();
        verify(turret).requestManual(-.5);
        verify(turret, never()).requestPosition(anyDouble());
        op.gamepad1.left_stick_x = .05f;
        op.loop();
        verify(turret).requestManual(0);
        op.gamepad1.left_bumper = false;
        op.gamepad1.right_trigger = 0;
        op.gamepad1.left_stick_x = 0;
        op.loop();
        verify(turret, atLeastOnce()).idle();
    }

    @Test public void cancelRemovesPowerBeforeSamplingAndHeldTriggerCannotResume() {
        arm(); op.gamepad1.right_trigger = 1; op.loop();
        clearInvocations(turret, drive, aim);
        op.gamepad1.x = true; op.loop();
        InOrder order = inOrder(turret);
        order.verify(turret).idle();
        order.verify(turret).periodic();
        verify(drive, atLeastOnce()).stop();
        verify(aim).clear();
        op.gamepad1.x = false; op.loop();
        verify(turret, never()).requestPosition(anyDouble());
        op.gamepad1.right_trigger = 0; op.loop();
        op.gamepad1.right_trigger = 1; op.loop();
        verify(turret).requestPosition(35);
    }

    @Test public void dashboardConfiguredPoseAloneDoesNotAuthorizeFieldCentricDrive() {
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
        verify(aim).update(null, 0, true, 120, 120, .5, true, 15,
                Alliance.RED, HiveCell.RED_AUDIENCE);
        verify(telemetry).addData("Field half (estimate)", "BLUE_HALF / FAR_HALF");
        assertEquals(Alliance.RED, HiveAutoAimTest.alliance);
        assertEquals(HiveCell.RED_AUDIENCE, HiveAutoAimTest.selectedCell);
        HiveAutoAimTest.alliance = Alliance.BLUE;
        HiveAutoAimTest.selectedCell = HiveCell.BLUE_FAR;
        op.loop();
        verify(aim).update(null, 0, true, 120, 120, .5, true, 15,
                Alliance.BLUE, HiveCell.BLUE_FAR);
    }

    @Test public void forwardsExposureAgeAndCurrentMeasuredPoseToAim() {
        LLResult frame = mock(LLResult.class);
        when(vision.getFreshResult()).thenReturn(frame);
        when(vision.getFreshResultAgeMs()).thenReturn(62.5);
        arm();
        op.loop();
        verify(aim).update(frame, 62.5, true, 30, 40, .25, true, 15,
                Alliance.RED, HiveCell.RED_AUDIENCE);
    }

    @Test public void zeroRequestIsConsumedByASensorSampleBeforeAnotherIdleCanCancelIt() {
        arm();
        when(op.gamepad1.bWasPressed()).thenReturn(true, false);
        op.loop();
        InOrder order = inOrder(turret);
        order.verify(turret).idle();
        order.verify(turret).zeroHere();
        order.verify(turret).periodic();
        List<String> controlCalls = new ArrayList<>();
        mockingDetails(turret).getInvocations().forEach(invocation -> {
            String name = invocation.getMethod().getName();
            if (name.equals("idle") || name.equals("zeroHere") || name.equals("periodic")) controlCalls.add(name);
        });
        assertEquals("periodic", controlCalls.get(controlCalls.indexOf("zeroHere") + 1));
        verify(aim).clear();
        op.loop();
        verify(turret).zeroHere();
    }

    @Test public void referenceAndFaultButtonsDuringMotionAreConsumedWithoutDelayedAction() {
        arm();
        when(op.gamepad1.bWasPressed()).thenReturn(true, false);
        when(op.gamepad1.yWasPressed()).thenReturn(true, false);
        op.gamepad1.right_trigger = 1; op.loop();
        op.gamepad1.right_trigger = 0; op.loop();
        verify(turret, never()).zeroHere();
        verify(turret, never()).clearFault();
        when(op.gamepad1.yWasPressed()).thenReturn(true, false);
        op.loop();
        verify(turret).clearFault();
        verify(aim).clear();
    }

    @Test public void nullAllianceBlocksFieldDriveAfterAValidSeed() {
        seedPose(); arm();
        HiveAutoAimTest.alliance = null;
        op.gamepad2.left_bumper = true; op.loop();
        verify(drive, atLeastOnce()).stop();
        verify(drive, never()).arcadeDrive(anyDouble(), anyDouble(), anyDouble(), any());
    }

    @Test public void sensorFailureStopsAllHardwareAndClosedOpModeCannotRestart() {
        arm();
        doThrow(new IllegalStateException("encoder disconnected")).when(turret).periodic();
        assertThrows(IllegalStateException.class, op::loop);
        verify(turret).stop(); verify(drive, atLeastOnce()).stop(); verify(vision).stop(); verify(aim).clear();
        clearInvocations(turret, drive, vision, aim);
        op.start(); op.loop(); op.init_loop(); op.stop();
        verifyNoInteractions(turret, drive, vision, aim);
    }

    @Test public void cleanupStillStopsDriveAndVisionWhenTurretStopThrows() {
        arm();
        doThrow(new IllegalStateException("servo write failed")).when(turret).stop();
        assertThrows(IllegalStateException.class, op::stop);
        verify(drive, atLeastOnce()).stop(); verify(vision).stop(); verify(aim).clear();
        clearInvocations(turret, drive, vision, aim);
        op.stop(); op.start(); op.loop();
        verifyNoInteractions(turret, drive, vision, aim);
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
        clearInvocations(turret, drive, vision, aim, telemetry);
    }
}
