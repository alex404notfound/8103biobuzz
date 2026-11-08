package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;

public class HiveAutoAimTest {
    private final AtomicLong now = new AtomicLong();
    private final Map<Field, Object> originalSettings = new LinkedHashMap<>();
    private HiveAutoAim aim;
    private double minDegrees = -180, maxDegrees = 180;

    @Before public void setUp() throws ReflectiveOperationException {
        for (Field field : HiveAutoAim.class.getFields()) {
            if (Modifier.isStatic(field.getModifiers()) && !Modifier.isFinal(field.getModifiers()))
                originalSettings.put(field, field.get(null));
        }
        HiveAutoAim.redHiveX = 59.25; HiveAutoAim.blueHiveX = 84.75;
        HiveAutoAim.audienceCellY = 56.2; HiveAutoAim.farCellY = 87.8;
        HiveAutoAim.pivotForward = HiveAutoAim.pivotLeft = HiveAutoAim.turretZeroYawDegrees = 0;
        HiveAutoAim.shooterForward = HiveAutoAim.shooterLeft = HiveAutoAim.shooterYawDegrees = 0;
        HiveAutoAim.minimumRangeInches = 12; HiveAutoAim.maximumRangeInches = 180;
        HiveAutoAim.minimumApproachDegrees = 30; HiveAutoAim.maximumPoseAgeMs = 100;
        aim = new HiveAutoAim(now::get);
    }

    @After public void tearDown() throws IllegalAccessException {
        for (Map.Entry<Field, Object> entry : originalSettings.entrySet()) entry.getKey().set(null, entry.getValue());
    }

    @Test public void nominalFieldMapUsesRaisedOpeningsAndBothAllianceSymmetries() {
        assertEquals(25.5, FieldConstants.HIVE_CENTER_SPACING_INCHES, 0);
        assertEquals(30, FieldConstants.HIVE_STABLE_TILT_DEGREES, 0);
        assertEquals(59.25, FieldConstants.RED_HIVE_X_INCHES, 0);
        assertEquals(84.75, FieldConstants.BLUE_HIVE_X_INCHES, 0);
        assertEquals(56.2, FieldConstants.AUDIENCE_CELL_UP_Y_INCHES, 1e-9);
        assertEquals(87.8, FieldConstants.FAR_CELL_UP_Y_INCHES, 1e-9);
        for (HiveCell cell : HiveCell.values()) {
            double x = cell.alliance == Alliance.RED ? 59.25 : 84.75;
            double targetY = cell.audienceSide ? 56.2 : 87.8;
            sample(x, cell.audienceSide ? 20 : 124, cell.audienceSide ? 90 : -90, 0, cell);
            assertTrue(aim.getStatus(), aim.hasAim());
            assertEquals(x, aim.getTargetXInches(), 0);
            assertEquals(targetY, aim.getTargetYInches(), 0);
            assertEquals(0, aim.getTurretGoalDegrees(), 1e-9);
            assertEquals(36.2, aim.getRangeInches(), 1e-9);
            assertEquals(90, aim.getApproachDegrees(), 1e-9);
        }
    }

    @Test public void headingIsSubtractedFromFieldBearingInCounterclockwiseDegrees() {
        for (double heading : new double[]{0, 90, 180, 270, 450, -90}) {
            sample(59.25, 20, heading, 0, HiveCell.RED_AUDIENCE);
            assertTrue(aim.getStatus(), aim.hasAim());
            assertEquals(90, aim.getFieldBearingDegrees(), 1e-9);
            assertEquals(Math.IEEEremainder(90 - heading, 360), aim.getTurretGoalDegrees(), 1e-9);
        }
    }

    @Test public void translationUpdatesTheBearingWithoutAnyVisionObservationOrDwell() {
        sample(29.25, 26.2, 0, 0, HiveCell.RED_AUDIENCE);
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(45, aim.getTurretGoalDegrees(), 1e-9);
        now.set(20_000_000L);
        sample(89.25, 26.2, 0, 0, HiveCell.RED_AUDIENCE);
        assertEquals(135, aim.getTurretGoalDegrees(), 1e-9);
        assertEquals(Math.hypot(30, 30), aim.getRangeInches(), 1e-9);
    }

    @Test public void blueRotationPreservesTheRobotRelativeAimAngle() {
        sample(29.25, 26.2, 20, 0, HiveCell.RED_AUDIENCE);
        double redGoal = aim.getTurretGoalDegrees();
        sample(114.75, 117.8, 200, 0, HiveCell.BLUE_FAR);
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(redGoal, aim.getTurretGoalDegrees(), 1e-9);
    }

    @Test public void pivotOffsetRotatesWithTheChassisAndZeroYawIsRemoved() {
        HiveAutoAim.pivotForward = 5; HiveAutoAim.pivotLeft = 2; HiveAutoAim.turretZeroYawDegrees = 10;
        sample(59.25, 20, 90, 0, HiveCell.RED_AUDIENCE); // Pivot = (57.25,25), delta = (2,31.2).
        assertTrue(aim.getStatus(), aim.hasAim());
        assertEquals(Math.toDegrees(Math.atan2(-2, 31.2)) - 10, aim.getTurretGoalDegrees(), 1e-9);
    }

    @Test public void offsetMuzzleAndBoreRayPassThroughTheMappedTarget() {
        HiveAutoAim.pivotForward = 3; HiveAutoAim.pivotLeft = -2;
        HiveAutoAim.shooterForward = 5; HiveAutoAim.shooterLeft = 4;
        HiveAutoAim.shooterYawDegrees = 12; HiveAutoAim.turretZeroYawDegrees = 17;
        sample(29.25, 16.2, 35, 0, HiveCell.RED_AUDIENCE);
        assertTrue(aim.getStatus(), aim.hasAim());
        double chassis = Math.toRadians(35), turret = Math.toRadians(35 + aim.getTurretGoalDegrees() + 17);
        double muzzleX = 29.25 + 3 * Math.cos(chassis) + 2 * Math.sin(chassis)
                + 5 * Math.cos(turret) - 4 * Math.sin(turret);
        double muzzleY = 16.2 + 3 * Math.sin(chassis) - 2 * Math.cos(chassis)
                + 5 * Math.sin(turret) + 4 * Math.cos(turret);
        double ray = turret + Math.toRadians(12), dx = 59.25 - muzzleX, dy = 56.2 - muzzleY;
        assertEquals(0, dx * Math.sin(ray) - dy * Math.cos(ray), 1e-9);
        assertTrue(dx * Math.cos(ray) + dy * Math.sin(ray) > 0);
        assertEquals(Math.hypot(dx, dy), aim.getRangeInches(), 1e-9);
    }

    @Test public void unknownFieldOriginCannotAimEvenWithHealthyPinpointAndCalibratedTurret() {
        aim.update(false, true, 59.25, 20, 0, 0, minDegrees, maxDegrees, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        assertTrue(aim.getStatus().contains("field pose"));
    }

    @Test public void missingCalibrationOrPinpointHealthImmediatelyInvalidatesTheLastGoal() {
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        assertTrue(aim.hasAim());
        aim.update(true, false, 59.25, 20, 0, 0, minDegrees, maxDegrees, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        assertTrue(Double.isNaN(aim.getTurretGoalDegrees()));
        aim.update(true, true, 59.25, 20, 0, 0, Double.NaN, Double.NaN, Alliance.RED, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
    }

    @Test public void onlySelectedAllianceCellCanBeUsedAndSwitchingNeverReusesOldTarget() {
        for (HiveCell cell : new HiveCell[]{HiveCell.BLUE_AUDIENCE, HiveCell.BLUE_FAR, null}) {
            aim.update(true, true, 59.25, 20, 0, 0, minDegrees, maxDegrees, Alliance.RED, cell);
            assertFalse(aim.hasAim());
        }
        aim.update(true, true, 59.25, 20, 0, 0, minDegrees, maxDegrees, null, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        assertTrue(aim.hasAim());
        sample(59.25, 20, 0, 0, HiveCell.RED_FAR); // Robot is behind the newly selected far mouth.
        assertFalse(aim.hasAim());
        assertEquals(87.8, aim.getTargetYInches(), 0);
    }

    @Test public void poseAndSettingsAreRecheckedWhenTheSolutionIsConsumed() {
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        now.set(101_000_000L);
        assertFalse(aim.hasAim());
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        HiveAutoAim.redHiveX += 1;
        assertFalse(aim.hasAim());
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        assertTrue(aim.hasAim());
        HiveAutoAim.turretZeroYawDegrees = 10;
        assertFalse(aim.hasAim());
    }

    @Test public void clearAndClockReversalInvalidateAim() {
        now.set(20_000_000L);
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        now.set(19_000_000L);
        assertFalse(aim.hasAim());
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        aim.clear();
        assertFalse(aim.hasAim());
        assertTrue(Double.isNaN(aim.getCandidateGoalDegrees()));
    }

    @Test public void targetMustBeInFrontOfTheMouthAndWithinTestedRange() {
        sample(59.25, 80, 0, 0, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim()); assertEquals(-90, aim.getApproachDegrees(), 1e-9);
        sample(20, 56.2, 0, 0, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim()); assertEquals(0, aim.getApproachDegrees(), 1e-9);
        sample(59.25, 50, 0, 0, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        HiveAutoAim.maximumRangeInches = 30;
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
    }

    @Test public void limitedTravelRejectsUnreachableAnglesAndUsesNearestAllowedTurn() {
        minDegrees = -45; maxDegrees = 45;
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
        maxDegrees = 500;
        sample(59.25, 20, 0, 440, HiveCell.RED_AUDIENCE);
        assertEquals(450, aim.getTurretGoalDegrees(), 1e-9);
    }

    @Test public void nonfiniteAndOutOfFieldPoseCannotReachTheTurret() {
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -.1, 144.1}) {
            sample(bad, 20, 0, 0, HiveCell.RED_AUDIENCE); assertFalse(aim.hasAim());
            sample(59.25, bad, 0, 0, HiveCell.RED_AUDIENCE); assertFalse(aim.hasAim());
        }
        sample(59.25, 20, Double.NaN, 0, HiveCell.RED_AUDIENCE); assertFalse(aim.hasAim());
        sample(59.25, 20, 0, Double.NaN, HiveCell.RED_AUDIENCE); assertFalse(aim.hasAim());
    }

    @Test public void invalidMapGeometryAndLimitsDisableAim() {
        HiveAutoAim.redHiveX = Double.NaN; assertNoAim();
        HiveAutoAim.redHiveX = 145; assertNoAim();
        HiveAutoAim.redHiveX = 85; assertNoAim();
        HiveAutoAim.redHiveX = 59.25;
        HiveAutoAim.audienceCellY = 90; assertNoAim();
        HiveAutoAim.audienceCellY = 56.2;
        HiveAutoAim.pivotForward = Double.NaN; assertNoAim();
        HiveAutoAim.pivotForward = 0;
        HiveAutoAim.maximumPoseAgeMs = 0; assertNoAim();
        HiveAutoAim.maximumPoseAgeMs = 251; assertNoAim();
        HiveAutoAim.maximumPoseAgeMs = 100;
        HiveAutoAim.minimumApproachDegrees = 0; assertNoAim();
        HiveAutoAim.minimumApproachDegrees = 91; assertNoAim();
        HiveAutoAim.minimumApproachDegrees = 30;
        HiveAutoAim.minimumRangeInches = -1; assertNoAim();
    }

    private void assertNoAim() {
        sample(59.25, 20, 0, 0, HiveCell.RED_AUDIENCE);
        assertFalse(aim.hasAim());
    }
    private void sample(double x, double y, double headingDegrees, double turretDegrees, HiveCell cell) {
        aim.update(true, true, x, y, Math.toRadians(headingDegrees), turretDegrees, minDegrees, maxDegrees,
                cell.alliance, cell);
    }
}
