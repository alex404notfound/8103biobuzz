package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class HiveTagAimTest {
    private final AtomicLong now = new AtomicLong();
    private HiveTagAim aim;

    @Before public void setUp() { defaults(); aim = new HiveTagAim(2, now::get); }
    @After public void tearDown() { defaults(); }

    private static void defaults() {
        HiveTagAim.maxFrameAgeMs = 100; HiveTagAim.minimumArea = 0.02;
        HiveTagAim.targetTxDegrees = 0; HiveTagAim.turnKp = 0.015; HiveTagAim.maximumTurnPower = 0.18;
        HiveTagAim.turnSign = 1; HiveTagAim.toleranceDegrees = 1; HiveTagAim.alignedDwellMs = 200;
        HiveTagAim.verticalStabilityDegrees = 1.5;
    }

    @Test public void selectsExactIdAndNeverUsesTheGlobalOrLargestTarget() {
        LLResult result = frame(1, tag(38, -25, 0), tag(34, 5, 0), tag(35, -10, 0));
        when(result.getTx()).thenReturn(-25.0);
        aim.update(result, Alliance.RED, 34);
        assertEquals(0.075, aim.getTurnPower(), 1e-9);
        assertEquals(5, aim.getTx(), 0);
        aim.update(result, Alliance.RED, 33);
        assertFalse(aim.hasTarget());
        assertEquals(0, aim.getTurnPower(), 0);
    }

    @Test public void opponentInvalidAndWrongPipelineTargetsCannotDrive() {
        LLResult red = frame(1, tag(34, 10, 0));
        aim.update(red, Alliance.BLUE, 34);
        assertFalse(aim.hasTarget());
        aim.update(frame(2, tag(2, 10, 0)), Alliance.RED, 2);
        assertFalse(aim.hasTarget());
        when(red.getPipelineIndex()).thenReturn(3);
        aim.update(red, Alliance.RED, 34);
        assertEquals(0, aim.getTurnPower(), 0);
    }

    @Test public void frameAgeIsRecheckedAtConsumptionEvenWhenCameraKeepsReturningSameFrame() {
        aim.update(frame(1, tag(34, 15, 0)), Alliance.RED, 34);
        assertEquals(0.18, aim.getTurnPower(), 0);
        now.set(101_000_000L);
        assertEquals(0, aim.getTurnPower(), 0);
        aim.update(frame(1, tag(34, 15, 0)), Alliance.RED, 34);
        assertFalse(aim.hasTarget());
        aim.update(frame(2, tag(34, 15, 0)), Alliance.RED, 34);
        assertTrue(aim.hasTarget());
    }

    @Test public void alignmentRequiresDistinctFramesAndObservedTimeAndResetsForCellMotion() {
        sample(0, 1, 0, 0);
        sample(50, 1, 0, 0);
        sample(100, 1, 0, 0);
        assertFalse(aim.isAligned());
        sample(100, 2, 0, 0);
        sample(150, 3, 0, 0);
        sample(200, 4, 0, 0);
        assertTrue(aim.isAligned());
        sample(225, 5, 0, 5);
        assertFalse(aim.isAligned());
        sample(250, 6, 3, 5);
        assertFalse(aim.isAligned());
    }

    @Test public void changingTargetOrSetpointClearsAlignmentEvenInSameCameraFrame() {
        sample(0, 1, 0, 0); sample(100, 2, 0, 0); sample(200, 3, 0, 0);
        assertTrue(aim.isAligned());
        HiveTagAim.targetTxDegrees = 2;
        assertEquals(0, aim.getTurnPower(), 0); // Old observation cannot use new setpoint.
        aim.update(frame(3, tag(35, 2, 0)), Alliance.RED, 35);
        assertFalse(aim.isAligned());
        assertTrue(aim.hasTarget());
    }

    @Test public void rejectsBadQualityAndNonfiniteValuesAndBoundsBothTurnDirections() {
        aim.update(frame(1, tag(34, Double.NaN, 0)), Alliance.RED, 34);
        assertFalse(aim.hasTarget());
        FiducialResult tiny = tag(34, 1, 0);
        when(tiny.getTargetArea()).thenReturn(0.001);
        aim.update(frame(2, tiny), Alliance.RED, 34);
        assertFalse(aim.hasTarget());
        aim.update(frame(3, tag(34, -30, 0)), Alliance.RED, 34);
        assertEquals(-0.18, aim.getTurnPower(), 0);
        HiveTagAim.turnSign = -1;
        assertEquals(0.18, aim.getTurnPower(), 0);
        HiveTagAim.maximumTurnPower = Double.NaN;
        assertEquals(0, aim.getTurnPower(), 0);
    }

    @Test public void rangeUsesThreeDimensionalSlantDistanceAndMissingPoseIsUnknown() {
        FiducialResult target = tag(34, 0, 0);
        Pose3D pose = mock(Pose3D.class);
        when(pose.getPosition()).thenReturn(new Position(DistanceUnit.METER, 3, 4, 0, 0));
        when(target.getTargetPoseCameraSpace()).thenReturn(pose);
        aim.update(frame(1, target), Alliance.RED, 34);
        assertEquals(5 * 100 / 2.54, aim.getRangeInches(), 1e-6);
        when(pose.getPosition()).thenReturn(new Position(DistanceUnit.METER, 0, 0, 0, 0));
        aim.update(frame(2, target), Alliance.RED, 34);
        assertTrue(Double.isNaN(aim.getRangeInches()));
    }

    @Test public void allOfficialIdsMapToTheirCellAndUnrelatedIdsDoNot() {
        for (HiveCell cell : HiveCell.values())
            for (int id = cell.firstTagId; id < cell.firstTagId + 4; id++) assertSame(cell, HiveCell.fromTagId(id));
        assertEquals(Alliance.RED, HiveCell.fromTagId(30).alliance);
        assertSame(HiveCell.BLUE_AUDIENCE, HiveCell.fromTagId(41));
        assertSame(HiveCell.BLUE_FAR, HiveCell.fromTagId(45));
        assertNull(HiveCell.fromTagId(29)); assertNull(HiveCell.fromTagId(46));
    }

    private void sample(long ms, double timestamp, double tx, double ty) {
        now.set(ms * 1_000_000L);
        aim.update(frame(timestamp, tag(34, tx, ty)), Alliance.RED, 34);
    }

    private static LLResult frame(double timestamp, FiducialResult... tags) {
        LLResult result = VisionResultGateTest.result(timestamp, 2);
        when(result.getFiducialResults()).thenReturn(Arrays.asList(tags));
        return result;
    }

    private static FiducialResult tag(int id, double tx, double ty) {
        FiducialResult target = mock(FiducialResult.class);
        when(target.getFiducialId()).thenReturn(id);
        when(target.getTargetXDegrees()).thenReturn(tx);
        when(target.getTargetYDegrees()).thenReturn(ty);
        when(target.getTargetArea()).thenReturn(1.0);
        return target;
    }
}
