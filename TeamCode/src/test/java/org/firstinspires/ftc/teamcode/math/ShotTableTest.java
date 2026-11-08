package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.junit.Test;
import java.util.Arrays;
import java.util.Collections;
import static org.junit.Assert.*;

public class ShotTableTest {
    private ShotTable.Point point(double distance, double rpm) {
        HiveCell cell = HiveCell.RED_AUDIENCE;
        return new ShotTable.Point(new Pose(ShotTable.targetX(cell) + distance, ShotTable.targetY(cell), 0), cell, rpm);
    }
    @Test public void sortsMeasurementsAndInterpolatesOnlyTheTwoSurroundingDistances() {
        ShotTable table = new ShotTable(Arrays.asList(point(60, 2600), point(20, 1000), point(40, 2000)));
        assertEquals(20, table.points().get(0).distance, 0);
        ShotTable.Lookup between = table.lookup(50);
        assertTrue(between.usable);
        assertEquals(2300, between.rpm, 1e-9);
        assertEquals(40, between.lowerDistance, 0);
        assertEquals(60, between.upperDistance, 0);
        assertEquals(.5, between.fraction, 1e-9);
        assertEquals(1500, table.lookup(30).rpm, 1e-9);
        assertEquals(1000, table.lookup(20).rpm, 0);
        assertEquals(2600, table.lookup(60).rpm, 0);
    }
    @Test public void emptySinglePointAndOutOfRangeNeverProduceARunnableRpm() {
        assertFalse(new ShotTable().lookup(20).usable);
        assertFalse(new ShotTable().with(point(20, 1000)).lookup(20).usable);
        ShotTable table = new ShotTable().with(point(20, 1000)).with(point(40, 2000));
        for (double invalid : new double[]{19.9, 40.1, -1, 0, Double.NaN, Double.POSITIVE_INFINITY}) {
            ShotTable.Lookup lookup = table.lookup(invalid);
            assertFalse(lookup.usable); assertTrue(Double.isNaN(lookup.rpm));
        }
    }
    @Test public void retuningNearbyDistanceReplacesMeasurementWithoutMutatingOriginal() {
        ShotTable original = new ShotTable().with(point(20, 1000)).with(point(40, 2000));
        ShotTable replaced = original.with(point(20.5, 1200));
        assertEquals(2, replaced.points().size());
        assertEquals(20.5, replaced.points().get(0).distance, 0);
        assertEquals(1200, replaced.points().get(0).rpm, 0);
        assertEquals(1000, original.lookup(20).rpm, 0);
        assertThrows(UnsupportedOperationException.class, () -> original.points().clear());
    }
    @Test public void duplicateDistancesAndOversizedTablesAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new ShotTable(Arrays.asList(point(20, 1000), point(21, 1200))));
        assertThrows(IllegalArgumentException.class, () -> new ShotTable(Collections.nCopies(101, point(20, 1000))));
    }
    @Test public void samplesRejectInvalidPositionsCellsAndRpm() {
        for (double invalid : new double[]{0, -1, 6001, Double.NaN, Double.POSITIVE_INFINITY})
            assertThrows(IllegalArgumentException.class, () -> point(20, invalid));
        assertThrows(IllegalArgumentException.class, () -> point(0, 1000));
        for (Pose invalid : new Pose[]{null, new Pose(-1, 20, 0), new Pose(20, 145, 0),
                new Pose(20, 20, Double.NaN), new Pose(Double.NaN, 0, 0)})
            assertThrows(IllegalArgumentException.class, () -> new ShotTable.Point(invalid, HiveCell.RED_AUDIENCE, 1000));
        assertThrows(IllegalArgumentException.class, () -> new ShotTable.Point(new Pose(20, 20, 0), null, 1000));
    }
    @Test public void rangeUsesSelectedCellAndRobotCenterIndependentOfHeading() {
        for (HiveCell cell : HiveCell.values()) {
            Pose pose = new Pose(ShotTable.targetX(cell) + 30, ShotTable.targetY(cell) + 40, 0);
            assertEquals(50, ShotTable.range(pose, cell), 1e-9);
            assertEquals(50, ShotTable.range(new Pose(pose.x(), pose.y(), Math.PI), cell), 1e-9);
        }
        assertTrue(Double.isNaN(ShotTable.range(null, HiveCell.RED_AUDIENCE)));
    }
}
