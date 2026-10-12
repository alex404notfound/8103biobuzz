package org.firstinspires.ftc.teamcode.math;

import org.junit.Test;
import static org.junit.Assert.*;

public class HiveBallGeometryTest {
    @Test public void opticalAxisIntersectsMeasuredBallHeightAtExpectedDistance() {
        HiveBallGeometry geometry = new HiveBallGeometry(15, 3, 45, 0, 0, 0, 120);
        HiveBallGeometry.Position position = geometry.project(0, 0);
        assertEquals(12, position.forwardInches, 1e-9);
        assertEquals(0, position.leftInches, 1e-9);
        assertEquals(12, position.cameraRangeInches, 1e-9);
    }

    @Test public void rightPositiveCameraAngleBecomesNegativeRobotLeftAndIncludesOffsets() {
        HiveBallGeometry geometry = new HiveBallGeometry(15, 3, 45, 0, 4, -2, 120);
        HiveBallGeometry.Position right = geometry.project(30, 0);
        HiveBallGeometry.Position left = geometry.project(-30, 0);
        assertEquals(16, right.forwardInches, 1e-9);
        assertEquals(-2 - 12 * Math.sqrt(2) / Math.sqrt(3), right.leftInches, 1e-9);
        assertEquals(-2 + 12 * Math.sqrt(2) / Math.sqrt(3), left.leftInches, 1e-9);
        assertTrue(right.cameraRangeInches > 12);
    }

    @Test public void cameraYawRotatesRayBeforeAddingRobotOffsets() {
        HiveBallGeometry.Position position = new HiveBallGeometry(15, 3, 45, 90, 4, -2, 120).project(0, 0);
        assertEquals(4, position.forwardInches, 1e-9);
        assertEquals(10, position.leftInches, 1e-9);
    }

    @Test public void upperImageTargetsAreFartherOnFloorAndOutOfRangeIsUnavailable() {
        HiveBallGeometry geometry = new HiveBallGeometry(15, 3, 45, 0, 0, 0, 30);
        assertTrue(geometry.project(0, 10).cameraRangeInches > geometry.project(0, -10).cameraRangeInches);
        assertNull(geometry.project(0, 30));
        assertNull(geometry.project(0, 45));
        assertNull(geometry.project(0, 60));
    }

    @Test public void rejectsImpossibleAndNonfiniteMeasurementsAndAngles() {
        assertThrows(IllegalArgumentException.class, () -> new HiveBallGeometry(3, 3, 45, 0, 0, 0, 120));
        assertThrows(IllegalArgumentException.class, () -> new HiveBallGeometry(15, 0, 45, 0, 0, 0, 120));
        assertThrows(IllegalArgumentException.class, () -> new HiveBallGeometry(15, 3, 90, 0, 0, 0, 120));
        assertThrows(IllegalArgumentException.class, () -> new HiveBallGeometry(15, 3, 45, Double.NaN, 0, 0, 120));
        HiveBallGeometry geometry = new HiveBallGeometry(15, 3, 45, 0, 0, 0, 120);
        assertNull(geometry.project(Double.NaN, 0));
        assertNull(geometry.project(0, Double.POSITIVE_INFINITY));
        assertNull(geometry.project(90, 0));
    }
}
