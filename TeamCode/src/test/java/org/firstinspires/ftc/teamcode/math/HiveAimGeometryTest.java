package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.math.HiveAimGeometry.Vec;
import org.junit.Test;

import static org.firstinspires.ftc.teamcode.math.HiveAimGeometry.*;
import static org.junit.Assert.*;

public class HiveAimGeometryTest {
    @Test public void vectorArithmeticPreservesInputsAndUnits() {
        Vec a = new Vec(3, 4, 12), b = new Vec(-2, 5, 1);
        assertVec(1, 9, 13, a.plus(b));
        assertVec(5, -1, 11, a.minus(b));
        assertVec(6, 8, 24, a.scale(2));
        assertVec(3, 4, 12, a);
        assertEquals(13, a.norm(), 1e-9);
    }

    @Test public void cardinalRotationsFollowRightHandedAxes() {
        assertVec(0, 0, 1, rotate(new Vec(0, 1, 0), 90, 0, 0));
        assertVec(0, 0, -1, rotate(new Vec(1, 0, 0), 0, 90, 0));
        assertVec(0, 1, 0, rotate(new Vec(1, 0, 0), 0, 0, 90));
    }

    @Test public void combinedRotationsApplyExtrinsicXThenYThenZ() {
        // X turns +Y into +Z, then Y turns +Z into +X, then Z turns +X into +Y.
        assertVec(0, 1, 0, rotate(new Vec(0, 1, 0), 90, 90, 90));
        Vec v = new Vec(2, -3, 7);
        Vec combined = rotate(v, 23, -42, 76);
        Vec separate = rotate(rotate(rotate(v, 23, 0, 0), 0, -42, 0), 0, 0, 76);
        assertVec(separate.x, separate.y, separate.z, combined);
        assertEquals(v.norm(), combined.norm(), 1e-9);
    }

    @Test public void opticalAxesMapToForwardLeftUpBeforeMountingRotation() {
        assertVec(30, -10, -20, opticalToTurret(new Vec(10, 20, 30), 0, 0, 0));
        assertVec(0, 10, 0, opticalToTurret(new Vec(0, 0, 10), 0, 0, 90));
        assertVec(Math.sqrt(75), 0, -5, opticalToTurret(new Vec(0, 0, 10), 0, 30, 0));
        assertVec(Math.sqrt(75), 0, 5, opticalToTurret(new Vec(0, 0, 10), 0, -30, 0));
    }

    @Test public void allOfficialTagIdsResolveTheirOwnInchOffsets() {
        double[] offsets = {-6.5, -2.75, 2.75, 6.5};
        for (int id = 30; id <= 45; id++) {
            Vec cameraTag = new Vec(8 + offsets[(id - 30) % 4], -12, 72);
            assertVec(8, -12, 72, tagCenterFromTag(cameraTag, 0, 0, 0, id));
        }
    }

    @Test public void rotatedTagRowUsesPoseRotationRatherThanImageMidpoint() {
        // The row extends along optical depth at 90 degrees around Y: the tags have unequal range.
        assertVec(4, -8, 60, tagCenterFromTag(new Vec(4, -8, 66.5), 0, 90, 0, 30));
        assertVec(4, -8, 60, tagCenterFromTag(new Vec(4, -8, 53.5), 0, 90, 0, 33));
        assertVec(4, -8, 60, tagCenterFromTag(new Vec(4, -14.5, 60), 0, 0, 90, 30));
    }

    @Test public void tagUpKeepsSignedInclinationWhereUnsignedNormalCannot() {
        Vec upState = opticalToTurret(tagUp(60, 0, 0), 0, 0, 0);
        Vec downState = opticalToTurret(tagUp(120, 0, 0), 0, 0, 0);
        assertEquals(0.5, upState.z, 1e-9);
        assertEquals(-0.5, downState.z, 1e-9);
        Vec upNormal = opticalToTurret(tagNormal(60, 0, 0), 0, 0, 0);
        Vec downNormal = opticalToTurret(tagNormal(120, 0, 0), 0, 0, 0);
        assertEquals(Math.abs(upNormal.z), Math.abs(downNormal.z), 1e-9);
        assertEquals(1, upState.norm(), 1e-9);
        assertEquals(1, upNormal.norm(), 1e-9);
    }

    @Test public void continuousTurretUsesNearestUnwrappedEquivalent() {
        assertEquals(181, nearestTurretDegrees(-179, 179, false, 0, 0), 1e-9);
        assertEquals(-181, nearestTurretDegrees(179, -179, false, 0, 0), 1e-9);
        assertEquals(725, nearestTurretDegrees(5, 720, false, Double.NaN, Double.NaN), 1e-9);
        assertEquals(725, nearestTurretDegrees(1085, 720, false, 0, 0), 1e-9);
    }

    @Test public void centeredMuzzleUsesTargetBearingMinusBoreYaw() {
        assertEquals(45, turretBearingDegrees(new Vec(10, 10, 40), 0, 0, 0), 1e-9);
        assertEquals(25, turretBearingDegrees(new Vec(10, 10, -40), 0, 0, 20), 1e-9);
        assertEquals(-150, turretBearingDegrees(new Vec(-10, 0, 0), 0, 0, -30), 1e-9);
    }

    @Test public void offsetMuzzleRayActuallyPassesThroughTarget() {
        assertEquals(Math.toDegrees(Math.asin(-0.06)),
                turretBearingDegrees(new Vec(100, 0, 0), 0, 6, 0), 1e-9);
        for (Vec target : new Vec[]{new Vec(80, 40, 20), new Vec(-100, 30, 50), new Vec(20, -70, 30)}) {
            for (double boreYaw : new double[]{-20, 0, 35}) {
                double forward = 8, left = -5;
                double angle = turretBearingDegrees(target, forward, left, boreYaw);
                Vec muzzle = rotate(new Vec(forward, left, 0), 0, 0, angle);
                double rayAngle = Math.toRadians(angle + boreYaw);
                double dx = target.x - muzzle.x, dy = target.y - muzzle.y;
                // Zero cross product: target is on the bore line. Positive dot: in front of muzzle.
                assertEquals(0, dx * Math.sin(rayAngle) - dy * Math.cos(rayAngle), 1e-9);
                assertTrue(dx * Math.cos(rayAngle) + dy * Math.sin(rayAngle) > 0);
            }
        }
    }

    @Test public void unreachableOrBackwardMuzzleRayIsRejected() {
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(0, 0, 5), 0, 0, 0)));
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(5, 0, 0), 0, 6, 0)));
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(5, 0, 0), 0, 5, 0)));
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(5, 0, 0), 6, 0, 0)));
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(5, 0, 0), 5, 0, 0)));
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(5, 0, 0), 0, 0, Double.NaN)));
        assertTrue(Double.isNaN(turretBearingDegrees(new Vec(5, 0, 0), Double.NaN, 0, 0)));
    }

    @Test public void limitedTurretTakesValidLongRouteInsteadOfCrossingStop() {
        assertEquals(-170, nearestTurretDegrees(-170, 160, true, -175, 175), 1e-9);
        assertEquals(190, nearestTurretDegrees(-170, 160, true, -200, 200), 1e-9);
        assertEquals(370, nearestTurretDegrees(10, 350, true, 300, 400), 1e-9);
        assertEquals(170, nearestTurretDegrees(170, 700, true, -175, 175), 1e-9);
    }

    @Test public void unreachableBearingDoesNotBecomeAnEndpointTarget() {
        assertTrue(Double.isNaN(nearestTurretDegrees(150, 0, true, -90, 90)));
        assertEquals(90, nearestTurretDegrees(450, 0, true, -90, 90), 1e-9);
        assertEquals(0, nearestTurretDegrees(360, 0, true, 0, 0), 1e-9);
        assertTrue(Double.isNaN(nearestTurretDegrees(1, 0, true, 0, 0)));
    }

    @Test public void invalidAimingInputsFailClosed() {
        assertTrue(Double.isNaN(nearestTurretDegrees(Double.NaN, 0, false, 0, 0)));
        assertTrue(Double.isNaN(nearestTurretDegrees(0, Double.POSITIVE_INFINITY, false, 0, 0)));
        assertTrue(Double.isNaN(nearestTurretDegrees(0, 0, true, 90, -90)));
        assertTrue(Double.isNaN(nearestTurretDegrees(0, 0, true, Double.NaN, 90)));
        assertTrue(Double.isNaN(nearestTurretDegrees(0, 0, true, -90, Double.POSITIVE_INFINITY)));
        assertThrows(IllegalArgumentException.class, () -> new Vec(0, Double.NaN, 0));
        assertThrows(IllegalArgumentException.class, () -> rotate(new Vec(1, 0, 0), 0, Double.NaN, 0));
        assertThrows(IllegalArgumentException.class, () -> tagCenterFromTag(new Vec(0, 0, 1), 0, 0, 0, 29));
        assertThrows(IllegalArgumentException.class, () -> tagCenterFromTag(new Vec(0, 0, 1), 0, 0, 0, 46));
    }

    private static void assertVec(double x, double y, double z, Vec actual) {
        assertEquals(x, actual.x, 1e-9);
        assertEquals(y, actual.y, 1e-9);
        assertEquals(z, actual.z, 1e-9);
    }
}
