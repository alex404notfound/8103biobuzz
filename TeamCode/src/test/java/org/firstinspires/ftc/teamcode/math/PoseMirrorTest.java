package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.junit.Test;
import static org.junit.Assert.*;

public class PoseMirrorTest {
    @Test public void officialOppositeGardenAndLoadingTilesMapByRotation() {
        // Nominal 24-inch tile centers from FIRST's BIOBUZZ setup guide, pp. 8, 14-15.
        // These identify tiles, not exact zone centers or robot targets.
        assertPose(132, 132, Math.PI, PoseMirror.mirror(new Pose(12, 12, 0))); // A1 -> F6
        assertPose(132, 36, Math.PI, PoseMirror.mirror(new Pose(12, 108, 0))); // A5 -> F2
    }

    @Test public void fieldCenterStaysFixedAndOppositeCornersExchange() {
        assertEquals(72, FieldConstants.FIELD_CENTER_X_INCHES, 0);
        assertEquals(72, FieldConstants.FIELD_CENTER_Y_INCHES, 0);
        assertPose(72, 72, Math.PI, PoseMirror.mirror(new Pose(72, 72, 0)));
        assertPose(144, 144, Math.PI, PoseMirror.mirror(new Pose(0, 0, 0)));
    }

    @Test public void allCardinalHeadingsRotateHalfATurn() {
        assertEquals(Math.PI, PoseMirror.mirrorHeading(0), 1e-9);
        assertEquals(3 * Math.PI / 2, PoseMirror.mirrorHeading(Math.PI / 2), 1e-9);
        assertEquals(0, PoseMirror.mirrorHeading(Math.PI), 1e-9);
        assertEquals(Math.PI / 2, PoseMirror.mirrorHeading(3 * Math.PI / 2), 1e-9);
    }

    @Test public void twoAllianceTransformsRecoverPositionAndEquivalentHeading() {
        for (double heading : new double[]{-7, -Math.PI, 0, 0.37, Math.PI, 9}) {
            Pose original = new Pose(19.5, 101.25, heading);
            Pose twice = PoseMirror.mirror(PoseMirror.mirror(original));
            assertPose(original.x(), original.y(), original.heading(), twice);
        }
    }

    private static void assertPose(double x, double y, double heading, Pose actual) {
        assertEquals(x, actual.x(), 1e-9);
        assertEquals(y, actual.y(), 1e-9);
        double difference = actual.heading() - heading;
        assertEquals(0, Math.atan2(Math.sin(difference), Math.cos(difference)), 1e-9);
    }
}
