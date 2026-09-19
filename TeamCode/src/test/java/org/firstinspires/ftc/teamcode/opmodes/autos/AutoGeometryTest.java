package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.junit.Test;
import static org.junit.Assert.*;

public class AutoGeometryTest {
    @Test public void redAuthoringIsUnchangedAndBlueUsesOppositeLoadingTileAndHeading() {
        TestAuto red = new TestAuto(Alliance.RED);
        TestAuto blue = new TestAuto(Alliance.BLUE);
        assertEquals(12, red.transformed(12, 108).x(), 1e-9);
        assertEquals(108, red.transformed(12, 108).y(), 1e-9);
        assertEquals(Math.PI / 2, red.transformedHeading(90), 1e-9);
        assertEquals(132, blue.transformed(12, 108).x(), 1e-9);
        assertEquals(36, blue.transformed(12, 108).y(), 1e-9);
        assertEquals(3 * Math.PI / 2, blue.transformedHeading(90), 1e-9);
    }

    private static class TestAuto extends AutoOpMode {
        TestAuto(Alliance alliance) { super(alliance); }
        @Override protected Pose startingPose() { return Pose.zero(); }
        @Override protected Command buildSequence() { return null; }
    }
}
