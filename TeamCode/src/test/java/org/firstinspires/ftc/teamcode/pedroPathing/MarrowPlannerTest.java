package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

/** Uses the actual published Weaver and Pedro converter, including their failure cases. */
public class MarrowPlannerTest {
    private static final Zone FIELD = new PolygonZone(new Point(72, 72), 144, 144);

    private MarrowPlanner planner(Zone... obstacles) {
        return new MarrowPlanner(4, 4, 1, FIELD, Arrays.asList(obstacles));
    }

    @Test public void destinationKeepsTheRequestedPedroHeadingEvenThoughWeaverDropsIt() {
        MarrowPlanner.Plan plan = planner().planTo(new Pose(20, 72, 0), new Pose(100, 72, Math.PI / 2));
        assertEquals(20, plan.path.get(0).x(), 1e-8);
        assertEquals(100, plan.path.endPose().x(), 1e-8);
        assertEquals(72, plan.path.endPose().y(), 1e-8);
        assertEquals(Math.PI / 2, plan.path.endPose().heading(), 1e-8);
        assertTrue(plan.maximumLengthInches >= 80);
    }

    @Test public void headingCrossesZeroAlongTheShortArc() {
        MarrowPlanner.Plan plan = planner().planTo(new Pose(20, 72, Math.toRadians(350)),
                new Pose(100, 72, Math.toRadians(10)));
        double delta = Math.atan2(Math.sin(plan.path.get(.5).heading()), Math.cos(plan.path.get(.5).heading()));
        assertEquals(0, delta, 1e-8);
    }

    @Test public void circleDetourClearsTheWholeRobotAndPreservesTheGoal() {
        MarrowPlanner.Plan plan = planner(new CircleZone(new Point(70, 72), 12))
                .planTo(new Pose(20, 72, 0), new Pose(120, 72, 0));
        assertTrue(plan.maximumLengthInches > 100);
        assertEquals(120, plan.path.endPose().x(), 1e-8);
        for (Point sample : plan.preview())
            assertTrue(Math.hypot(sample.getX() - 70, sample.getY() - 72) > 12 + Math.hypot(4, 4) / 2 + 1);
    }

    @Test public void unreachableDestinationRejectsWeaversCollidingFallback() {
        MarrowPlanner planner = planner(
                new PolygonZone(new Point(60, 72), 10, 50), new PolygonZone(new Point(100, 72), 10, 50),
                new PolygonZone(new Point(80, 52), 50, 10), new PolygonZone(new Point(80, 92), 50, 10));
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> planner.planTo(new Pose(20, 72, 0), new Pose(80, 72, 0)));
        assertTrue(failure.getMessage().contains("keep-out"));
    }

    @Test public void pathAroundAnObstacleCannotEscapeTheAllowedArea() {
        MarrowPlanner planner = new MarrowPlanner(4, 4, 1,
                new PolygonZone(new Point(72, 72), 144, 30),
                Collections.singletonList(new CircleZone(new Point(70, 72), 12)));
        assertThrows(IllegalArgumentException.class,
                () -> planner.planTo(new Pose(20, 72, 0), new Pose(120, 72, 0)));
    }

    @Test public void fullFootprintAndAllHeadingsCountAtTheWall() {
        MarrowPlanner planner = new MarrowPlanner(18, 10, 1, FIELD, Collections.emptyList());
        assertThrows(IllegalArgumentException.class, () -> planner.validatePose(new Pose(8, 72, Math.PI / 2)));
        planner.validatePose(new Pose(20, 72, Math.PI / 2));
    }

    @Test public void thinKeepOutBetweenSamplesCannotDisappear() {
        MarrowPlanner planner = new MarrowPlanner(.01, .01, .001, FIELD,
                Collections.singletonList(new CircleZone(new Point(50.25, 72), .01)));
        assertThrows(IllegalArgumentException.class,
                () -> planner.validateLine(new Pose(20, 72, 0), new Pose(100, 72, 0)));
    }

    @Test public void cameraApproachLineUsesTheSameKeepOutAndBoundaryChecks() {
        assertThrows(IllegalArgumentException.class, () -> planner(new CircleZone(new Point(70, 72), 2))
                .validateLine(new Pose(50, 72, 0), new Pose(90, 72, 0)));
        assertThrows(IllegalArgumentException.class, () -> planner()
                .validateLine(new Pose(20, 72, 0), new Pose(1, 72, 0)));
    }

    @Test public void multiTargetPlansReportSelectionWithoutClaimingPickups() {
        List<Point> targets = Arrays.asList(new Point(40, 72), new Point(60, 72), new Point(100, 72));
        MarrowPlanner.Plan plan = planner().planTargets(new Pose(20, 72, 0), targets, 2, 0);
        assertEquals(2, plan.plannedTargets().size());
        assertEquals(1, plan.skippedTargets().size());
        assertEquals(60, plan.path.endPose().x(), 1e-8);
    }

    @Test public void allBlockedTargetsAndNonFiniteInputsCannotBecomeADrivePath() {
        MarrowPlanner planner = planner(new CircleZone(new Point(100, 72), 8));
        assertThrows(IllegalArgumentException.class, () -> planner.planTargets(new Pose(20, 72, 0),
                Collections.singletonList(new Point(100, 72)), 1, 0));
        assertThrows(IllegalArgumentException.class,
                () -> planner.planTo(new Pose(Double.NaN, 72, 0), new Pose(100, 72, 0)));
        assertThrows(IllegalArgumentException.class, () -> planner.planTargets(new Pose(20, 72, 0),
                Collections.singletonList(new Point(Double.POSITIVE_INFINITY, 72)), 1, 0));
        assertThrows(IllegalArgumentException.class, () -> new MarrowPlanner(0, 4, 1, FIELD, Collections.emptyList()));
    }
}
