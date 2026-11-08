package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathCurve;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.WeaverGenerator;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Weaver geometry behind independent clearance checks; never commands motors or schedules work.
 * All coordinates are inches in one measured frame. Keep the supplied zones fixed for this instance.
 * The circular envelope includes every robot heading, including Pedro's separate heading interpolation.
 */
public final class MarrowPlanner {
    private static final double SAMPLE_SPACING = 0.5;
    private static final int MAX_STEPS = 8192;
    private static final int MAX_TARGETS = 6;
    private final PathConfig config;
    private final Zone bounds;
    private final List<Zone> obstacles;
    private final double envelopeRadius;

    public static final class Plan {
        public final Path path;
        /** Upper bound from the Bezier control polygons, suitable for a travel budget. */
        public final double maximumLengthInches;
        private final PathResult result;

        private Plan(Path path, PathResult result, double maximumLengthInches) {
            this.path = path;
            this.result = result;
            this.maximumLengthInches = maximumLengthInches;
        }

        public List<Point> preview() { return result.getPath().sample(100); }
        /** Planned visits, not sensor-confirmed pickups. */
        public List<Point> plannedTargets() { return result.getVisitedTargets(); }
        public List<Point> skippedTargets() { return result.getUnvisitedTargets(); }
    }

    public MarrowPlanner(double robotLength, double robotWidth, double clearance,
                         Zone bounds, List<Zone> obstacles) {
        if (!Double.isFinite(robotLength) || robotLength <= 0 || !Double.isFinite(robotWidth) || robotWidth <= 0
                || !Double.isFinite(clearance) || clearance <= 0 || bounds == null || obstacles == null
                || obstacles.size() > 8 || obstacles.contains(null))
            throw new IllegalArgumentException("Marrow needs finite positive dimensions/clearance, bounds and up to 8 obstacles");
        this.bounds = bounds;
        this.obstacles = new ArrayList<>(obstacles);
        envelopeRadius = Math.hypot(robotLength, robotWidth) / 2 + clearance;
        // Use the same conservative envelope in Weaver and our independent validator.
        // Direct generator calls avoid changing Weaver's process-wide configuration.
        config = new PathConfig().robotWidth(0).robotHeight(0).clearance(envelopeRadius + SAMPLE_SPACING)
                .bruteForceOrderLimit(MAX_TARGETS);
    }

    /** Go to an exact robot-center pose. Pedro supplies headings that Weaver 1.2.1's end() mode drops. */
    public Plan planTo(Pose start, Pose end) {
        validatePose(start);
        validatePose(end);
        PathResult result = WeaverGenerator.generate(PedroPathingConverter.toPoint(start),
                PedroPathingConverter.toPoint(end), Collections.emptyList(), obstacles, false, 0, config);
        double length = validate(result);
        double heading = start.heading() + Math.atan2(Math.sin(end.heading() - start.heading()),
                Math.cos(end.heading() - start.heading()));
        Path path = PedroPathingConverter.toPath(result).linear(start.heading(), heading);
        return new Plan(path, result, length);
    }

    /**
     * Plan a bounded set of already-confirmed field targets. Intake width is a conservative effective width.
     * Movement, camera freshness, intake control and pickup confirmation belong to the calling command.
     */
    public Plan planTargets(Pose start, List<Point> targets, int maximumVisits, double intakeWidth) {
        validatePose(start);
        if (targets == null || targets.isEmpty() || targets.size() > MAX_TARGETS || maximumVisits < 1
                || maximumVisits > MAX_TARGETS || !Double.isFinite(intakeWidth) || intakeWidth < 0)
            throw new IllegalArgumentException("Supply 1..6 confirmed targets, 1..6 visits and a nonnegative intake width");
        for (Point target : targets) {
            if (target == null || !Double.isFinite(target.getX()) || !Double.isFinite(target.getY()))
                throw new IllegalArgumentException("Target coordinates must be finite");
        }
        config.intakeWidth(intakeWidth);
        PathResult result = WeaverGenerator.generate(PedroPathingConverter.toPoint(start), null,
                new ArrayList<>(targets), obstacles, true, maximumVisits, config);
        if (result.getVisitedTargets().isEmpty()) throw new IllegalArgumentException("No reachable Marrow targets");
        double length = validate(result);
        return new Plan(PedroPathingConverter.toPath(result), result, length);
    }

    public void validatePose(Pose pose) {
        if (!Constants.isFinitePose(pose)) throw new IllegalArgumentException("Non-finite robot pose");
        validatePoint(new Point(pose.x(), pose.y()), envelopeRadius);
    }

    /** Also used before camera steering, so the final approach cannot bypass the keep-out map. */
    public void validateLine(Pose start, Pose end) {
        validatePose(start);
        validatePose(end);
        validateCurve(new PathCurve(java.util.Arrays.asList(
                new Point(start.x(), start.y()), new Point(end.x(), end.y()))));
    }

    private double validate(PathResult result) {
        if (result.getSegments().isEmpty()) throw new IllegalArgumentException("Empty Marrow route");
        double length = 0;
        for (PathCurve chain : result.getSegments()) {
            for (PathCurve curve : chain.toCubicSegments()) length += validateCurve(curve);
        }
        if (!Double.isFinite(length) || length < 1e-6)
            throw new IllegalArgumentException("Marrow route has no travel");
        return length;
    }

    private double validateCurve(PathCurve curve) {
        List<Point> controls = curve.getControlPoints();
        double maximumEdge = 0, length = 0;
        for (int i = 0; i < controls.size(); i++) {
            Point p = controls.get(i);
            if (!Double.isFinite(p.getX()) || !Double.isFinite(p.getY()))
                throw new IllegalArgumentException("Non-finite Marrow control point");
            if (i > 0) {
                Point previous = controls.get(i - 1);
                double edge = Math.hypot(p.getX() - previous.getX(), p.getY() - previous.getY());
                maximumEdge = Math.max(maximumEdge, edge);
                length += edge;
            }
        }
        // A degree-n Bezier derivative is bounded by n times the longest control edge.
        // Inflate each sample by half the maximum travel between samples, covering the gaps.
        double derivativeBound = (controls.size() - 1) * maximumEdge;
        if (!Double.isFinite(derivativeBound) || derivativeBound > MAX_STEPS * SAMPLE_SPACING)
            throw new IllegalArgumentException("Marrow route exceeds validation budget");
        int steps = Math.max(1, (int) Math.ceil(derivativeBound / SAMPLE_SPACING));
        double margin = envelopeRadius + derivativeBound / (2 * steps);
        for (int i = 0; i <= steps; i++) validatePoint(curve.get((double) i / steps), margin);
        return length;
    }

    private void validatePoint(Point point, double margin) {
        double boundaryDistance = Math.abs(bounds.distanceToBoundary(point));
        if (!Double.isFinite(point.getX()) || !Double.isFinite(point.getY()) || !bounds.contains(point)
                || !Double.isFinite(boundaryDistance) || boundaryDistance <= margin)
            throw new IllegalArgumentException("Robot footprint leaves the configured field bounds");
        for (Zone obstacle : obstacles) {
            double distance = obstacle.distanceToBoundary(point);
            if (obstacle.contains(point) || !Double.isFinite(distance) || distance <= margin)
                throw new IllegalArgumentException("Marrow route intersects a keep-out or its clearance");
        }
    }
}
