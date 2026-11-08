package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/** Immutable, distance-indexed measurements for one ball and fixed launcher geometry. */
public final class ShotTable {
    public static final int MAX_POINTS = 100;
    public static final double REPLACE_WITHIN_INCHES = 1;
    private final List<Point> points;

    public static final class Point {
        public final double distance, rpm, x, y, heading;
        public final HiveCell cell;
        public Point(Pose pose, HiveCell cell, double rpm) {
            if (!validPose(pose) || cell == null || !Double.isFinite(rpm)
                    || rpm <= 0 || rpm > ShooterFlywheel.MAX_MOTOR_RPM)
                throw new IllegalArgumentException("A sample needs an in-field pose, CELL and positive RPM <= 6000");
            this.x = pose.x(); this.y = pose.y(); this.heading = pose.heading(); this.cell = cell;
            this.distance = range(pose, cell); this.rpm = rpm;
            if (distance < 1) throw new IllegalArgumentException("Robot center must be at least 1 inch from the CELL");
        }
    }

    public static final class Lookup {
        public final boolean usable;
        public final double rpm, lowerDistance, upperDistance, fraction;
        public final String status;
        private Lookup(boolean usable, double rpm, double lower, double upper, double fraction, String status) {
            this.usable = usable; this.rpm = rpm; this.lowerDistance = lower;
            this.upperDistance = upper; this.fraction = fraction; this.status = status;
        }
        private static Lookup unavailable(String status) {
            return new Lookup(false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, status);
        }
    }

    public ShotTable() { this(Collections.emptyList()); }
    public ShotTable(List<Point> samples) {
        if (samples.size() > MAX_POINTS) throw new IllegalArgumentException("At most 100 points per shooter");
        List<Point> sorted = new ArrayList<>(samples);
        if (sorted.contains(null)) throw new IllegalArgumentException("Missing shot sample");
        sorted.sort(Comparator.comparingDouble(p -> p.distance));
        for (int i = 1; i < sorted.size(); i++) {
            if (sorted.get(i).distance - sorted.get(i - 1).distance <= REPLACE_WITHIN_INCHES)
                throw new IllegalArgumentException("Shot distances must be more than 1 inch apart");
        }
        points = Collections.unmodifiableList(sorted);
    }

    public List<Point> points() { return points; }

    /** A re-tune within one inch replaces the old sample; distinct distances remain sorted. */
    public ShotTable with(Point point) {
        List<Point> next = new ArrayList<>(points);
        next.removeIf(old -> Math.abs(old.distance - point.distance) <= REPLACE_WITHIN_INCHES);
        next.add(point);
        return new ShotTable(next);
    }

    public Lookup lookup(double distance) {
        if (!Double.isFinite(distance) || distance < 1) return Lookup.unavailable("Invalid distance");
        if (points.size() < 2) return Lookup.unavailable("Save at least two different distances for this ball");
        if (distance < points.get(0).distance) return Lookup.unavailable("Closer than calibrated range");
        if (distance > points.get(points.size() - 1).distance) return Lookup.unavailable("Farther than calibrated range");
        for (int i = 0; i < points.size(); i++) {
            Point upper = points.get(i);
            if (distance == upper.distance)
                return new Lookup(true, upper.rpm, distance, distance, 0, "Saved distance");
            if (distance < upper.distance) {
                Point lower = points.get(i - 1);
                double fraction = (distance - lower.distance) / (upper.distance - lower.distance);
                return new Lookup(true, lower.rpm + fraction * (upper.rpm - lower.rpm),
                        lower.distance, upper.distance, fraction, "Interpolated");
            }
        }
        return Lookup.unavailable("Outside calibrated range");
    }

    public static double targetX(HiveCell cell) {
        return cell.alliance == Alliance.RED ? FieldConstants.RED_HIVE_X_INCHES : FieldConstants.BLUE_HIVE_X_INCHES;
    }
    public static double targetY(HiveCell cell) {
        return cell.audienceSide ? FieldConstants.AUDIENCE_CELL_UP_Y_INCHES : FieldConstants.FAR_CELL_UP_Y_INCHES;
    }
    /** Consistently use robot-center range for calibration and lookup, independent of turret angle. */
    public static double range(Pose pose, HiveCell cell) {
        return validPose(pose) && cell != null
                ? Math.hypot(pose.x() - targetX(cell), pose.y() - targetY(cell)) : Double.NaN;
    }
    public static boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading()) && pose.x() >= 0 && pose.x() <= FieldConstants.FIELD_WIDTH_INCHES
                && pose.y() >= 0 && pose.y() <= FieldConstants.FIELD_LENGTH_INCHES;
    }
}
