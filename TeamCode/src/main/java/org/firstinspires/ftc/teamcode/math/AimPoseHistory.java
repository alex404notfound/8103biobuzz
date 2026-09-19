package org.firstinspires.ftc.teamcode.math;

import java.util.ArrayDeque;

/**
 * Short history of robot and turret measurements on the same monotonic clock as frame capture.
 * Turret angles are unwrapped; chassis headings interpolate across their shortest angular arc.
 * A discontinuity invalidates the whole history so a stale frame cannot cross a reset or outage.
 */
public final class AimPoseHistory {
    public static final long HISTORY_NANOS = 1_000_000_000L;
    public static final long MAXIMUM_SAMPLE_GAP_NANOS = 250_000_000L;
    public static final long DEFAULT_INTERPOLATION_GAP_NANOS = 100_000_000L;
    private static final int MAXIMUM_SAMPLES = 256;

    /** All positions describe the same instant. Heading is radians; turret angle is degrees. */
    public static final class Sample {
        public final long nanos;
        public final double xInches, yInches, headingRadians, turretDegrees;

        private Sample(long nanos, double xInches, double yInches,
                       double headingRadians, double turretDegrees) {
            this.nanos = nanos;
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = headingRadians;
            this.turretDegrees = turretDegrees;
        }
    }

    private final ArrayDeque<Sample> samples = new ArrayDeque<>();
    private final long maximumInterpolationGapNanos;

    public AimPoseHistory() { this(DEFAULT_INTERPOLATION_GAP_NANOS); }

    public AimPoseHistory(long maximumInterpolationGapNanos) {
        if (maximumInterpolationGapNanos <= 0
                || maximumInterpolationGapNanos > MAXIMUM_SAMPLE_GAP_NANOS) {
            throw new IllegalArgumentException("Interpolation gap must be in (0, 250 ms]");
        }
        this.maximumInterpolationGapNanos = maximumInterpolationGapNanos;
    }

    /** A rejected measurement clears history and must be followed by a new valid sample. */
    public boolean add(long nanos, double xInches, double yInches,
                       double headingRadians, double turretDegrees) {
        if (!Double.isFinite(xInches) || !Double.isFinite(yInches)
                || !Double.isFinite(headingRadians) || !Double.isFinite(turretDegrees)) {
            clear();
            return false;
        }
        Sample last = samples.peekLast();
        if (last != null) {
            long gap = nanos - last.nanos;
            if (nanos <= last.nanos || gap <= 0 || gap > MAXIMUM_SAMPLE_GAP_NANOS) {
                clear();
                return false;
            }
        }
        samples.addLast(new Sample(nanos, xInches, yInches, headingRadians, turretDegrees));
        while (samples.size() > MAXIMUM_SAMPLES
                || nanos - samples.peekFirst().nanos > HISTORY_NANOS) {
            samples.removeFirst();
        }
        return true;
    }

    /** Returns null outside the recorded interval, or inside an interval too sparse to trust. */
    public Sample interpolate(long captureNanos) {
        Sample first = samples.peekFirst(), last = samples.peekLast();
        if (first == null || captureNanos < first.nanos || captureNanos > last.nanos) return null;
        Sample before = first;
        for (Sample after : samples) {
            if (captureNanos == after.nanos) return after;
            if (captureNanos < after.nanos) {
                long gap = after.nanos - before.nanos;
                if (gap <= 0 || gap > maximumInterpolationGapNanos) return null;
                double fraction = (double) (captureNanos - before.nanos) / gap;
                double headingDelta = Math.atan2(Math.sin(after.headingRadians - before.headingRadians),
                        Math.cos(after.headingRadians - before.headingRadians));
                // Weighted interpolation avoids overflow when finite positions have opposite signs.
                double x = interpolate(before.xInches, after.xInches, fraction);
                double y = interpolate(before.yInches, after.yInches, fraction);
                double turret = interpolate(before.turretDegrees, after.turretDegrees, fraction);
                double heading = before.headingRadians + fraction * headingDelta;
                if (!Double.isFinite(x) || !Double.isFinite(y)
                        || !Double.isFinite(turret) || !Double.isFinite(heading)) return null;
                return new Sample(captureNanos, x, y, heading, turret);
            }
            before = after;
        }
        return null;
    }

    public void clear() { samples.clear(); }

    private static double interpolate(double a, double b, double fraction) {
        return (1 - fraction) * a + fraction * b;
    }
}
