package org.firstinspires.ftc.teamcode.math;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

/** Fits encoder position against elapsed time without assuming a motor type or rotation scale. */
public final class EncoderRpmEstimator {
    private static final double NS_PER_SECOND = 1_000_000_000.0;

    private final long windowNs;
    private final long maximumGapNs;
    private final Deque<Sample> samples = new ArrayDeque<>();
    private int previousRawPosition;
    private long unwrappedPosition;
    private double countsPerSecond = Double.NaN;
    private double residualCounts = Double.NaN;

    public EncoderRpmEstimator(double windowSeconds) {
        this(windowSeconds, 0.5);
    }

    public EncoderRpmEstimator(double windowSeconds, double maximumGapSeconds) {
        windowNs = secondsToNs(windowSeconds);
        maximumGapNs = secondsToNs(maximumGapSeconds);
    }

    /** Clears the fit, including the raw encoder reference. */
    public void reset() {
        samples.clear();
        previousRawPosition = 0;
        unwrappedPosition = 0;
        countsPerSecond = residualCounts = Double.NaN;
    }

    /**
     * Adds a fresh encoder reading. A non-increasing timestamp or a long sampling gap starts
     * a new history, so a stale estimate cannot survive a clock or communication interruption.
     */
    public void addSample(long timestampNs, int rawPosition) {
        if (!samples.isEmpty()) {
            long previousTime = samples.getLast().timestampNs;
            long gapNs = timestampNs - previousTime;
            if (timestampNs <= previousTime || gapNs <= 0 || gapNs > maximumGapNs) reset();
        }
        if (samples.isEmpty()) {
            previousRawPosition = rawPosition;
        } else {
            // Int subtraction deliberately unwraps the SDK's signed 32-bit encoder counter.
            unwrappedPosition += rawPosition - previousRawPosition;
            previousRawPosition = rawPosition;
        }
        samples.addLast(new Sample(timestampNs, unwrappedPosition));

        // Keep one boundary sample at or before the window's beginning. Discarding every older
        // sample would leave an irregularly sampled history perpetually shorter than the window.
        while (samples.size() > 2) {
            Iterator<Sample> iterator = samples.iterator();
            iterator.next();
            Sample second = iterator.next();
            if (timestampNs - second.timestampNs < windowNs) break;
            samples.removeFirst();
        }
        fit();
    }

    public boolean isReady() { return Double.isFinite(countsPerSecond); }

    /** Signed encoder counts per second; NaN until a full window of valid samples is available. */
    public double getCountsPerSecond() { return countsPerSecond; }

    public int getSampleCount() { return samples.size(); }

    public double getSpanSeconds() {
        if (samples.size() < 2) return 0;
        return (samples.getLast().timestampNs - samples.getFirst().timestampNs) / NS_PER_SECOND;
    }

    /** RMS position residual around the fitted line, in counts. This is not absolute RPM accuracy. */
    public double getResidualCounts() { return residualCounts; }

    private void fit() {
        countsPerSecond = residualCounts = Double.NaN;
        if (samples.size() < 3) return;
        Sample origin = samples.getFirst();
        if (samples.getLast().timestampNs - origin.timestampNs < windowNs) return;

        double sumTime = 0;
        double sumPosition = 0;
        for (Sample sample : samples) {
            sumTime += (sample.timestampNs - origin.timestampNs) / NS_PER_SECOND;
            sumPosition += sample.position - origin.position;
        }
        double meanTime = sumTime / samples.size();
        double meanPosition = sumPosition / samples.size();
        double sumTimeSquared = 0;
        double sumTimePosition = 0;
        for (Sample sample : samples) {
            double centeredTime = (sample.timestampNs - origin.timestampNs) / NS_PER_SECOND - meanTime;
            double centeredPosition = sample.position - origin.position - meanPosition;
            sumTimeSquared += centeredTime * centeredTime;
            sumTimePosition += centeredTime * centeredPosition;
        }
        if (sumTimeSquared <= 0) return;
        countsPerSecond = sumTimePosition / sumTimeSquared;

        double squaredResidual = 0;
        for (Sample sample : samples) {
            double centeredTime = (sample.timestampNs - origin.timestampNs) / NS_PER_SECOND - meanTime;
            double error = sample.position - origin.position - meanPosition - countsPerSecond * centeredTime;
            squaredResidual += error * error;
        }
        residualCounts = Math.sqrt(squaredResidual / samples.size());
    }

    private static long secondsToNs(double seconds) {
        double nanoseconds = seconds * NS_PER_SECOND;
        if (!Double.isFinite(nanoseconds) || nanoseconds < 1 || nanoseconds >= Long.MAX_VALUE) {
            throw new IllegalArgumentException("Sampling durations must be positive finite seconds.");
        }
        return Math.round(nanoseconds);
    }

    private static final class Sample {
        final long timestampNs;
        final long position;

        Sample(long timestampNs, long position) {
            this.timestampNs = timestampNs;
            this.position = position;
        }
    }
}
