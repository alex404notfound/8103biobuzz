package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import java.util.function.LongSupplier;

/** Tracks camera-frame age separately from the SDK's HTTP receipt age. Main thread only. */
final class VisionResultGate {
    private final int pipeline;
    private final LongSupplier clock;
    private LLResult latest;
    private double lastFrameTimestamp = Double.NaN;
    private long ageObservedNanos;
    private double knownFrameAgeMs;

    VisionResultGate(int pipeline, LongSupplier clock) {
        this.pipeline = pipeline;
        this.clock = clock;
    }

    void observe(LLResult result) {
        if (result == null || !result.isValid() || result.getPipelineIndex() != pipeline
                || !nonnegative(result.getTimestamp()) || !nonnegative(result.getStaleness())) {
            latest = null;
            return;
        }
        // A reboot may move ts backwards. Any different frame starts a new local age.
        if (Double.compare(lastFrameTimestamp, result.getTimestamp()) != 0) {
            lastFrameTimestamp = result.getTimestamp();
            ageObservedNanos = clock.getAsLong();
            knownFrameAgeMs = result.getStaleness();
        } else if (!advanceKnownAge(result.getStaleness())) {
            latest = null;
            return;
        }
        latest = result;
    }

    LLResult getFresh(double maxAgeMs) {
        if (!Double.isFinite(maxAgeMs) || maxAgeMs <= 0) return null;
        double age = getAgeMs();
        return Double.isFinite(age) && age <= maxAgeMs ? latest : null;
    }

    /** Estimated exposure age in the local monotonic clock domain, including pipeline latency. */
    double getAgeMs() {
        if (latest == null || !latest.isValid()) return Double.NaN;
        double receiptAge = latest.getStaleness();
        double capture = latest.getCaptureLatency();
        double targeting = latest.getTargetingLatency();
        double parse = latest.getParseLatency();
        if (!advanceKnownAge(receiptAge) || !nonnegative(capture)
                || !nonnegative(targeting) || !nonnegative(parse)) return Double.NaN;
        return knownFrameAgeMs + capture + targeting + parse;
    }

    /** Repeated HTTP responses must never reduce the age already known for this camera frame. */
    private boolean advanceKnownAge(double receiptAgeMs) {
        long now = clock.getAsLong();
        double elapsedMs = (now - ageObservedNanos) / 1e6;
        if (!nonnegative(elapsedMs) || !nonnegative(receiptAgeMs)) return false;
        knownFrameAgeMs = Math.max(knownFrameAgeMs + elapsedMs, receiptAgeMs);
        ageObservedNanos = now;
        return true;
    }

    void clear() { latest = null; }

    private static boolean nonnegative(double value) { return Double.isFinite(value) && value >= 0; }
}
