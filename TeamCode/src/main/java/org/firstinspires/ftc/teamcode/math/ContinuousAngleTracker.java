package org.firstinspires.ftc.teamcode.math;

/**
 * Counts shaft rotation from a single-turn absolute encoder sampled faster than half a turn.
 * A rejected sample loses the revolution count: callers must discard any homing reference.
 * This class tracks encoder-shaft degrees; gearing and direction belong to the caller.
 */
public final class ContinuousAngleTracker {
    private static final double NOISE_ALLOWANCE_DEGREES = 2;

    private final double maximumDegreesPerSecond;
    private final double maximumSampleGapSeconds;
    private boolean sampled;
    private double lastWrappedDegrees, degrees = Double.NaN, velocity = Double.NaN;
    private long lastNanos;
    private String status = "No sample";

    public ContinuousAngleTracker(double maximumDegreesPerSecond, double maximumSampleGapSeconds) {
        this.maximumDegreesPerSecond = maximumDegreesPerSecond;
        this.maximumSampleGapSeconds = maximumSampleGapSeconds;
    }

    /** Returns false when the prior revolution count can no longer be trusted. */
    public boolean update(double wrappedDegrees, long nowNanos) {
        if (!finitePositive(maximumDegreesPerSecond) || !finitePositive(maximumSampleGapSeconds)) {
            return reject("Invalid encoder speed or sample-gap configuration");
        }
        if (!Double.isFinite(wrappedDegrees) || wrappedDegrees < 0 || wrappedDegrees >= 360) {
            return reject("Encoder angle must be finite and in [0, 360)");
        }
        if (!sampled) {
            sampled = true;
            degrees = lastWrappedDegrees = wrappedDegrees;
            velocity = 0;
            lastNanos = nowNanos;
            status = "Tracking; revolution origin requires a known-position reference";
            return true;
        }
        if (nowNanos < lastNanos) return reject("Encoder sample time moved backwards");
        if (nowNanos == lastNanos) {
            if (wrappedDegrees != lastWrappedDegrees) return reject("Encoder moved without elapsed sample time");
            velocity = 0;
            return true;
        }
        long elapsedNanos = nowNanos - lastNanos;
        if (elapsedNanos <= 0) return reject("Encoder sample interval overflowed");
        double dt = elapsedNanos / 1e9;
        if (dt > maximumSampleGapSeconds) return reject("Encoder sample gap exceeded limit");
        double possibleTravel = maximumDegreesPerSecond * dt;
        double delta = wrappedDegrees - lastWrappedDegrees;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        if (Math.abs(delta) == 180) return reject("Encoder half-turn direction is ambiguous");
        if (possibleTravel + NOISE_ALLOWANCE_DEGREES >= 180) {
            return reject("Encoder sample gap could hide a half-turn (including noise)");
        }
        if (Math.abs(delta) > possibleTravel + NOISE_ALLOWANCE_DEGREES) {
            return reject("Encoder movement exceeded configured maximum speed");
        }
        double nextDegrees = degrees + delta;
        double nextVelocity = delta / dt;
        if (!Double.isFinite(nextDegrees) || !Double.isFinite(nextVelocity)) {
            return reject("Encoder position or velocity overflowed");
        }
        degrees = nextDegrees;
        velocity = nextVelocity;
        lastWrappedDegrees = wrappedDegrees;
        lastNanos = nowNanos;
        return true;
    }

    public void reset() {
        sampled = false;
        degrees = velocity = Double.NaN;
        lastWrappedDegrees = 0;
        lastNanos = 0;
        status = "No sample";
    }

    public boolean hasSample() { return sampled; }
    public double getDegrees() { return degrees; }
    public double getVelocityDegreesPerSecond() { return velocity; }
    public String getStatus() { return status; }

    private boolean reject(String reason) {
        reset();
        status = reason;
        return false;
    }

    private static boolean finitePositive(double value) { return Double.isFinite(value) && value > 0; }
}
