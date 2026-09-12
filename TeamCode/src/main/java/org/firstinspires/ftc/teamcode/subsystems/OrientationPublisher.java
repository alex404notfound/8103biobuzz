package org.firstinspires.ftc.teamcode.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.DoublePredicate;
import java.util.function.LongSupplier;

/** Keeps the SDK's blocking camera writes off the OpMode thread, with one pending heading. */
final class OrientationPublisher implements AutoCloseable {
    private static final long WRITE_INTERVAL_MS = 50;
    private static final long MAX_HEADING_AGE_NANOS = TimeUnit.MILLISECONDS.toNanos(250);

    private final BooleanSupplier configure;
    private final DoublePredicate sender;
    private final ScheduledExecutorService worker;
    private final LongSupplier clock;
    private volatile Heading latest;
    private volatile boolean closed;
    private volatile boolean ready;
    private volatile boolean lastWriteSucceeded;
    private volatile Heading lastSuccessfulHeading;

    OrientationPublisher(BooleanSupplier configure, DoublePredicate sender) {
        this(configure, sender, Executors.newSingleThreadScheduledExecutor(task -> {
            Thread thread = new Thread(task, "Limelight-orientation");
            thread.setDaemon(true);
            return thread;
        }), System::nanoTime);
    }

    OrientationPublisher(BooleanSupplier configure, DoublePredicate sender,
                         ScheduledExecutorService worker, LongSupplier clock) {
        this.configure = configure;
        this.sender = sender;
        this.worker = worker;
        this.clock = clock;
        worker.scheduleWithFixedDelay(this::sendLatest, 0, WRITE_INTERVAL_MS, TimeUnit.MILLISECONDS);
    }

    void publish(double degrees) {
        if (!Double.isFinite(degrees)) throw new IllegalArgumentException("Heading must be finite");
        if (!closed) latest = new Heading(degrees, clock.getAsLong());
    }

    boolean isReady() { return ready && !closed; }
    boolean wasLastWriteSuccessful() { return lastWriteSucceeded; }

    boolean hasRecentHeading(double maxAgeMs) {
        Heading sent = lastSuccessfulHeading;
        if (closed || !lastWriteSucceeded || sent == null || !Double.isFinite(maxAgeMs) || maxAgeMs <= 0) return false;
        double age = (clock.getAsLong() - sent.timestamp) / 1e6;
        return age >= 0 && age <= maxAgeMs;
    }

    private void sendLatest() {
        if (closed) return;
        try {
            if (!ready) ready = configure.getAsBoolean();
            Heading heading = latest;
            if (closed || !ready || heading == null
                    || clock.getAsLong() - heading.timestamp > MAX_HEADING_AGE_NANOS) return;
            lastWriteSucceeded = sender.test(heading.degrees);
            if (lastWriteSucceeded) lastSuccessfulHeading = heading;
        } catch (RuntimeException cameraFailure) {
            // A transient SDK/network failure must not suppress all future scheduled writes.
            lastWriteSucceeded = false;
        }
    }

    @Override public void close() {
        closed = true;
        latest = null;
        worker.shutdownNow();
        // The SDK may ignore interruption during HTTP I/O. Never join it from stop().
    }

    private static final class Heading {
        final double degrees;
        final long timestamp;
        Heading(double degrees, long timestamp) { this.degrees = degrees; this.timestamp = timestamp; }
    }
}
