package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/** One-way motor safety latch shared by all Pedro 3 AutoTune phases. */
public final class TuningSafety implements AutoCloseable {
    private static final long CALIBRATION_MIN_NANOS = 300_000_000L;
    private final BooleanSupplier sample;
    private final Runnable stopDrive;
    private final Runnable cleanup;
    private final BooleanSupplier stopRequested;
    private final BooleanSupplier stopButton;
    private final LongSupplier clock;
    private final long timeoutNanos;
    private boolean prepared;
    private boolean running;
    private boolean aborted;
    private boolean closed;
    private boolean calibrating;
    private long calibrationStarted;
    private long started;

    public TuningSafety(BooleanSupplier sample, Runnable stopDrive, Runnable cleanup,
                        BooleanSupplier stopRequested, BooleanSupplier stopButton,
                        LongSupplier clock, long timeoutNanos) {
        this.sample = sample;
        this.stopDrive = stopDrive;
        this.cleanup = cleanup;
        this.stopRequested = stopRequested;
        this.stopButton = stopButton;
        this.clock = clock;
        if (timeoutNanos <= 0) throw new IllegalArgumentException("Tuning timeout must be positive");
        this.timeoutNanos = timeoutNanos;
    }

    /** Pre-motion sensor faults may recover. Every attempt leaves the drivetrain stopped. */
    public boolean prepare() {
        checkStop();
        if (running) throw abort("A running phase cannot return to setup");
        stopDrive.run();
        prepared = readSample();
        if (calibrating && clock.getAsLong() - calibrationStarted < CALIBRATION_MIN_NANOS)
            prepared = false;
        if (prepared) calibrating = false;
        return prepared;
    }

    public void recalibrate(Runnable recalibration) {
        checkStop();
        if (running) throw abort("Recalibration is only available before arming");
        prepared = false;
        calibrating = true;
        stopDrive.run();
        calibrationStarted = clock.getAsLong();
        try {
            recalibration.run();
        } catch (RuntimeException | Error fault) {
            stopAfterFailure(fault);
            throw fault;
        }
    }

    /** Arming requires both a prepared sample and another healthy sample at the transition. */
    public void begin() {
        checkStop();
        if (!prepared || running) throw abort("Pinpoint is not prepared for tuning");
        running = true;
        started = clock.getAsLong();
        checkpoint();
    }

    /** Used by every loop and immediately before every motor write, including the first one. */
    public void checkpoint() {
        checkStop();
        if (!running) throw abort("Tuning has not been armed");
        if (clock.getAsLong() - started >= timeoutNanos)
            throw abort("Tuning phase timed out; restart the procedure to try again");
        if (!readSample()) throw abort("Pinpoint sample is not READY and finite; tuning aborted");
        if (clock.getAsLong() - started >= timeoutNanos)
            throw abort("Tuning phase timed out during the sensor read");
    }

    public void command(Runnable output) {
        checkpoint();
        try {
            output.run();
        } catch (RuntimeException | Error fault) {
            stopAfterFailure(fault);
            throw fault;
        }
    }

    private boolean readSample() {
        try {
            boolean ready = sample.getAsBoolean();
            // STOP or B can arrive while the I2C read is in progress.
            checkStop();
            return ready;
        } catch (RuntimeException | Error fault) {
            stopAfterFailure(fault);
            throw fault;
        }
    }

    private void checkStop() {
        if (aborted || closed) throw abort("Tuning was stopped; recovery will not restart motion");
        if (stopRequested.getAsBoolean() || Thread.currentThread().isInterrupted())
            throw abort("Operator stopped tuning");
        if (stopButton.getAsBoolean()) throw abort("B stopped tuning");
    }

    private Aborted abort(String message) {
        aborted = true;
        running = false;
        stopDrive.run();
        return new Aborted(message);
    }

    private void stopAfterFailure(Throwable fault) {
        aborted = true;
        running = false;
        try { stopDrive.run(); }
        catch (RuntimeException | Error stopFailure) { fault.addSuppressed(stopFailure); }
    }

    /** Motors stop before offset-restoration I2C writes and again even if cleanup throws. */
    @Override public void close() {
        aborted = true;
        running = false;
        boolean firstClose = !closed;
        closed = true;
        try {
            stopDrive.run();
        } finally {
            try {
                if (firstClose) cleanup.run();
            } finally {
                stopDrive.run();
            }
        }
    }

    public static final class Aborted extends RuntimeException {
        public Aborted(String message) { super(message); }
    }
}
