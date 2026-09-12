package org.firstinspires.ftc.teamcode.pedroPathing;

import org.junit.Test;

import static org.junit.Assert.*;

/** Exercises the latch used by every AutoTune phase without an Android OpMode thread. */
public class TuningSafetyTest {
    private boolean ready = true;
    private boolean bHeld;
    private boolean stopRequested;
    private double power;
    private long now;
    private int samples;
    private int cleanupCount;
    private double powerAtCleanup = -1;
    private Runnable duringSample = () -> { };
    private final TuningSafety safety = new TuningSafety(
            () -> { samples++; duringSample.run(); return ready; },
            () -> power = 0,
            () -> {
                cleanupCount++;
                powerAtCleanup = power;
                // A cleanup implementation must not be able to leave motors powered.
                power = 0.5;
            },
            () -> stopRequested, () -> bHeld, () -> now, 15_000_000_000L);

    @Test public void startWithoutPreparedReadySampleNeverCommandsMotion() {
        expectAbort(() -> safety.begin());
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void initialCalibrationCanRecoverWhileMotorsStayStopped() {
        ready = false;
        power = 1;
        assertFalse(safety.prepare());
        assertEquals(0, power, 0);
        ready = true;
        start();
        safety.command(() -> power = 1);
        assertEquals(1, power, 0);
    }

    @Test public void readyAtPreparationDoesNotAllowBadSampleAtStart() {
        assertTrue(safety.prepare());
        ready = false;
        expectAbort(() -> safety.begin());
        assertEquals(0, power, 0);
    }

    @Test public void everyCommandRequiresAnotherSample() {
        start();
        int earlierSamples = samples;
        safety.command(() -> power = 1);
        assertEquals(earlierSamples + 1, samples);
        ready = false;
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void faultStopsPowerAndRecoveryNeverResumes() {
        start();
        safety.command(() -> power = 1);
        ready = false;
        expectAbort(() -> safety.checkpoint());
        assertEquals(0, power, 0);
        ready = true;
        expectAbort(() -> safety.prepare());
        expectAbort(() -> safety.begin());
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void heldBStopsWithoutAnEdgeAndRemainsLatchedAfterRelease() {
        start();
        safety.command(() -> power = 1);
        bHeld = true;
        expectAbort(() -> safety.checkpoint());
        bHeld = false;
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void stopArrivingDuringSensorReadPreventsOutput() {
        start();
        duringSample = () -> stopRequested = true;
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void calibrationWaitsThreeHundredMillisecondsAndANewReadySample() {
        assertTrue(safety.prepare());
        safety.recalibrate(() -> assertEquals(0, power, 0));
        assertFalse(safety.prepare());
        now = 299_999_999L;
        assertFalse(safety.prepare());
        now = 300_000_000L;
        ready = false;
        assertFalse(safety.prepare());
        ready = true;
        assertTrue(safety.prepare());
        safety.begin();
        safety.command(() -> power = 1);
        assertEquals(1, power, 0);
    }

    @Test public void timeoutAbortsAndCannotRestart() {
        start();
        safety.command(() -> power = 1);
        now = 15_000_000_000L;
        expectAbort(() -> safety.checkpoint());
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void timeoutDuringSensorReadPreventsTheNextMotorWrite() {
        start();
        now = 14_999_999_999L;
        duringSample = () -> now = 15_000_000_000L;
        expectAbort(() -> safety.command(() -> power = 1));
        assertEquals(0, power, 0);
    }

    @Test public void motorWriteFailureStopsBeforePropagating() {
        start();
        try {
            safety.command(() -> { power = 1; throw new IllegalStateException("hardware write"); });
            fail("Expected hardware failure");
        } catch (IllegalStateException expected) {
            assertEquals("hardware write", expected.getMessage());
        }
        assertEquals(0, power, 0);
        expectAbort(() -> safety.command(() -> power = 1));
    }

    @Test public void sensorExceptionAlsoStopsBeforePropagating() {
        start();
        safety.command(() -> power = 1);
        duringSample = () -> { throw new IllegalArgumentException("bad sensor response"); };
        try {
            safety.checkpoint();
            fail("Expected sensor failure");
        } catch (IllegalArgumentException expected) {
            assertEquals("bad sensor response", expected.getMessage());
        }
        assertEquals(0, power, 0);
    }

    @Test public void cleanupRunsOnceWithPowerZeroAndStopsAgainAfterward() {
        start();
        safety.command(() -> power = 1);
        safety.close();
        assertEquals(0, powerAtCleanup, 0);
        assertEquals(0, power, 0);
        safety.close();
        assertEquals(1, cleanupCount);
        expectAbort(() -> safety.command(() -> power = 1));
    }

    @Test public void cleanupStillRunsAfterAnAbortedSample() {
        start();
        safety.command(() -> power = 1);
        ready = false;
        try {
            expectAbort(() -> safety.checkpoint());
        } finally {
            safety.close();
        }
        assertEquals(1, cleanupCount);
        assertEquals(0, powerAtCleanup, 0);
        assertEquals(0, power, 0);
    }

    private void start() { assertTrue(safety.prepare()); safety.begin(); }

    private static void expectAbort(Runnable action) {
        try { action.run(); fail("Expected latched tuning abort"); }
        catch (TuningSafety.Aborted expected) { }
    }
}
