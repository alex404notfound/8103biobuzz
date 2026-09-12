package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class VisionResultGateTest {
    @Test public void wrongPipelineIsRejectedAndConsumerAgeIsRechecked() {
        AtomicLong now = new AtomicLong();
        VisionResultGate gate = new VisionResultGate(2, now::get);
        gate.observe(result(1, 0));
        assertNull(gate.getFresh(100));
        LLResult good = result(2, 2);
        gate.observe(good);
        assertSame(good, gate.getFresh(100));
        now.set(TimeUnit.MILLISECONDS.toNanos(101));
        assertNull(gate.getFresh(100));
    }

    @Test public void repeatedFrameExpiresDespiteNewHttpResponsesAndRebootCanRecover() {
        AtomicLong now = new AtomicLong();
        VisionResultGate gate = new VisionResultGate(0, now::get);
        gate.observe(result(1000, 0));
        now.set(TimeUnit.MILLISECONDS.toNanos(101));
        gate.observe(result(1000, 0));
        assertNull(gate.getFresh(100));
        LLResult reboot = result(1, 0);
        gate.observe(reboot);
        assertSame(reboot, gate.getFresh(100));
    }

    @Test public void includesCaptureAndPipelineLatencyAndFailsClosedForBadValues() {
        VisionResultGate gate = new VisionResultGate(0, () -> 0L);
        LLResult result = result(1, 0);
        when(result.getCaptureLatency()).thenReturn(60.0);
        when(result.getTargetingLatency()).thenReturn(50.0);
        gate.observe(result);
        assertNull(gate.getFresh(100));
        when(result.getTargetingLatency()).thenReturn(-1.0);
        assertNull(gate.getFresh(200));
        assertNull(gate.getFresh(Double.NaN));
        assertNull(gate.getFresh(-1));
    }

    @Test public void duplicateCannotRejuvenateAFrameAlreadyKnownToBeStale() {
        AtomicLong now = new AtomicLong();
        VisionResultGate gate = new VisionResultGate(0, now::get);
        LLResult stale = result(1, 0);
        when(stale.getStaleness()).thenReturn(200L);
        gate.observe(stale);
        assertNull(gate.getFresh(100));
        now.set(TimeUnit.MILLISECONDS.toNanos(1));
        gate.observe(result(1, 0));
        assertNull(gate.getFresh(100));
        LLResult next = result(2, 0);
        gate.observe(next);
        assertSame(next, gate.getFresh(100));
    }

    @Test public void consumerReceiptAgeRemainsKnownAfterADuplicateArrives() {
        AtomicLong now = new AtomicLong();
        VisionResultGate gate = new VisionResultGate(0, now::get);
        LLResult stale = result(1, 0);
        gate.observe(stale);
        when(stale.getStaleness()).thenReturn(200L);
        assertNull(gate.getFresh(100));
        now.set(TimeUnit.MILLISECONDS.toNanos(1));
        gate.observe(result(1, 0));
        assertNull(gate.getFresh(100));
    }

    static LLResult result(double timestamp, int pipeline) {
        LLResult result = mock(LLResult.class);
        when(result.isValid()).thenReturn(true);
        when(result.getTimestamp()).thenReturn(timestamp);
        when(result.getPipelineIndex()).thenReturn(pipeline);
        return result;
    }
}
