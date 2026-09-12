package org.firstinspires.ftc.teamcode.subsystems;

import org.junit.Test;
import org.mockito.ArgumentCaptor;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.DoublePredicate;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class OrientationPublisherTest {
    @Test public void successfulButDelayedWriteDoesNotMakeAnOldHeadingFreshAgain() {
        ScheduledExecutorService executor = mock(ScheduledExecutorService.class);
        AtomicLong now = new AtomicLong();
        OrientationPublisher publisher = new OrientationPublisher(() -> true, heading -> {
            now.set(TimeUnit.SECONDS.toNanos(1));
            return true;
        }, executor, now::get);
        ArgumentCaptor<Runnable> tick = ArgumentCaptor.forClass(Runnable.class);
        verify(executor).scheduleWithFixedDelay(tick.capture(), anyLong(), anyLong(), eq(TimeUnit.MILLISECONDS));
        publisher.publish(1);
        tick.getValue().run();
        assertFalse(publisher.hasRecentHeading(100));
        publisher.publish(2);
        tick.getValue().run();
        assertTrue(publisher.hasRecentHeading(100));
        publisher.close();
        assertFalse(publisher.hasRecentHeading(100));
    }

    @Test(timeout = 5000) public void blockedCameraDoesNotBlockPublicationAndPendingHeadingsCoalesce() throws Exception {
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        BlockingQueue<Double> sent = new LinkedBlockingQueue<>();
        AtomicInteger writes = new AtomicInteger();
        OrientationPublisher publisher = new OrientationPublisher(() -> true, heading -> {
            sent.add(heading);
            if (writes.getAndIncrement() == 0) {
                entered.countDown();
                try { release.await(); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            }
            return true;
        }, executor, System::nanoTime);
        try {
            publisher.publish(1);
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            assertEquals(Double.valueOf(1), sent.poll());
            for (int heading = 2; heading <= 1000; heading++) publisher.publish(heading);
            assertEquals(1, writes.get());
            release.countDown();
            assertEquals(Double.valueOf(1000), sent.poll(2, TimeUnit.SECONDS));
        } finally {
            release.countDown();
            publisher.close();
            assertTrue(executor.awaitTermination(2, TimeUnit.SECONDS));
        }
    }

    @Test(timeout = 5000) public void closeReturnsWithoutWaitingForAnUninterruptibleRequest() throws Exception {
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicInteger writes = new AtomicInteger();
        OrientationPublisher publisher = new OrientationPublisher(() -> true, heading -> {
            writes.incrementAndGet();
            entered.countDown();
            boolean released = false;
            while (!released) {
                try { release.await(); released = true; } catch (InterruptedException ignored) { }
            }
            return true;
        }, executor, System::nanoTime);
        try {
            publisher.publish(1);
            assertTrue(entered.await(2, TimeUnit.SECONDS));
            publisher.publish(2);
            publisher.close();
            assertEquals(1, release.getCount());
            publisher.publish(3);
        } finally {
            release.countDown();
            publisher.close();
            assertTrue(executor.awaitTermination(2, TimeUnit.SECONDS));
        }
        assertEquals(1, writes.get());
    }

    @Test public void failedSetupRetriesAndExpiredOrNonfiniteHeadingsAreNotSent() {
        ScheduledExecutorService executor = mock(ScheduledExecutorService.class);
        AtomicLong now = new AtomicLong();
        AtomicInteger setups = new AtomicInteger();
        DoublePredicate sender = mock(DoublePredicate.class);
        OrientationPublisher publisher = new OrientationPublisher(
                () -> setups.incrementAndGet() > 1, sender, executor, now::get);
        ArgumentCaptor<Runnable> tick = ArgumentCaptor.forClass(Runnable.class);
        verify(executor).scheduleWithFixedDelay(tick.capture(), anyLong(), anyLong(), eq(TimeUnit.MILLISECONDS));
        publisher.publish(30);
        tick.getValue().run();
        verifyNoInteractions(sender);
        now.set(TimeUnit.SECONDS.toNanos(1));
        tick.getValue().run();
        verifyNoInteractions(sender);

        assertThrows(IllegalArgumentException.class, () -> publisher.publish(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> publisher.publish(Double.POSITIVE_INFINITY));
        publisher.publish(45);
        tick.getValue().run();
        verify(sender).test(45);
        publisher.close();
        tick.getValue().run();
        verifyNoMoreInteractions(sender);
    }

    @Test public void senderExceptionDoesNotPreventSubsequentWrites() {
        ScheduledExecutorService executor = mock(ScheduledExecutorService.class);
        DoublePredicate sender = mock(DoublePredicate.class);
        when(sender.test(1)).thenThrow(new IllegalStateException("camera unavailable"));
        OrientationPublisher publisher = new OrientationPublisher(() -> true, sender, executor, () -> 0L);
        ArgumentCaptor<Runnable> tick = ArgumentCaptor.forClass(Runnable.class);
        verify(executor).scheduleWithFixedDelay(tick.capture(), anyLong(), anyLong(), eq(TimeUnit.MILLISECONDS));
        publisher.publish(1);
        tick.getValue().run();
        publisher.publish(2);
        tick.getValue().run();
        verify(sender).test(2);
        publisher.close();
    }
}
