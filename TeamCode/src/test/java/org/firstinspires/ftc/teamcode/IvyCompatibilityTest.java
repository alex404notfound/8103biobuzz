package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class IvyCompatibilityTest {
    @Before
    public void resetSchedulerBeforeTest() {
        Scheduler.reset();
    }

    @After
    public void resetSchedulerAfterTest() {
        Scheduler.reset();
    }

    @Test
    public void sequenceStartsEachActionOnlyAfterThePreviousActionEnds() {
        List<String> events = new ArrayList<>();
        AtomicBoolean secondActionComplete = new AtomicBoolean();

        Command first = instant(() -> events.add("first:start"))
                .setEnd(condition -> events.add("first:end:" + condition));
        Command second = Command.build()
                .setStart(() -> events.add("second:start"))
                .setExecute(() -> {
                    events.add("second:execute");
                    secondActionComplete.set(true);
                })
                .setDone(secondActionComplete::get)
                .setEnd(condition -> events.add("second:end:" + condition));
        Command sequence = sequential(first, second);

        sequence.schedule();
        Scheduler.execute();
        Scheduler.execute();
        Scheduler.execute();

        assertFalse(sequence.isScheduled());
        assertEquals(Arrays.asList(
                "first:start",
                "first:end:NATURALLY",
                "second:start",
                "second:execute",
                "second:end:NATURALLY"
        ), events);
    }

    @Test
    public void requirementConflictInterruptsTheOwnerAndCancelEndsTheReplacement() {
        List<String> events = new ArrayList<>();
        Object mechanism = new Object();

        Command owner = infinite(() -> { })
                .setStart(() -> events.add("owner:start"))
                .setEnd(condition -> events.add("owner:end:" + condition))
                .requiring(mechanism);
        Command replacement = infinite(() -> { })
                .setStart(() -> events.add("replacement:start"))
                .setEnd(condition -> events.add("replacement:end:" + condition))
                .requiring(mechanism);

        owner.schedule();
        replacement.schedule();
        replacement.cancel();

        assertFalse(owner.isScheduled());
        assertFalse(replacement.isScheduled());
        assertEquals(Arrays.asList(
                "owner:start",
                "owner:end:INTERRUPTED",
                "replacement:start",
                "replacement:end:INTERRUPTED"
        ), events);
    }

    @Test
    public void timeoutInterruptsTheLongRunningActionAndInvokesItsEndCallback() {
        AtomicReference<EndCondition> workerEnd = new AtomicReference<>();
        Command worker = infinite(() -> { }).setEnd(workerEnd::set);
        Command timedAction = race(worker, waitMs(0));

        timedAction.schedule();
        Scheduler.execute();

        assertFalse(timedAction.isScheduled());
        assertEquals(EndCondition.INTERRUPTED, workerEnd.get());
    }
}
