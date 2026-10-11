package org.firstinspires.ftc.teamcode.robot.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.junit.Test;

/**
 * Retry, run by hand the way the Scheduler runs it (start, execute until done, end):
 * it stops at the first try that works, gives up after maxRetries, uses the retry
 * command for later tries, and ends every try exactly once, even when it is ended
 * twice from outside.
 */
public class RetryTest {
    /** A try that finishes after `loops` executes, and counts how often it was started and ended. */
    private static class Try extends CommandBuilder {
        int starts = 0;
        int ends = 0;
        int left = 0;

        Try(int loops) {
            setStart(() -> {
                starts++;
                left = loops;
            });
            setExecute(() -> left--);
            setDone(() -> left <= 0);
            setEnd(endCondition -> ends++);
        }
    }

    /** Runs the Retry until it is done (at most 100 loops), then ends it. */
    private static void run(Retry retry) {
        retry.start();
        for (int i = 0; i < 100 && !retry.done(); i++) retry.execute();
        assertTrue("the Retry should be done", retry.done());
        retry.end(EndCondition.NATURALLY);
    }

    @Test
    public void aTryThatWorksIsNotRepeated() {
        Try attempt = new Try(2);
        run(new Retry(attempt, () -> true, 3));
        assertEquals(1, attempt.starts);
        assertEquals(1, attempt.ends);
    }

    @Test
    public void itGivesUpAfterMaxRetries() {
        Try attempt = new Try(2);
        run(new Retry(attempt, () -> false, 2));
        // The first try and 2 retries, each ended once.
        assertEquals(3, attempt.starts);
        assertEquals(3, attempt.ends);
    }

    @Test
    public void itStopsAtTheFirstTryThatWorks() {
        Try attempt = new Try(1);
        // Works once the second try has ended.
        run(new Retry(attempt, () -> attempt.ends >= 2, 5));
        assertEquals(2, attempt.starts);
        assertEquals(2, attempt.ends);
    }

    @Test
    public void laterTriesUseTheRetryCommand() {
        Try first = new Try(1);
        Try again = new Try(1);
        run(new Retry(first, again, () -> false, 2));
        assertEquals(1, first.starts);
        assertEquals(2, again.starts);
        assertEquals(2, again.ends);
    }

    @Test
    public void endedFromOutsideEndsTheRunningTryOnce() {
        Try attempt = new Try(10);
        Retry retry = new Retry(attempt, () -> false, 2);
        retry.start();
        retry.execute();
        // Ivy groups can end a command twice.
        retry.end(EndCondition.INTERRUPTED);
        retry.end(EndCondition.INTERRUPTED);
        assertEquals(1, attempt.starts);
        assertEquals(1, attempt.ends);
    }
}
