package org.firstinspires.ftc.teamcode.robot.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Runs a command, checks if it worked, and if not runs it again, up to
 * maxRetries more times. After the last retry it gives up and ends anyway,
 * so the auto moves on instead of getting stuck. See RobotCommands.shootAllWithRetry.
 *
 * The check runs after each try ends, including when the try hit its own
 * timeout. So give each try a timeout and let the check decide if it worked.
 *
 * Each try is ended exactly once, even if the Retry itself is ended twice
 * (Ivy groups can do that, see CLAUDE.md).
 */
public class Retry extends CommandBuilder {
    private final Command first;
    private final Command retry;
    private final BooleanSupplier succeeded;
    private final int maxRetries;

    /** The try running now. null once finished, so end() never ends a try twice. */
    private Command running = null;
    private int retries = 0;

    /**
     * @param first      the first try
     * @param retry      every later try. It can be the same command as first.
     * @param succeeded  checked after each try. true ends the Retry, false starts another try.
     * @param maxRetries tries after the first one. 2 means up to 3 tries in all.
     */
    public Retry(Command first, Command retry, BooleanSupplier succeeded, int maxRetries) {
        this.first = first;
        this.retry = retry;
        this.succeeded = succeeded;
        this.maxRetries = maxRetries;

        // Hold everything either try needs for the whole time, so nothing else grabs it between tries.
        Set<Object> requirements = new HashSet<>(first.requirements());
        requirements.addAll(retry.requirements());
        requiring(requirements);
        setPriority(Math.max(first.priority(), retry.priority()));

        setStart(this::begin);
        setExecute(this::step);
        setDone(() -> running == null);
        setEnd(this::stop);
    }

    /** Same command for every try. */
    public Retry(Command attempt, BooleanSupplier succeeded, int maxRetries) {
        this(attempt, attempt, succeeded, maxRetries);
    }

    private void begin() {
        retries = 0;
        running = first;
        running.start();
    }

    private void step() {
        if (running == null) return;
        if (!running.done()) {
            running.execute();
            return;
        }
        // This try is over. Clear running before ending it, so stop() can't end it again.
        Command finished = running;
        running = null;
        finished.end(EndCondition.NATURALLY);

        if (!succeeded.getAsBoolean() && retries < maxRetries) {
            retries++;
            running = retry;
            running.start();
        }
    }

    /** Cancelled or timed out from outside: end the try in progress, if there is one. */
    private void stop(EndCondition endCondition) {
        if (running == null) return;
        Command interrupted = running;
        running = null;
        interrupted.end(endCondition);
    }
}
