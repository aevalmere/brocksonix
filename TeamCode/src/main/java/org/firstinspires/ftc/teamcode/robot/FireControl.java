package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Decides when to fire, one ball at a time.
 *
 * A burst only starts with at least 2 balls stored, then lasts as long as the
 * button is held. Inside a burst, every ball goes through the same three steps:
 *
 * 1. WAIT: the shot must be ready (turret on target AND flywheel at speed)
 *    and stay ready for READY_HOLD_MS, so one lucky reading can't fire.
 * 2. FEED: push the next ball in until the flywheel's RPM dips, which means
 *    the ball hit the wheel. No dip within FEED_TIMEOUT_MS counts as a misfeed.
 * 3. RECOVER: stop feeding for RECOVER_MS, then back to WAIT. The flywheel has
 *    to be back at speed before the next ball goes.
 *
 * The door opens on the first ready moment and stays open for the rest of the
 * burst, so it never closes on a ball. The rail is what pauses.
 *
 * Releasing the button ends the burst right away, except during FEED: that
 * ball finishes first (shot seen or timeout), then the burst ends.
 */
@Configurable
public class FireControl {
    public enum Mode { NONE, SHOOT, PASS }

    private enum Step { WAIT, FEED, RECOVER }

    /** Passing (RT) is on hold until the variable hood is on the robot. */
    public static boolean PASS_ENABLED = false;

    // TODO(6): Raise if shots go early while the turret is still settling.
    public static double READY_HOLD_MS = 40;
    // TODO(6): Shooter Test: watch RPM as a ball goes through. Set a bit under the smallest dip you see.
    public static double SHOT_DIP_RPM = 150;
    public static double FEED_TIMEOUT_MS = 400;
    // TODO(6): Lower for faster bursts; raise if back-to-back balls still miss.
    public static double RECOVER_MS = 60;

    private Mode burst = Mode.NONE;
    private Step step = Step.WAIT;
    private boolean doorOpen = false;
    private boolean wasReady = false;
    private final ElapsedTime readyFor = new ElapsedTime();
    private final ElapsedTime inStep = new ElapsedTime();
    private int shots = 0;
    private int misfeeds = 0;

    /**
     * Call once per loop. Shoot wins if both buttons are held.
     *
     * @param ready     turret on target and flywheel at speed right now
     * @param rpm       flywheel speed now
     * @param targetRpm what the flywheel is aiming for
     */
    public void update(boolean shootHeld, boolean passHeld, boolean hasAtLeastTwo,
                       boolean ready, double rpm, double targetRpm, boolean doorFullyOpen) {
        boolean pass = passHeld && PASS_ENABLED;
        Mode wanted = shootHeld ? Mode.SHOOT : pass ? Mode.PASS : Mode.NONE;
        // Released mid-feed: let that ball finish first, so the door never closes on it.
        boolean finishingFeed = wanted == Mode.NONE && feeding();
        if (wanted == Mode.NONE && !finishingFeed) {
            burst = Mode.NONE;
            step = Step.WAIT;
            doorOpen = false;
            return;
        }
        if (burst == Mode.NONE) {
            if (!hasAtLeastTwo) return;
            shots = 0;
            misfeeds = 0;
        }
        if (!finishingFeed) burst = wanted;

        if (ready && !wasReady) readyFor.reset();
        wasReady = ready;
        boolean steadyReady = ready && readyFor.milliseconds() >= READY_HOLD_MS;

        switch (step) {
            case WAIT:
                if (steadyReady) {
                    doorOpen = true;
                    if (doorFullyOpen) enter(Step.FEED);
                }
                break;
            case FEED:
                if (rpm < targetRpm - SHOT_DIP_RPM) {
                    shots++;
                    enter(Step.RECOVER);
                } else if (inStep.milliseconds() >= FEED_TIMEOUT_MS) {
                    misfeeds++;
                    enter(Step.WAIT);
                }
                break;
            case RECOVER:
                if (inStep.milliseconds() >= RECOVER_MS) enter(Step.WAIT);
                break;
        }
    }

    private void enter(Step next) {
        step = next;
        inStep.reset();
    }

    public Mode burst() {
        return burst;
    }

    public boolean isFiring() {
        return burst != Mode.NONE;
    }

    public boolean feeding() {
        return isFiring() && step == Step.FEED;
    }

    public boolean doorOpen() {
        return doorOpen;
    }

    public String status() {
        if (!isFiring()) return "idle";
        return burst + " " + step + ", shots " + shots + (misfeeds > 0 ? ", misfeeds " + misfeeds : "");
    }
}
