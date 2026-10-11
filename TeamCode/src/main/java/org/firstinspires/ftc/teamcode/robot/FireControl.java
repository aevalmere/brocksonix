package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Decides when the rail feeds balls into the shooter. Fire is continuous: once a
 * burst is going, the rail runs for as long as the button is held, and the feed
 * power (ShotTables.SHOT_FEED_POWER and PASS_FEED_POWER, by distance) sets how far
 * apart the balls go. The rail doesn't stop and wait for the wheel between balls.
 *
 * A burst has three steps:
 *
 * 1. WAIT: a burst only starts when the caller says enough balls are stored
 *    (TeleOp needs 2, auto can allow 1). The shot must be ready (Robot decides:
 *    the aim is good, and the wheel is at speed, or with the coax hood shooting,
 *    within Robot.COAX_RPM_TOLERANCE) and stay ready for READY_HOLD_MS, so one
 *    lucky reading can't start it. Then the door opens, and the rail starts once
 *    the door is fully open.
 * 2. FIRE: the rail runs. The wheel slows as balls go through, and that's
 *    expected: the ready check isn't asked again. The rail only pauses (the door
 *    stays open) while the aim is off (the `aimed` input), or while the wheel is
 *    more than BURST_RPM_FLOOR under its target. It goes again as soon as both are back.
 * 3. CLOSE: letting go of the button stops the rail at once. The door stays open
 *    DOOR_CLOSE_DELAY_MS longer, so it doesn't close on a ball that is already past
 *    it, and then the burst is over. Switching between shoot and pass mid-burst ends
 *    it the same way: the wheel needs a different speed, so the new burst starts like
 *    any other, once its own shot is ready.
 *
 * Shots are counted from the RPM: a ball hitting the wheel takes it down fast, in
 * tens of ms. A shot counts when the RPM falls SHOT_DIP_RPM below the highest reading
 * of the last DIP_WINDOW_MS, the same for both hoods, wherever the wheel was against
 * its target. A wheel coasting down to a lower target (it can't brake) takes far
 * longer than DIP_WINDOW_MS to lose that much, and a wheel still speeding up has its
 * highest reading right now, so neither counts. After a shot, the next one can only
 * count once the wheel has come back up half of SHOT_DIP_RPM from the bottom, so one
 * deep dip isn't counted twice. Two balls that hit before the wheel comes back up
 * count as one.
 *
 * The count is for telemetry and Robot.shotsFired(). It never moves the rail, except
 * with stopAfterOneShot (auto's RobotCommands.shoot()): the burst then closes by
 * itself after the first shot.
 */
@Configurable
public class FireControl {
    public enum Mode { NONE, SHOOT, PASS }

    private enum Step { WAIT, FIRE, CLOSE }

    // TODO(16): Once coax shots work, turn on and pass from around the field. Check no pass can leave the field (G405).
    /**
     * Passing (RT) is on hold until the coax hood is on the robot. With Robot.HOOD = COAX, a
     * pass is worked out from the physics (ShotSolver.solveCoaxPass); with the static hood it
     * uses the PASS_RPM table.
     */
    public static boolean PASS_ENABLED = false;

    // TODO(6): Main TeleOp: raise if bursts start while the turret is still settling.
    /** The shot must stay ready this long before a burst starts. */
    public static double READY_HOLD_MS = 40;
    // TODO(6): Main TeleOp, hold RB with 4 balls: watch the lowest RPM during the burst. Set this well past that sag, so a normal burst never pauses; lower it if the last balls of a burst fall short. 0 turns the check off.
    /** Mid-burst, the rail pauses while the wheel is more than this many RPM under its target. 0 = never. */
    public static double BURST_RPM_FLOOR = 600;
    // TODO(6): Main TeleOp: let go of RB in the middle of a burst. Raise it if the door closes on a ball; lower it if the door stays open longer than it needs to.
    /** After the rail stops, the door stays open this long, so it doesn't close on a ball already past it. */
    public static double DOOR_CLOSE_DELAY_MS = 100;
    // TODO(6): Shooter Test: watch RPM as a ball goes through. Set a bit under the smallest dip you see.
    // TODO(16): Coax Hood Test, physics mode: check it again with the wheel anywhere in the 300 RPM ready window (Robot.COAX_RPM_TOLERANCE).
    /**
     * A shot counts when the RPM falls this far below the highest reading of the last
     * DIP_WINDOW_MS. Keep it well above the reading's noise (it moves in steps of about
     * 43 RPM, so at least 2 steps, 86) plus what the wheel coasts down in DIP_WINDOW_MS.
     */
    public static double SHOT_DIP_RPM = 150;
    // TODO(6): Shooter Test: longer than a ball's dip takes to reach SHOT_DIP_RPM (watch the RPM as a ball goes through), and far shorter than the wheel takes to coast down SHOT_DIP_RPM with the power off (Cross turns the wheel off: do it at full speed and time it).
    /** How far back the shot count looks for the highest RPM, in ms. A ball's dip is quick, a coasting wheel is slow. */
    public static double DIP_WINDOW_MS = 100;
    // TODO(6): Main TeleOp, hold RB with balls loaded: a bit longer than the slowest time from the rail starting to the first shot counted.
    /** While the rail runs this long with no shot counted, telemetry says so: a jam, or SHOT_DIP_RPM is too big. */
    public static double NO_SHOT_WARN_MS = 400;

    // Room for 50 RPM readings: 250 ms even at a fast 5 ms loop, longer than any sensible
    // DIP_WINDOW_MS. With a longer window only the newest 50 count. A fixed ring, so
    // nothing is created while the robot runs.
    private static final int MAX_READINGS = 50;

    private final ElapsedTime clock = new ElapsedTime();
    private Mode burst = Mode.NONE;
    private Step step = Step.WAIT;
    private boolean doorOpen = false;
    private boolean feeding = false;
    /** Why the rail is paused mid-burst, for telemetry. */
    private String pauseReason = "";
    private boolean wasReady = false;
    private double readySinceMs = 0;
    private double closeStartMs = 0;
    private double lastUpdateMs = 0;
    /** Time the rail has run since the last shot (or since the burst started), for the "no shot seen" note. */
    private double feedingWithoutShotMs = 0;

    // The shot count: this burst's recent RPM readings, and whether a dip is still going on.
    private final double[] readingMs = new double[MAX_READINGS];
    private final double[] readingRpm = new double[MAX_READINGS];
    private int readingCount = 0;
    private int nextReading = 0;
    private boolean inDip = false;
    private double dipBottomRpm = 0;
    private int shots = 0;
    private int totalShots = 0;

    /**
     * Call once per loop.
     *
     * @param wanted           what the drivers want this loop. Robot decides it (shoot wins over pass, and pass
     *                         counts only while PASS_ENABLED), so it is decided in one place.
     * @param enoughBalls      enough balls are stored to start a burst (only checked when a burst starts)
     * @param stopAfterOneShot the burst closes by itself after the first shot is counted (auto's shoot())
     * @param ready            the shot is ready right now: the aim is good and the wheel is at speed. Only starts a burst.
     * @param aimed            the aim is good right now (turret, and coax hood if used), without the wheel.
     *                         Mid-burst, the rail pauses while it is false.
     * @param rpm              flywheel speed now
     * @param targetRpm        what the flywheel is aiming for, for BURST_RPM_FLOOR
     * @param doorFullyOpen    the door has had time to open all the way, so the rail may push balls through it
     */
    public void update(Mode wanted, boolean enoughBalls, boolean stopAfterOneShot,
                       boolean ready, boolean aimed, double rpm, double targetRpm, boolean doorFullyOpen) {
        update(wanted, enoughBalls, stopAfterOneShot, ready, aimed, rpm, targetRpm, doorFullyOpen, clock.milliseconds());
    }

    /** Same, with the time in ms passed in. Not private, so the tests can run their own clock. */
    void update(Mode wanted, boolean enoughBalls, boolean stopAfterOneShot,
                boolean ready, boolean aimed, double rpm, double targetRpm, boolean doorFullyOpen, double nowMs) {
        double loopMs = nowMs - lastUpdateMs;
        lastUpdateMs = nowMs;

        if (burst == Mode.NONE) {
            if (wanted == Mode.NONE || !enoughBalls) return;
            burst = wanted;
            step = Step.WAIT;
            shots = 0;
        }

        switch (step) {
            case WAIT:
                // Nothing has gone through the door yet, so letting go ends the burst right away,
                // and a switch between shoot and pass just changes what it waits for.
                if (wanted == Mode.NONE) {
                    endBurst();
                    return;
                }
                burst = wanted;
                if (ready && !wasReady) readySinceMs = nowMs;
                wasReady = ready;
                if (ready && nowMs - readySinceMs >= READY_HOLD_MS) {
                    doorOpen = true;
                    if (doorFullyOpen) startFiring(rpm, nowMs);
                }
                break;
            case FIRE:
                boolean shot = watchForShot(rpm, nowMs);
                if (wanted != burst || (stopAfterOneShot && shot)) {
                    startClosing(nowMs);
                    break;
                }
                boolean wheelTooSlow = BURST_RPM_FLOOR > 0 && rpm < targetRpm - BURST_RPM_FLOOR;
                feeding = aimed && !wheelTooSlow;
                pauseReason = !aimed ? "aim off" : wheelTooSlow ? "wheel under BURST_RPM_FLOOR" : "";
                if (feeding) feedingWithoutShotMs += loopMs;
                break;
            case CLOSE:
                // A ball already past the door still hits the wheel, so the count goes on.
                watchForShot(rpm, nowMs);
                if (nowMs - closeStartMs >= DOOR_CLOSE_DELAY_MS) endBurst();
                break;
        }
    }

    private void startFiring(double rpm, double nowMs) {
        step = Step.FIRE;
        // ready includes the aim and the wheel, so the rail runs from this loop on.
        feeding = true;
        pauseReason = "";
        feedingWithoutShotMs = 0;
        readingCount = 0;
        inDip = false;
        remember(rpm, nowMs);
    }

    private void startClosing(double nowMs) {
        step = Step.CLOSE;
        feeding = false;
        closeStartMs = nowMs;
    }

    private void endBurst() {
        burst = Mode.NONE;
        step = Step.WAIT;
        doorOpen = false;
        feeding = false;
        // Or the next burst would skip the READY_HOLD_MS wait if this one ended while ready.
        wasReady = false;
    }

    /** Remembers this reading and says whether it completes a shot (see the class comment). */
    private boolean watchForShot(double rpm, double nowMs) {
        boolean shot = false;
        if (inDip) {
            dipBottomRpm = Math.min(dipBottomRpm, rpm);
            // Back up half a dip from the bottom: this ball's dip is over. Start the readings
            // over, so the top of this dip doesn't count against the next ball.
            if (rpm > dipBottomRpm + SHOT_DIP_RPM / 2) {
                inDip = false;
                readingCount = 0;
            }
        } else if (rpm < highestRecentRpm(nowMs) - SHOT_DIP_RPM) {
            inDip = true;
            dipBottomRpm = rpm;
            shots++;
            totalShots++;
            feedingWithoutShotMs = 0;
            shot = true;
        }
        remember(rpm, nowMs);
        return shot;
    }

    private void remember(double rpm, double nowMs) {
        readingMs[nextReading] = nowMs;
        readingRpm[nextReading] = rpm;
        nextReading = (nextReading + 1) % MAX_READINGS;
        readingCount = Math.min(readingCount + 1, MAX_READINGS);
    }

    /** The highest reading of the last DIP_WINDOW_MS. -Infinity with none, so nothing counts. */
    private double highestRecentRpm(double nowMs) {
        double highest = Double.NEGATIVE_INFINITY;
        for (int i = 1; i <= readingCount; i++) {
            int index = (nextReading - i + MAX_READINGS) % MAX_READINGS;
            // Newest first, so the first one that is too old ends the search.
            if (nowMs - readingMs[index] > DIP_WINDOW_MS) break;
            highest = Math.max(highest, readingRpm[index]);
        }
        return highest;
    }

    /** True from the first press until the door has closed after the burst. */
    public boolean isFiring() {
        return burst != Mode.NONE;
    }

    /** The rail should push balls into the shooter now. */
    public boolean feeding() {
        return feeding;
    }

    public boolean doorOpen() {
        return doorOpen;
    }

    /** Shots counted since this object was built. Unlike the per-burst count, it never resets. */
    public int totalShots() {
        return totalShots;
    }

    public String status() {
        if (!isFiring()) return "idle";
        String doing;
        if (step == Step.WAIT) doing = "waiting for the shot";
        else if (step == Step.CLOSE) doing = "closing";
        else if (feeding) doing = "feeding";
        else doing = "paused (" + pauseReason + ")";
        String text = burst + " " + doing + ", shots " + shots;
        if (feeding && feedingWithoutShotMs >= NO_SHOT_WARN_MS) {
            text += String.format(", no shot seen for %.0f ms: jam?", feedingWithoutShotMs);
        }
        return text;
    }
}
