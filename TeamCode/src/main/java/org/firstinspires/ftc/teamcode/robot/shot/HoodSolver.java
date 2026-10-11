package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.LookupTable;

/**
 * Picks the coax hood's launch angle from projectile physics, so a shot still
 * scores when the flywheel isn't at its target speed. Ported from team GNCE
 * Ditto's ShotPlanner (FTC DECODE 2025-26). Here everything is in inches and
 * seconds, and launch angles are degrees above horizontal (0 = flat, 90 =
 * straight up).
 *
 * Why the measured RPM: the flywheel's target RPM still comes from a distance
 * table (ShotTables.COAX_RPM), but the hood angle is solved from the RPM the
 * wheel really has right now. Right after a shot the wheel dips, and while it
 * spins up it is slow. The hood moves in milliseconds and the flywheel takes
 * much longer, so aiming the hood for the speed we really have still scores.
 *
 * The physics: a ball that leaves at speed v and angle a is at height
 *   LAUNCH_HEIGHT + x tan(a) - g x² / (2 v² cos²(a))
 * after x inches along the floor. Setting that equal to the height we want and
 * solving for a gives two answers, a low arc (flat and fast) and a high arc (a
 * lob), or none when the ball is too slow to get there at all.
 *
 * The exit speed depends on the angle too: on a coax hood the ball wraps the
 * wheel for a different distance at each angle (ShotTables.COAX_EXIT_SPEED_ANGLE).
 * So we solve with the speed at a guess angle, then solve again with the speed
 * at that answer, a few times until the angle stops moving, and check the final
 * angle with its own speed.
 *
 * The target is a band, not a point: the ball center may cross anywhere in
 * TARGET_HEIGHT ± TARGET_BAND. We try heights from the center outward (center,
 * a bit above, a bit below, further out, ...) and keep the first one that
 * works. The center goes first because it leaves the most room for error.
 *
 * The hood only moves between minDeg and maxDeg, so each answer is clamped
 * into that range. A clamped angle no longer crosses at the height we aimed
 * for, so we check where it really crosses: still inside the band, it's kept.
 *
 * The lip: the CELL's opening leans back, so its bottom edge sticks out
 * LIP_BEFORE inches toward the robot. The ball must pass above that edge there
 * too, or it hits the front of the CELL.
 *
 * The margin: the measured RPM lags a little and is noisy, so an angle is only
 * kept if it also scores with the wheel RPM_MARGIN slower or faster. Near the
 * slowest RPM that scores, the right angle changes fast with the RPM, and this
 * keeps the hood from betting on the exact reading.
 *
 * Up close, all the sampled heights can miss even though some angles score (the
 * band sits just above the lip). Then we walk the hood's whole range instead.
 *
 * Flight time (for shooting on the move) comes from the same physics:
 * distance / horizontal speed. Passes use the same physics too, aimed at a
 * spot on the floor instead of the CELL (passRpm, passLaunchDeg).
 *
 * Plain math, no hardware. A NaN in (or a typo in Panels) gives NaN out, and
 * callers must never send a NaN to the servo.
 *
 * From GNCE Ditto (github.com/GNCE/DECODEv2-Ditto, their final commit):
 *   - solving the hood from the measured, averaged RPM; the band sampled from the
 *     center out; low and high arcs clamped to the hood, then checked; the lip;
 *     flight time from the physics: ShotPlanner's solveHoodDeg, tryArc,
 *     sampledHeights, estimateFlightTimeSec and plan()
 *     https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/core/util/ShotPlanner.java
 *   - the seed table (seedRpm, but they picked the flattest shot; see below):
 *     https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/LutTuningSolver.java
 * Ours, not theirs: inches and launch angles above horizontal, low arcs before any
 * high arc and no lob on a wheel that's too fast (no 30-degree hood swings), exit
 * speed by RPM and by hood angle instead of a near/far step, the RPM margin, the
 * walk up close, seeds picked for room to correct the RPM, and passes.
 */
@Configurable
public final class HoodSolver {
    private HoodSolver() {}

    // TODO(16): Measure the ball center's height above the tiles at the moment it leaves the wheel.
    /** Ball center height above the tiles where it leaves the shooter, inches. */
    public static double LAUNCH_HEIGHT = 11;
    /** Center of the CELL opening above the tiles: halfway between its edges at 53.5 and 65.6 in. */
    public static double TARGET_HEIGHT = 59.55;
    // TODO(16): Smaller = pickier, more accurate shots; larger = shoots in more situations. Keep it above about 2 while RPM_MARGIN is 50 (the margin moves the crossing 1 to 2 in.), or lower both together.
    /** The ball center may cross this many inches above or below TARGET_HEIGHT. */
    public static double TARGET_BAND = 3;
    /** Heights tried across the band, center first, then outward (1 = center only). */
    public static int TARGET_SAMPLES = 7;
    // TODO(16): Only if shots clip the CELL's bottom edge: raise it a little, and check LIP_BEFORE against the real CELL.
    /** The opening's bottom edge (53.5) + the ball's radius (1.4): the ball center must pass above this... */
    public static double LIP_HEIGHT = 54.9;
    /**
     * ...this many inches before the target point. The opening leans back 30° and
     * is 14 in. tall, so its bottom edge sticks out 3.5 in. toward the robot.
     */
    public static double LIP_BEFORE = 3.5;
    // TODO(16): Raise it if shots fired while the wheel recovers miss; lower it if the hood refuses shots it would make.
    /**
     * A shot angle must also score with the wheel this many RPM slower or faster,
     * because the measured RPM lags and is noisy. It uses up some of TARGET_BAND:
     * 50 RPM moves the crossing about 1 to 2 in.
     */
    public static double RPM_MARGIN = 50;
    // TODO(16): Keep it equal to Robot.COAX_RPM_TOLERANCE (both 300 now): the seeds leave the hood room for exactly the RPM error a burst may start with.
    /**
     * Seeds (seedRpm) pick the slowest RPM at which the hood can still score with
     * the wheel this many RPM off either way. The coax hood is there to make up
     * for RPM error, and a slower wheel recovers faster between balls.
     */
    public static double SEED_RPM_WINDOW = 300;
    /** Ball center height when a pass lands on the tiles: the ball's radius. */
    public static double PASS_LANDING_HEIGHT = 1.4;
    /** A pass counts if it lands within this many inches (along the floor) of where it was aimed. */
    public static double PASS_LANDING_TOLERANCE = 6;
    /** in/s². */
    public static final double GRAVITY = 386.09;

    /** A speed (in/s) or cosine this small counts as zero. */
    private static final double TINY = 1e-6;
    /** Inches of rounding error the band check allows, so a TARGET_BAND of 0 can still pass. */
    private static final double BAND_SLACK = 1e-6;
    /** Step of the last-try walk through the hood's range, degrees. */
    private static final double ANGLE_STEP_DEG = 0.1;
    /** Most steps in that walk (180° worth), so a strange range from Panels can't make it long. */
    private static final int MOST_ANGLE_STEPS = 1800;
    /** Most heights tried, so a typo in Panels can't slow down the loop. */
    private static final int MOST_SAMPLES = 99;
    /**
     * Most rounds of "solve, then re-solve with the speed at the answer". With the
     * same multiplier at every angle it settles after 2. A multiplier that changes
     * a lot with the angle can make it bounce near the slowest RPM that scores;
     * the final check still keeps an angle that doesn't score from getting out.
     */
    private static final int ROUNDS = 8;
    /** An angle that moves less than this in a round has settled. */
    private static final double SETTLED_DEG = 0.01;
    /** RPM steps seedRpm checks, from SCAN_START_RPM up. */
    private static final double SCAN_STEP_RPM = 25;
    private static final double SCAN_START_RPM = 500;
    /** Far past any FTC flywheel: caps the seed scan and passRpm's search. */
    private static final double MOST_RPM = 20000;

    /** Ball exit speed in in/s at this flywheel RPM (uses |rpm|), from ShotTables.COAX_EXIT_SPEED, before the angle's multiplier. */
    public static double exitSpeed(double rpm) {
        double wheel = Math.abs(rpm);
        return wheel * LookupTable.at(ShotTables.COAX_EXIT_SPEED, wheel);
    }

    /** Ball exit speed in in/s at this RPM and launch angle: exitSpeed(rpm) × ShotTables.COAX_EXIT_SPEED_ANGLE's multiplier. */
    public static double exitSpeed(double rpm, double launchDeg) {
        if (Double.isNaN(launchDeg)) return Double.NaN;
        return exitSpeed(rpm) * LookupTable.at(ShotTables.COAX_EXIT_SPEED_ANGLE, launchDeg);
    }

    /**
     * Ball center height above the tiles after travelling `distance` inches along
     * the floor, launched at launchDeg with `speed` in/s. -Infinity if it never
     * gets there (launchDeg >= 90 or speed <= 0).
     */
    public static double heightAt(double distance, double launchDeg, double speed) {
        // Written as "not (...)" so a NaN counts as "never gets there" too.
        if (!(speed > 0) || !(launchDeg < 90) || !(launchDeg > -90)) return Double.NEGATIVE_INFINITY;
        double angle = Math.toRadians(launchDeg);
        double cos = Math.cos(angle);
        if (cos < TINY) return Double.NEGATIVE_INFINITY;
        return LAUNCH_HEIGHT + distance * Math.tan(angle)
                - GRAVITY * distance * distance / (2 * speed * speed * cos * cos);
    }

    /**
     * Launch angle (degrees above horizontal) within [minDeg, maxDeg] that scores
     * from `distance` at this measured RPM, or NaN if none does. Tries the low arc
     * at every height (center first, then outward), then the high arc the same
     * way, each clamped into the range, and keeps the first that still crosses
     * inside the band and clears the lip, also at RPM_MARGIN either side. If none
     * does, walks the range in 0.1° steps. Also NaN for distance <= 0, an RPM of
     * about 0, a NaN or infinite input, or minDeg > maxDeg.
     */
    public static double launchDeg(double distance, double rpm, double minDeg, double maxDeg) {
        if (!Double.isFinite(distance) || !Double.isFinite(rpm) || !Double.isFinite(minDeg) || !Double.isFinite(maxDeg)) return Double.NaN;
        if (distance <= 0 || minDeg > maxDeg || !tunablesAreNumbers()) return Double.NaN;
        if (!(exitSpeed(rpm) >= TINY)) return Double.NaN;
        // Cheap check first. This runs every loop while the wheel spins up, and the
        // searches below (the last one most of all) are slow when nothing scores.
        if (!canReachBand(distance, rpm)) return Double.NaN;

        // Every low arc before any high arc, so the hood moves smoothly as the RPM
        // changes: a high arc at the center could otherwise win over a low arc a
        // little off center, and a small RPM change would flip the hood onto a lob
        // in the middle of a burst.
        double angle = firstScoringArc(distance, rpm, false, minDeg, maxDeg);
        if (!Double.isNaN(angle)) return angle;
        // No low arc scores. If that's because the wheel is too fast (the low arc through the
        // center would be flatter than the hood can go), don't lob: the hood would swing 30
        // degrees or more and back again as the wheel slows. Wait for the wheel instead.
        // High arcs stay for a wheel that's too slow, or a low arc that hits the lip up close.
        double centerLowArc = arcDeg(distance, exitSpeed(rpm, minDeg), TARGET_HEIGHT, false);
        if (centerLowArc < minDeg) return Double.NaN;
        angle = firstScoringArc(distance, rpm, true, minDeg, maxDeg);
        if (!Double.isNaN(angle)) return angle;
        // Last try: up close the sampled heights can all fall between the band and the lip.
        return middleOfFlattestScoringStretch(distance, rpm, minDeg, maxDeg);
    }

    /**
     * Seconds from leaving the shooter to reaching `distance` along the floor:
     * distance / (exitSpeed(rpm, launchDeg) * cos(launchDeg)). NaN if the answer
     * isn't a number, the distance is negative, or the ball barely moves forward.
     */
    public static double flightTime(double distance, double rpm, double launchDeg) {
        if (!Double.isFinite(distance) || distance < 0) return Double.NaN;
        double forwardSpeed = exitSpeed(rpm, launchDeg) * Math.cos(Math.toRadians(launchDeg));
        // Also false for a NaN, so a NaN RPM or angle gives NaN.
        if (!(forwardSpeed > TINY)) return Double.NaN;
        double seconds = distance / forwardSpeed;
        return Double.isFinite(seconds) ? seconds : Double.NaN;
    }

    /**
     * RPM for a COAX_RPM starting table: the slowest RPM (up to maxRpm) at which
     * launchDeg still finds an angle in [minDeg, maxDeg] with the wheel anywhere
     * within SEED_RPM_WINDOW of it, checked every 25 RPM. If no RPM has that much
     * room, the middle of the longest unbroken stretch of RPMs that score. NaN if
     * none scores at all.
     *
     * Why: the coax hood is there to make up for RPM error, so the seed is where
     * it has room both ways, and the slowest such RPM, because a slower wheel
     * recovers faster between balls.
     */
    public static double seedRpm(double distance, double minDeg, double maxDeg, double maxRpm) {
        if (!Double.isFinite(distance) || !Double.isFinite(minDeg) || !Double.isFinite(maxDeg) || !Double.isFinite(maxRpm)) return Double.NaN;
        if (distance <= 0 || minDeg > maxDeg || !Double.isFinite(SEED_RPM_WINDOW)) return Double.NaN;

        // Which RPMs, every 25 from 500 up to maxRpm, score.
        double top = Math.min(maxRpm, MOST_RPM);
        int count = (int) Math.floor((top - SCAN_START_RPM) / SCAN_STEP_RPM) + 1;
        if (count <= 0) return Double.NaN;
        boolean[] scores = new boolean[count];
        for (int i = 0; i < count; i++) {
            scores[i] = !Double.isNaN(launchDeg(distance, scanRpm(i), minDeg, maxDeg));
        }

        // The slowest RPM with every step within the window scoring. (Capped at the
        // scan's length: a wider window can't fit anyway, and 2 * window + 1 stays small.)
        int window = (int) Math.min(Math.ceil(Math.abs(SEED_RPM_WINDOW) / SCAN_STEP_RPM), count);
        int run = 0;
        for (int i = 0; i < count; i++) {
            run = scores[i] ? run + 1 : 0;
            // Steps i - 2 * window to i all score: the middle one has the whole window.
            if (run >= 2 * window + 1) return scanRpm(i - window);
        }

        // No RPM has the whole window: the middle of the longest run that scores.
        int bestStart = -1, bestLength = 0;
        run = 0;
        for (int i = 0; i < count; i++) {
            run = scores[i] ? run + 1 : 0;
            if (run > bestLength) {
                bestLength = run;
                bestStart = i - run + 1;
            }
        }
        if (bestLength == 0) return Double.NaN;
        return scanRpm(bestStart + (bestLength - 1) / 2);
    }

    /** The launch angle at seedRpm's RPM, or NaN. */
    public static double seedLaunchDeg(double distance, double minDeg, double maxDeg, double maxRpm) {
        double rpm = seedRpm(distance, minDeg, maxDeg, maxRpm);
        if (Double.isNaN(rpm)) return Double.NaN;
        return launchDeg(distance, rpm, minDeg, maxDeg);
    }

    /**
     * RPM that lands a pass (ball center at PASS_LANDING_HEIGHT) `distance`
     * inches away along the floor when launched at launchDeg. From the same
     * physics with the landing spot as the target:
     *   v² = g x² / (2 cos²(a) (x tan(a) - (PASS_LANDING_HEIGHT - LAUNCH_HEIGHT)))
     * NaN if no speed does it (aimed into the floor, or straight up) or a number
     * isn't finite.
     */
    public static double passRpm(double distance, double launchDeg) {
        if (!Double.isFinite(distance) || !Double.isFinite(launchDeg) || distance <= 0 || !passTunablesAreNumbers()) return Double.NaN;
        return rpmFor(speedThrough(distance, launchDeg, PASS_LANDING_HEIGHT), launchDeg);
    }

    /**
     * Launch angle in [minDeg, maxDeg] that lands a pass `distance` inches away
     * at this measured RPM, or NaN. A pass is a lob at a chosen angle, so of the
     * two arcs it keeps the one closer to preferDeg. The angle is accepted only
     * if, with its own exit speed, the ball lands within PASS_LANDING_TOLERANCE
     * of `distance`. No band or lip: a pass lands on the floor.
     */
    public static double passLaunchDeg(double distance, double rpm, double preferDeg, double minDeg, double maxDeg) {
        if (!Double.isFinite(distance) || !Double.isFinite(rpm) || !Double.isFinite(preferDeg)
                || !Double.isFinite(minDeg) || !Double.isFinite(maxDeg)) return Double.NaN;
        if (distance <= 0 || minDeg > maxDeg || !passTunablesAreNumbers()) return Double.NaN;
        if (!(exitSpeed(rpm) >= TINY)) return Double.NaN;

        double guess = clamp(preferDeg, minDeg, maxDeg);
        double low = solveArc(distance, rpm, PASS_LANDING_HEIGHT, false, guess, minDeg, maxDeg);
        double high = solveArc(distance, rpm, PASS_LANDING_HEIGHT, true, guess, minDeg, maxDeg);
        boolean lowLands = landsNear(distance, rpm, low);
        boolean highLands = landsNear(distance, rpm, high);
        if (lowLands && highLands) {
            return Math.abs(low - preferDeg) <= Math.abs(high - preferDeg) ? low : high;
        }
        if (lowLands) return low;
        if (highLands) return high;
        return Double.NaN;
    }

    /** The first angle on one arc (low or high) that scores, trying heights from the band's center outward, or NaN. */
    private static double firstScoringArc(double distance, double rpm, boolean high, double minDeg, double maxDeg) {
        int samples = Math.max(1, Math.min(TARGET_SAMPLES, MOST_SAMPLES));
        double guess = (minDeg + maxDeg) / 2;
        for (int i = 0; i < samples; i++) {
            double angle = solveArc(distance, rpm, targetHeight(i, samples), high, guess, minDeg, maxDeg);
            if (scoresWithMargin(distance, rpm, angle)) return angle;
        }
        return Double.NaN;
    }

    /**
     * Walks from minDeg to maxDeg in 0.1° steps and returns the middle of the first
     * (flattest) unbroken stretch of angles that score with the margin, or NaN.
     * The middle leaves the most room for the hood being a little off.
     */
    private static double middleOfFlattestScoringStretch(double distance, double rpm, double minDeg, double maxDeg) {
        int steps = (int) Math.min(Math.floor((maxDeg - minDeg) / ANGLE_STEP_DEG + 1e-9), MOST_ANGLE_STEPS);
        int first = -1, last = -1;
        for (int i = 0; i <= steps; i++) {
            if (scoresWithMargin(distance, rpm, stepAngle(i, minDeg, maxDeg))) {
                if (first < 0) first = i;
                last = i;
            } else if (first >= 0) {
                break;
            }
        }
        if (first < 0) return Double.NaN;
        return stepAngle(first + (last - first) / 2, minDeg, maxDeg);
    }

    private static double stepAngle(int step, double minDeg, double maxDeg) {
        return Math.min(minDeg + ANGLE_STEP_DEG * step, maxDeg);
    }

    /**
     * One arc's angle through `height` at `distance`, clamped into [minDeg, maxDeg],
     * or NaN if the ball is too slow to get there. The exit speed depends on the
     * angle, so solve with the speed at the guess, then again with the speed at
     * that answer, until it settles.
     */
    private static double solveArc(double distance, double rpm, double height, boolean high,
                                   double guessDeg, double minDeg, double maxDeg) {
        double angle = guessDeg;
        for (int round = 0; round < ROUNDS; round++) {
            double next = clamp(arcDeg(distance, exitSpeed(rpm, angle), height, high), minDeg, maxDeg);
            if (Double.isNaN(next)) return Double.NaN;
            boolean settled = Math.abs(next - angle) < SETTLED_DEG;
            angle = next;
            if (settled) break;
        }
        return angle;
    }

    /**
     * Angle (degrees, not clamped) of the low or high arc through `height` at
     * `distance` for this exit speed, or NaN if no angle gets the ball there.
     */
    private static double arcDeg(double distance, double speed, double height, boolean high) {
        if (!(speed > TINY)) return Double.NaN;
        double v2 = speed * speed;
        double rise = height - LAUNCH_HEIGHT;
        // Below 0, no angle reaches this height at this speed.
        double root = v2 * v2 - GRAVITY * (GRAVITY * distance * distance + 2 * rise * v2);
        if (!(root >= 0)) return Double.NaN;
        double tan = high
                ? (v2 + Math.sqrt(root)) / (GRAVITY * distance)
                : (v2 - Math.sqrt(root)) / (GRAVITY * distance);
        return Math.toDegrees(Math.atan(tan));
    }

    /**
     * False if the ball can't even reach the bottom of the band at the slowest RPM a shot
     * has to score at (rpm - RPM_MARGIN), at any angle, with the biggest angle multiplier.
     * Then no angle can score with the margin, so launchDeg can stop right away.
     */
    private static boolean canReachBand(double distance, double rpm) {
        double slowest = Math.abs(rpm) - Math.abs(RPM_MARGIN);
        if (slowest <= 0) return false;
        double biggestMultiplier = 0;
        for (double[] row : ShotTables.COAX_EXIT_SPEED_ANGLE) biggestMultiplier = Math.max(biggestMultiplier, row[1]);
        double fastest = exitSpeed(slowest) * biggestMultiplier;
        return !Double.isNaN(arcDeg(distance, fastest, TARGET_HEIGHT - Math.abs(TARGET_BAND) - BAND_SLACK, false));
    }

    /**
     * True if this angle scores at this RPM and also with the wheel RPM_MARGIN
     * slower or faster, at the same angle: the measured RPM lags and is noisy.
     */
    private static boolean scoresWithMargin(double distance, double rpm, double launchDeg) {
        double wheel = Math.abs(rpm);
        double margin = Math.abs(RPM_MARGIN);
        return scores(distance, wheel, launchDeg)
                && scores(distance, wheel - margin, launchDeg)
                && scores(distance, wheel + margin, launchDeg);
    }

    /** True if this angle, with its own exit speed, crosses inside the band and clears the lip. */
    private static boolean scores(double distance, double rpm, double launchDeg) {
        if (Double.isNaN(launchDeg)) return false;
        double speed = exitSpeed(rpm, launchDeg);
        double height = heightAt(distance, launchDeg, speed);
        double band = Math.abs(TARGET_BAND) + BAND_SLACK;
        if (!(height >= TARGET_HEIGHT - band && height <= TARGET_HEIGHT + band)) return false;
        return clearsLip(distance, launchDeg, speed);
    }

    /** True if the ball passes above LIP_HEIGHT, LIP_BEFORE inches before the target. Always true when the robot is closer than that. */
    private static boolean clearsLip(double distance, double launchDeg, double speed) {
        double lipDistance = distance - LIP_BEFORE;
        if (lipDistance <= 0) return true;
        return heightAt(lipDistance, launchDeg, speed) >= LIP_HEIGHT;
    }

    /** True if a pass at this angle, with its own exit speed, lands within PASS_LANDING_TOLERANCE of `distance`. */
    private static boolean landsNear(double distance, double rpm, double launchDeg) {
        if (Double.isNaN(launchDeg)) return false;
        double landing = landingDistance(launchDeg, exitSpeed(rpm, launchDeg));
        return Math.abs(landing - distance) <= Math.abs(PASS_LANDING_TOLERANCE);
    }

    /**
     * How far along the floor the ball center comes down to PASS_LANDING_HEIGHT:
     * the far answer of heightAt(x) = PASS_LANDING_HEIGHT. NaN if it never does.
     */
    private static double landingDistance(double launchDeg, double speed) {
        if (!(speed > TINY) || !(launchDeg < 90) || !(launchDeg > -90)) return Double.NaN;
        double angle = Math.toRadians(launchDeg);
        double cos = Math.cos(angle);
        if (cos < TINY) return Double.NaN;
        // heightAt(x) = PASS_LANDING_HEIGHT as a x² - b x + c = 0.
        double a = GRAVITY / (2 * speed * speed * cos * cos);
        double b = Math.tan(angle);
        double c = PASS_LANDING_HEIGHT - LAUNCH_HEIGHT;
        double root = b * b - 4 * a * c;
        if (!(root >= 0)) return Double.NaN;
        return (b + Math.sqrt(root)) / (2 * a);
    }

    /**
     * The i-th height to try: the center first, then one step up, one step down,
     * two steps up, two steps down, ... out to the band's edges.
     */
    private static double targetHeight(int i, int samples) {
        if (i == 0) return TARGET_HEIGHT;
        int stepsEachSide = samples / 2;
        int step = (i + 1) / 2;
        double offset = Math.abs(TARGET_BAND) * step / stepsEachSide;
        return i % 2 == 1 ? TARGET_HEIGHT + offset : TARGET_HEIGHT - offset;
    }

    /**
     * Exit speed (in/s) that sends a ball launched at launchDeg through `height`
     * at `distance`, from heightAt(distance) = height solved for v:
     *   v² = g x² / (2 cos²(a) (x tan(a) - (height - LAUNCH_HEIGHT)))
     * NaN if no speed does it (the angle is too flat to ever get there).
     */
    private static double speedThrough(double distance, double launchDeg, double height) {
        if (!(launchDeg < 90) || !(launchDeg > -90)) return Double.NaN;
        double angle = Math.toRadians(launchDeg);
        double cos = Math.cos(angle);
        double rise = distance * Math.tan(angle) - (height - LAUNCH_HEIGHT);
        if (cos < TINY || !(rise > TINY)) return Double.NaN;
        double v2 = GRAVITY * distance * distance / (2 * cos * cos * rise);
        return Double.isFinite(v2) ? Math.sqrt(v2) : Double.NaN;
    }

    /**
     * The RPM whose exit speed at launchDeg is `speed`, or NaN if even MOST_RPM is
     * too slow. The in/s per RPM is itself looked up by RPM, so we can't just
     * divide. Instead, halve the RPM range over and over: too slow in the middle
     * means the answer is above it, too fast means below. 50 halvings is far
     * finer than any flywheel holds.
     */
    private static double rpmFor(double speed, double launchDeg) {
        if (!Double.isFinite(speed) || !(speed > TINY)) return Double.NaN;
        if (!(exitSpeed(MOST_RPM, launchDeg) >= speed)) return Double.NaN;
        double low = 0;
        double high = MOST_RPM;
        for (int i = 0; i < 50; i++) {
            double middle = (low + high) / 2;
            if (exitSpeed(middle, launchDeg) < speed) low = middle;
            else high = middle;
        }
        return (low + high) / 2;
    }

    private static double scanRpm(int step) {
        return SCAN_START_RPM + SCAN_STEP_RPM * step;
    }

    /** NaN stays NaN. */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static boolean tunablesAreNumbers() {
        return Double.isFinite(LAUNCH_HEIGHT) && Double.isFinite(TARGET_HEIGHT) && Double.isFinite(TARGET_BAND)
                && Double.isFinite(LIP_HEIGHT) && Double.isFinite(LIP_BEFORE) && Double.isFinite(RPM_MARGIN);
    }

    private static boolean passTunablesAreNumbers() {
        return Double.isFinite(LAUNCH_HEIGHT) && Double.isFinite(PASS_LANDING_HEIGHT) && Double.isFinite(PASS_LANDING_TOLERANCE);
    }
}
