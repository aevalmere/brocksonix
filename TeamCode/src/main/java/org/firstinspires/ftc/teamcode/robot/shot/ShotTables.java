package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Lookup tables. Each row is {distance to target in inches, value}, except
 * COAX_EXIT_SPEED ({flywheel RPM, value}) and COAX_EXIT_SPEED_ANGLE ({launch
 * angle in degrees, value}). Values in between are interpolated. All numbers
 * here are placeholders: fill them from real shots (TUNING_GUIDE.md, step 7;
 * the COAX tables are step 16). COAX_RPM starts from physics
 * (HoodSolver.seedRpm), the others are guesses.
 *
 * These are the numbers to tune live in Panels. Panels edits are lost when the
 * app restarts, so copy the final numbers back here.
 *
 * Panels can change the numbers but can't add or remove rows. To add a row,
 * edit the code. Keep the first column increasing from top to bottom: the
 * lookup assumes it, and a row out of order makes it skip rows or return a
 * wrong value with no error. orderProblem() checks this (Shooter Test and the
 * robot's warnings show it).
 */
@Configurable
public final class ShotTables {
    private ShotTables() {}

    // TODO(7): Shooter Test at measured distances, about every 10 in. from 20 to 110. Check a few with the other cell up.
    /** Flywheel RPM that lands a ball in the upward cell from this distance. */
    public static double[][] SHOT_RPM = {
            {20, 2600},
            {50, 3200},
            {80, 3900},
            {110, 4600},
    };

    // TODO(11): Slow-motion video of shots at near, mid and far distances.
    /** Seconds from leaving the shooter to reaching the cell. */
    public static double[][] SHOT_FLIGHT_TIME = {
            {20, 0.45},
            {110, 0.90},
    };

    // TODO(7): Lower at long range if back-to-back balls miss because RPM hasn't recovered.
    /** Rail power while feeding a shot. */
    public static double[][] SHOT_FEED_POWER = {
            {20, 1.0},
            {110, 1.0},
    };

    // TODO(16): Once passing is on (FireControl.PASS_ENABLED): fill it from real passes like SHOT_RPM. Its first and last distances also limit where coax passes fire (G405): check a pass from the far end can't leave the field.
    /**
     * Flywheel RPM for a pass that lands at the pass target from this distance, with the static hood.
     * With the coax hood, passes come from HoodSolver instead, but only inside this table's distances (G405).
     * Passing is on hold until the coax hood is on the robot (FireControl.PASS_ENABLED).
     */
    public static double[][] PASS_RPM = {
            {24, 1500},
            {165, 3200},
    };

    // TODO(16): Once passing is on: lower it if back-to-back passes land short or scatter.
    /** Rail power while feeding a pass. */
    public static double[][] PASS_FEED_POWER = {
            {24, 1.0},
            {165, 1.0},
    };

    // TODO(16): Seeds from physics. On the field: if shots from a distance are too steep and slow, raise the RPM there; if too flat, lower it.
    /**
     * Flywheel target RPM with the coax hood. HoodSolver then picks the angle
     * from the wheel's measured RPM. Seeded with HoodSolver.seedRpm(distance,
     * 40, 75, 5500), SEED_RPM_WINDOW 300 and RPM_MARGIN 50: the slowest RPM
     * where the hood still scores with the wheel 300 RPM off either way. At 20,
     * 95 and 110 in. no RPM has that much room, so those rows are the middle of
     * the RPMs that score. From 20 in. only about 1850 RPM scores at all.
     * The seeds also depend on the physics numbers that are still guesses:
     * HoodSolver.LAUNCH_HEIGHT and TARGET_BAND, and COAX_EXIT_SPEED below. After
     * measuring those, re-seed from Coax Hood Test's init screen.
     */
    public static double[][] COAX_RPM = {
            {20, 1850},
            {35, 2025},
            {50, 2125},
            {65, 2225},
            {80, 2325},
            {95, 2400},
            {110, 2400},
    };

    // TODO(16): Slow-motion video of a shot at a known hood angle and RPM: (forward in/s) / cos(angle) / RPM. Or in the Coax Hood Test: balls land high at an RPM = they're faster than this says, so raise it at that RPM; low = lower it.
    /**
     * Rows are {flywheel RPM, ball exit speed in in/s per RPM}, keyed by RPM (not
     * distance). About 80% of the 72 mm wheel's surface speed (0.148 in/s per RPM).
     */
    public static double[][] COAX_EXIT_SPEED = {
            {2000, 0.12},
            {5000, 0.12},
    };

    // TODO(16): Calibrate COAX_EXIT_SPEED at one angle first. Then fire at a flat and a steep angle at the same RPM: balls land high at an angle = raise that angle's multiplier; low = lower it.
    /**
     * Rows are {launch angle in degrees, multiplier on COAX_EXIT_SPEED}, keyed by
     * angle. On a coax hood the ball wraps the wheel for a different distance at
     * each angle, so it can leave faster or slower at the same RPM.
     */
    public static double[][] COAX_EXIT_SPEED_ANGLE = {
            {40, 1.0},
            {75, 1.0},
    };

    /** Null if every table is in order, otherwise a message naming the first one that isn't. */
    public static String orderProblem() {
        if (!increasing(SHOT_RPM)) return "SHOT_RPM distances are not increasing";
        if (!increasing(SHOT_FLIGHT_TIME)) return "SHOT_FLIGHT_TIME distances are not increasing";
        if (!increasing(SHOT_FEED_POWER)) return "SHOT_FEED_POWER distances are not increasing";
        if (!increasing(PASS_RPM)) return "PASS_RPM distances are not increasing";
        if (!increasing(PASS_FEED_POWER)) return "PASS_FEED_POWER distances are not increasing";
        if (!increasing(COAX_RPM)) return "COAX_RPM distances are not increasing";
        if (!increasing(COAX_EXIT_SPEED)) return "COAX_EXIT_SPEED RPMs are not increasing";
        if (!increasing(COAX_EXIT_SPEED_ANGLE)) return "COAX_EXIT_SPEED_ANGLE angles are not increasing";
        return null;
    }

    private static boolean increasing(double[][] table) {
        for (int i = 1; i < table.length; i++) {
            // Written as "not greater" so a NaN counts as out of order too.
            if (!(table[i][0] > table[i - 1][0])) return false;
        }
        return true;
    }
}
