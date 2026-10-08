package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Distance tables, each row {distance to target in inches, value}. Values in
 * between are interpolated. All numbers here are placeholders: fill them from
 * real shots (TUNING_GUIDE.md, step 7).
 *
 * These are the numbers to tune live in Panels. Panels edits are lost when the
 * app restarts, so copy the final numbers back here.
 *
 * Panels can change the numbers but can't add or remove rows. To add a row,
 * edit the code. Keep the distances increasing from top to bottom: the lookup
 * assumes it, and a row out of order makes it skip rows or return a wrong value
 * with no error. orderProblem() checks this (Shooter Test and the robot's warnings show it).
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

    // TODO(7): Slow-motion video of shots at near, mid and far distances.
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

    /** Flywheel RPM for a pass that lands at the pass target from this distance. On hold until the variable hood. */
    public static double[][] PASS_RPM = {
            {24, 1500},
            {165, 3200},
    };

    public static double[][] PASS_FEED_POWER = {
            {24, 1.0},
            {165, 1.0},
    };

    /** Null if every table is in order, otherwise a message naming the first one that isn't. */
    public static String orderProblem() {
        if (!increasing(SHOT_RPM)) return "SHOT_RPM distances are not increasing";
        if (!increasing(SHOT_FLIGHT_TIME)) return "SHOT_FLIGHT_TIME distances are not increasing";
        if (!increasing(SHOT_FEED_POWER)) return "SHOT_FEED_POWER distances are not increasing";
        if (!increasing(PASS_RPM)) return "PASS_RPM distances are not increasing";
        if (!increasing(PASS_FEED_POWER)) return "PASS_FEED_POWER distances are not increasing";
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
