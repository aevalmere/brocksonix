package org.firstinspires.ftc.teamcode.shot;

/**
 * Distance tables, each row {distance to target in inches, value}, sorted by
 * distance. Values in between are interpolated. All numbers here are
 * placeholders: fill them from real shots (TUNING_GUIDE.md, step 7).
 */
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
}
