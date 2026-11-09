package org.firstinspires.ftc.teamcode.robot.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.LookupTable;

import java.util.Arrays;

/**
 * The static hood's shot RPM (seven measured spots, by angle then distance, see
 * staticShotRpm) and lookup tables. Each table row is {distance to target in
 * inches, value}, except COAX_EXIT_SPEED ({flywheel RPM, value}) and
 * COAX_EXIT_SPEED_ANGLE ({launch angle in degrees, value}). Values in between are
 * interpolated, and past either end a table gives its end value. The seven shot
 * spots' RPMs are measured (or scaled from measured, see the spots), and
 * SHOT_DISTANCE_SCALE is worked out from four of them.
 * The other numbers here are placeholders: fill them from real shots
 * (TUNING_GUIDE.md, step 7; the COAX tables are step 16). COAX_RPM starts from physics
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

    // ---- Static hood RPM: by the angle the shot comes in at first, then by distance. ----
    // Two spots about 46 in. from the cell needed 2350 head-on and 2200 from 48 degrees off, so the
    // angle matters even at one distance. The angle is measured from the up cell's front: 0 = straight
    // in from the wall the cell faces (the audience wall for the AUDIENCE cell, the far wall for the
    // FAR cell), 180 = from straight behind it, and left and right are the same. Each spot below is
    // where the robot stood, written for red with the AUDIENCE cell up; the code works out its angle
    // and distance (staticShotRpm). The same angles hold for the FAR cell and for blue.
    //
    // Closer needs faster: with the hood fixed, a faster ball flies a higher arc, and a close shot
    // needs that to come down into the opening. HIVE_EDGE and CLOSE are about the same angle (15 and
    // 19 deg) but 47.8 and 32.5 in. away. CORNER and GARDEN_WALL are both about 47 deg, 46.1 and
    // 71.2 in. away. SHOT_DISTANCE_SCALE carries both pairs.
    //
    // The RPMs are for the flywheel with the added MOI weight. HEAD_ON (2225) and GARDEN_WALL
    // (2200) were measured with it; the others were measured before it and scaled by the same
    // 2225 / 2350 that HEAD_ON dropped by.

    // TODO(7): Measured with the MOI weight. Shooter Test shows the angle and this RPM where the robot stands.
    /** Head-on: intake on the audience wall, in line with the HIVE (x = Field.RED_HIVE_X). 46.3 in., 0 deg. */
    public static double HEAD_ON_X = 58.0;
    public static double HEAD_ON_Y = 7.99;
    public static double HEAD_ON_RPM = 2225;
    // TODO(7): Scaled for the MOI weight from 2250 (before it). The pose is worked out: x = the HIVE frame's near edge (FieldZones, 46.02), y = intake on the audience wall. Check the pose and RPM with Shooter Test.
    /** Audience wall, in line with the HIVE frame's edge nearest our alliance wall. 47.8 in., about 15 deg. */
    public static double HIVE_EDGE_X = 46.02;
    public static double HIVE_EDGE_Y = 7.99;
    public static double HIVE_EDGE_RPM = 2130;
    // TODO(7): Scaled for the MOI weight from 4000 (before it). At the tile seams 2 tiles off our wall and 1 tile off the audience wall (141.5 / 6 per tile). Check the pose and RPM with Shooter Test.
    /** Close in front: where the second tile seam from our wall crosses the first seam off the audience wall. 32.5 in., about 19 deg. */
    public static double CLOSE_X = 47.17;
    public static double CLOSE_Y = 23.58;
    public static double CLOSE_RPM = 3787;
    // TODO(7): Scaled for the MOI weight from 2200 (before it). Check it with Shooter Test where the robot stands.
    /** GARDEN corner: robot centered where the 4 corner tiles meet, one tile (141.5 / 6) off both walls. 46.1 in., about 48 deg. */
    public static double CORNER_X = 23.58;
    public static double CORNER_Y = 23.58;
    public static double CORNER_RPM = 2083;
    // TODO(7): Measured with the MOI weight, but the pose is worked out: back on our alliance wall (AgateFlow.ROBOT_BACK, 5.72), side on the audience wall (Field.CENTER_TO_SIDE_WALL, 5.87). Check it with Shooter Test's pose line.
    /** GARDEN corner on the walls: back against our alliance wall, intake facing the other alliance, side on the audience wall. 71.2 in., about 47 deg. */
    public static double GARDEN_WALL_X = 5.72;
    public static double GARDEN_WALL_Y = 5.87;
    public static double GARDEN_WALL_RPM = 2200;
    // TODO(7): Scaled for the MOI weight from 2400 (before it). At the FLOWER intake pose (IntakePoses.flowerPose, ALLIANCE_WALL). Check the pose and RPM with Shooter Test.
    /** Intaking from the FLOWER on our alliance wall. 45.2 in., about 81 deg. */
    public static double FLOWER_X = 13.34;
    public static double FLOWER_Y = 47.3;
    public static double FLOWER_RPM = 2272;
    // TODO(7): Scaled for the MOI weight from 2575 (before it). The spot is a guess: "a bit up" from the field center was put 4 in. toward the far wall. Stand where it scores and put that pose here.
    /** Behind the cell: a bit up from the field center. About 24 in., about 148 deg. */
    public static double BEHIND_X = 70.75;
    public static double BEHIND_Y = 74.75;
    public static double BEHIND_RPM = 2438;

    // TODO(7): Worked out from two spot pairs. Fire head-on from 36 to 40 in. and from 56 to 62 in.: short = raise that row, long = lower it. Front shots under about 32 in. likely can't score at any RPM.
    /**
     * Rows are {distance to the cell in inches, RPM multiplier}, 1.0 at 46 in., where most spots
     * were measured. Up to 46 in. it is fitted to RPM^2 ~ d^2 / (d - 29.9), the shape a fixed
     * launch angle gives (the ball must still be climbing as it passes the opening's height);
     * 29.9 in. is where that runs out, worked out from HIVE_EDGE and CLOSE. Past 46 in. it is the
     * same shape, bent up in a straight line with distance so it meets GARDEN_WALL (1.056 at
     * 71.2 in. against CORNER). Each spot's own RPM is still exact at its spot: the spots are
     * scaled back to 46 in. with this table before the angle slide.
     */
    public static double[][] SHOT_DISTANCE_SCALE = {
            {32, 1.928},
            {33, 1.636},
            {34, 1.465},
            {36, 1.272},
            {38, 1.165},
            {40, 1.098},
            {43, 1.036},
            {46, 1.0},
            {50, 0.987},
            {56, 0.991},
            {62, 1.010},
            {72, 1.060},
    };

    // TODO(7): Fire from beside the cell (about 90 deg) at 35 and 45 in. If the closer one needs more RPM like the front does, raise both angles.
    /**
     * The distance scale fades out between these angles off the front, in degrees: all of it up
     * to DISTANCE_FULL_DEG, none past DISTANCE_NONE_DEG. The BEHIND spot scored from 24 in.,
     * closer than the front's limit, so the front's curve doesn't fit behind the cell. There
     * the RPM goes by angle alone.
     */
    public static double DISTANCE_FULL_DEG = 90;
    public static double DISTANCE_NONE_DEG = 145;

    // TODO(7): The far end reaches GARDEN_WALL (71.2 in.). Widen either end only after checking shots there.
    /** Static hood shots fire only from this far to this far from the cell, in inches (OUT_OF_RANGE outside). */
    public static double SHOT_MIN_DISTANCE = 20;
    public static double SHOT_MAX_DISTANCE = 72;

    // TODO(7): Fire from the GARDEN corner and from straight beside the HIVE. Balls hit the near side of the opening = raise it; bounce off the far wall and out = lower it. 0 turns the shift off.
    /**
     * Aim shift across the cell's opening, in inches. A shot from the side aims this far toward the
     * HIVE side wall on the far side of the opening, the wall that faces the robot, times how far off
     * head-on the shot is (sine of the angle): 0 head-on, all of it from straight beside the cell.
     * Turret and distance use the shifted point; the RPM still uses the angle to the cell itself.
     * The shift never goes past the HIVE wall (Field.CELL_OPENING_HALF_WIDTH), whatever this is set to.
     * Placeholder: a guess until measured.
     */
    public static double AIM_SHIFT_IN = 2.0;

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

    // TODO(7): Main TeleOp, pick an up cell (driver 2 LB/RB) and fire from the other half: fill it from real passes, like the shot spots. Check from the far end of the table that no pass can leave the field (G405).
    // TODO(16): With the coax hood, passes come from HoodSolver but only inside this table's distances: check a pass from the far end again in COAX (G405).
    /**
     * Flywheel RPM for a pass that lands at the pass target from this distance, with the static hood.
     * With the coax hood, passes come from HoodSolver instead, but only inside this table's distances (G405).
     * Outside them a pass is out of range and doesn't fire.
     */
    public static double[][] PASS_RPM = {
            {24, 1500},
            {165, 3200},
    };

    // TODO(7): Lower it if back-to-back passes land short or scatter.
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

    /**
     * Static hood RPM for a shot from (x, y) into this cell. Angle first: each spot's RPM is scaled
     * back to 46 in. (divided by its distanceScale), then slid in a straight line by the angle off
     * the cell's front between spots (past the end spots it stays at theirs). Then distance: times
     * distanceScale where the robot is. At a spot that gives the spot's own RPM.
     */
    public static double staticShotRpm(double x, double y, Pose cell) {
        Pose audienceCell = Field.cellTarget(Alliance.RED, Cell.AUDIENCE);
        double[][] spots = {
                {HEAD_ON_X, HEAD_ON_Y, HEAD_ON_RPM},
                {HIVE_EDGE_X, HIVE_EDGE_Y, HIVE_EDGE_RPM},
                {CLOSE_X, CLOSE_Y, CLOSE_RPM},
                {CORNER_X, CORNER_Y, CORNER_RPM},
                {GARDEN_WALL_X, GARDEN_WALL_Y, GARDEN_WALL_RPM},
                {FLOWER_X, FLOWER_Y, FLOWER_RPM},
                {BEHIND_X, BEHIND_Y, BEHIND_RPM},
        };
        double[][] byAngle = new double[spots.length][];
        for (int i = 0; i < spots.length; i++) {
            double sx = spots[i][0], sy = spots[i][1];
            byAngle[i] = new double[]{angleOffFront(sx, sy, audienceCell),
                    spots[i][2] / distanceScale(sx, sy, audienceCell)};
        }
        // Sorted by angle, so a spot moved in Panels can't put the rows out of order.
        Arrays.sort(byAngle, (a, b) -> Double.compare(a[0], b[0]));
        return LookupTable.at(byAngle, angleOffFront(x, y, cell)) * distanceScale(x, y, cell);
    }

    /**
     * SHOT_DISTANCE_SCALE at the distance from (x, y) to the cell, faded toward 1 between
     * DISTANCE_FULL_DEG and DISTANCE_NONE_DEG off the front.
     */
    public static double distanceScale(double x, double y, Pose cell) {
        double scale = LookupTable.at(SHOT_DISTANCE_SCALE, Math.hypot(x - cell.x(), y - cell.y()));
        double angle = angleOffFront(x, y, cell);
        double share;
        if (angle <= DISTANCE_FULL_DEG) share = 1;
        else if (angle >= DISTANCE_NONE_DEG) share = 0;
        else share = (DISTANCE_NONE_DEG - angle) / (DISTANCE_NONE_DEG - DISTANCE_FULL_DEG);
        return 1 + (scale - 1) * share;
    }

    /**
     * Where a shot from (x, y) aims: the cell moved AIM_SHIFT_IN x sin(angle off front) along x,
     * away from the robot's side, toward the HIVE wall that faces it. Head-on it is the cell.
     * The shift is held within Field.CELL_OPENING_HALF_WIDTH, so the point is always on the HIVE:
     * inside the opening, at most on one of its side walls.
     */
    public static Pose shiftedAim(double x, double y, Pose cell) {
        double angle = Math.toRadians(angleOffFront(x, y, cell));
        double away = x < cell.x() ? 1 : -1;
        double shift = away * AIM_SHIFT_IN * Math.sin(angle);
        double wall = Field.CELL_OPENING_HALF_WIDTH;
        shift = Math.max(-wall, Math.min(wall, shift));
        return new Pose(cell.x() + shift, cell.y());
    }

    /**
     * Degrees between the cell's front and the line from the cell to (x, y): 0 = straight in front,
     * 180 = straight behind, the same on either side. A cell on the far half faces the far wall
     * (+y), one on the audience half the audience wall (-y). This holds for both alliances, since
     * the cells only move in x.
     */
    public static double angleOffFront(double x, double y, Pose cell) {
        double dx = x - cell.x();
        double dy = y - cell.y();
        double length = Math.hypot(dx, dy);
        if (length < 1e-9) return 0;
        double front = cell.y() > Field.SIZE / 2 ? 1 : -1;
        return Math.toDegrees(Math.acos(Range.clip(front * dy / length, -1, 1)));
    }

    /** Null if every table is in order, otherwise a message naming the first one that isn't. */
    public static String orderProblem() {
        if (!increasing(SHOT_DISTANCE_SCALE)) return "SHOT_DISTANCE_SCALE distances are not increasing";
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
