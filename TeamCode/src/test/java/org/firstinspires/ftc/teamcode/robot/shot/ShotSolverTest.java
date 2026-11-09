package org.firstinspires.ftc.teamcode.robot.shot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.Angles;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * ShotSolver: with PREDICT off the shot is the plain static one (and LEAD's aim point)
 * from where the robot is now, with PREDICT on a robot braking to a stop gets no lead
 * the wrong way, and the turret aims from the turret's horizon while the distance and
 * LEAD use the ball's release.
 */
public class ShotSolverTest {
    private static final double EPS = 1e-9;
    private static final double DT = 0.02;
    // The same at every distance, so LEAD's aim point is easy to work out by hand.
    private static final double FLIGHT_TIME = 0.5;

    private boolean lead, predict, useAccel;
    private double turretLatency, releaseLatency, velocityLatency, accelFilter, maxAccel, maxAngularAccel;
    private int leadIterations;
    private double[][] shotFlightTime, shotFeedPower, distanceScale;
    private double[] spots;
    private double minDistance, maxDistance, aimShift, distanceFullDeg, distanceNoneDeg;

    @Before
    public void remember() {
        lead = ShotSolver.LEAD;
        predict = ShotSolver.PREDICT;
        useAccel = ShotSolver.USE_ACCEL;
        turretLatency = ShotSolver.TURRET_LATENCY_SEC;
        releaseLatency = ShotSolver.RELEASE_LATENCY_SEC;
        velocityLatency = ShotSolver.VELOCITY_LATENCY_SEC;
        accelFilter = ShotSolver.ACCEL_FILTER;
        maxAccel = ShotSolver.MAX_ACCEL;
        maxAngularAccel = ShotSolver.MAX_ANGULAR_ACCEL;
        leadIterations = ShotSolver.LEAD_ITERATIONS;
        shotFlightTime = ShotTables.SHOT_FLIGHT_TIME;
        shotFeedPower = ShotTables.SHOT_FEED_POWER;
        spots = new double[]{ShotTables.HEAD_ON_X, ShotTables.HEAD_ON_Y, ShotTables.HEAD_ON_RPM,
                ShotTables.CORNER_X, ShotTables.CORNER_Y, ShotTables.CORNER_RPM,
                ShotTables.BEHIND_X, ShotTables.BEHIND_Y, ShotTables.BEHIND_RPM,
                ShotTables.HIVE_EDGE_X, ShotTables.HIVE_EDGE_Y, ShotTables.HIVE_EDGE_RPM,
                ShotTables.CLOSE_X, ShotTables.CLOSE_Y, ShotTables.CLOSE_RPM,
                ShotTables.FLOWER_X, ShotTables.FLOWER_Y, ShotTables.FLOWER_RPM,
                ShotTables.GARDEN_WALL_X, ShotTables.GARDEN_WALL_Y, ShotTables.GARDEN_WALL_RPM};
        distanceScale = ShotTables.SHOT_DISTANCE_SCALE;
        distanceFullDeg = ShotTables.DISTANCE_FULL_DEG;
        distanceNoneDeg = ShotTables.DISTANCE_NONE_DEG;
        minDistance = ShotTables.SHOT_MIN_DISTANCE;
        maxDistance = ShotTables.SHOT_MAX_DISTANCE;
        aimShift = ShotTables.AIM_SHIFT_IN;

        // Our own numbers, so these tests don't change when ShotTables is tuned.
        ShotTables.HEAD_ON_X = 58;
        ShotTables.HEAD_ON_Y = 7.99;
        ShotTables.HEAD_ON_RPM = 2225;
        ShotTables.CORNER_X = 23.58;
        ShotTables.CORNER_Y = 23.58;
        ShotTables.CORNER_RPM = 2083;
        ShotTables.GARDEN_WALL_X = 5.72;
        ShotTables.GARDEN_WALL_Y = 5.87;
        ShotTables.GARDEN_WALL_RPM = 2200;
        ShotTables.BEHIND_X = 70.75;
        ShotTables.BEHIND_Y = 74.75;
        ShotTables.BEHIND_RPM = 2438;
        ShotTables.HIVE_EDGE_X = 46.02;
        ShotTables.HIVE_EDGE_Y = 7.99;
        ShotTables.HIVE_EDGE_RPM = 2130;
        ShotTables.CLOSE_X = 47.17;
        ShotTables.CLOSE_Y = 23.58;
        ShotTables.CLOSE_RPM = 3787;
        ShotTables.FLOWER_X = 13.34;
        ShotTables.FLOWER_Y = 47.3;
        ShotTables.FLOWER_RPM = 2272;
        ShotTables.SHOT_DISTANCE_SCALE = new double[][]{
                {32, 1.928}, {33, 1.636}, {34, 1.465}, {36, 1.272}, {38, 1.165},
                {40, 1.098}, {43, 1.036}, {46, 1.0}, {50, 0.987}, {56, 0.991}, {62, 1.010}, {72, 1.060}};
        ShotTables.DISTANCE_FULL_DEG = 90;
        ShotTables.DISTANCE_NONE_DEG = 145;
        ShotTables.SHOT_MIN_DISTANCE = 20;
        ShotTables.SHOT_MAX_DISTANCE = 120;
        // Off, so the aim math below is plain; aimShiftsTowardTheFarWallFromTheSide turns it on.
        ShotTables.AIM_SHIFT_IN = 0;
        ShotTables.SHOT_FLIGHT_TIME = new double[][]{{20, FLIGHT_TIME}, {120, FLIGHT_TIME}};
        ShotTables.SHOT_FEED_POWER = new double[][]{{20, 1.0}, {120, 1.0}};
        // The defaults, in case they were tuned.
        ShotSolver.LEAD = false;
        ShotSolver.PREDICT = false;
        ShotSolver.USE_ACCEL = true;
        ShotSolver.TURRET_LATENCY_SEC = 0.08;
        ShotSolver.RELEASE_LATENCY_SEC = 0.03;
        ShotSolver.VELOCITY_LATENCY_SEC = 0.06;
        ShotSolver.ACCEL_FILTER = 0.7;
        ShotSolver.MAX_ACCEL = 200;
        ShotSolver.MAX_ANGULAR_ACCEL = 20;
        ShotSolver.LEAD_ITERATIONS = 3;
    }

    @After
    public void putBack() {
        ShotSolver.LEAD = lead;
        ShotSolver.PREDICT = predict;
        ShotSolver.USE_ACCEL = useAccel;
        ShotSolver.TURRET_LATENCY_SEC = turretLatency;
        ShotSolver.RELEASE_LATENCY_SEC = releaseLatency;
        ShotSolver.VELOCITY_LATENCY_SEC = velocityLatency;
        ShotSolver.ACCEL_FILTER = accelFilter;
        ShotSolver.MAX_ACCEL = maxAccel;
        ShotSolver.MAX_ANGULAR_ACCEL = maxAngularAccel;
        ShotSolver.LEAD_ITERATIONS = leadIterations;
        ShotTables.SHOT_FLIGHT_TIME = shotFlightTime;
        ShotTables.SHOT_FEED_POWER = shotFeedPower;
        ShotTables.HEAD_ON_X = spots[0];
        ShotTables.HEAD_ON_Y = spots[1];
        ShotTables.HEAD_ON_RPM = spots[2];
        ShotTables.CORNER_X = spots[3];
        ShotTables.CORNER_Y = spots[4];
        ShotTables.CORNER_RPM = spots[5];
        ShotTables.BEHIND_X = spots[6];
        ShotTables.BEHIND_Y = spots[7];
        ShotTables.BEHIND_RPM = spots[8];
        ShotTables.HIVE_EDGE_X = spots[9];
        ShotTables.HIVE_EDGE_Y = spots[10];
        ShotTables.HIVE_EDGE_RPM = spots[11];
        ShotTables.CLOSE_X = spots[12];
        ShotTables.CLOSE_Y = spots[13];
        ShotTables.CLOSE_RPM = spots[14];
        ShotTables.FLOWER_X = spots[15];
        ShotTables.FLOWER_Y = spots[16];
        ShotTables.FLOWER_RPM = spots[17];
        ShotTables.GARDEN_WALL_X = spots[18];
        ShotTables.GARDEN_WALL_Y = spots[19];
        ShotTables.GARDEN_WALL_RPM = spots[20];
        ShotTables.SHOT_DISTANCE_SCALE = distanceScale;
        ShotTables.DISTANCE_FULL_DEG = distanceFullDeg;
        ShotTables.DISTANCE_NONE_DEG = distanceNoneDeg;
        ShotTables.SHOT_MIN_DISTANCE = minDistance;
        ShotTables.SHOT_MAX_DISTANCE = maxDistance;
        ShotTables.AIM_SHIFT_IN = aimShift;
    }

    @Test
    public void eachMeasuredSpotGetsItsOwnRpmOnEitherCellAndAlliance() {
        Pose redAudience = Field.cellTarget(Alliance.RED, Cell.AUDIENCE);
        assertEquals(2225, ShotTables.staticShotRpm(58, 7.99, redAudience), EPS);
        assertEquals(2083, ShotTables.staticShotRpm(23.58, 23.58, redAudience), EPS);
        assertEquals(2438, ShotTables.staticShotRpm(70.75, 74.75, redAudience), EPS);
        assertEquals(2130, ShotTables.staticShotRpm(46.02, 7.99, redAudience), EPS);
        assertEquals(3787, ShotTables.staticShotRpm(47.17, 23.58, redAudience), EPS);
        assertEquals(2272, ShotTables.staticShotRpm(13.34, 47.3, redAudience), EPS);
        assertEquals(2200, ShotTables.staticShotRpm(5.72, 5.87, redAudience), EPS);

        // Head-on is 0 deg, and the corner is about 48: the two ~46 in. shots differ only by angle.
        assertEquals(0, ShotTables.angleOffFront(58, 7.99, redAudience), EPS);
        assertEquals(48.3, ShotTables.angleOffFront(23.58, 23.58, redAudience), 0.1);

        // The same spots mirrored to the FAR cell (across the center line) and to blue (180 deg spin).
        Pose redFar = Field.cellTarget(Alliance.RED, Cell.FAR);
        Pose blueFar = Field.cellTarget(Alliance.BLUE, Cell.FAR);
        double s = Field.SIZE;
        assertEquals(2225, ShotTables.staticShotRpm(58, s - 7.99, redFar), EPS);
        assertEquals(2083, ShotTables.staticShotRpm(23.58, s - 23.58, redFar), EPS);
        assertEquals(2083, ShotTables.staticShotRpm(s - 23.58, s - 23.58, blueFar), EPS);
        assertEquals(3787, ShotTables.staticShotRpm(47.17, s - 23.58, redFar), EPS);
        assertEquals(3787, ShotTables.staticShotRpm(s - 47.17, s - 23.58, blueFar), EPS);
    }

    @Test
    public void closerInFrontNeedsMoreRpmButNotFromBehind() {
        Pose redAudience = Field.cellTarget(Alliance.RED, Cell.AUDIENCE);
        // Head-on, 46 in. away the scale is 1, so it is HEAD_ON_RPM scaled back from its 46.26 in.
        double at46 = ShotTables.staticShotRpm(58, 54.25 - 46, redAudience);
        double at36 = ShotTables.staticShotRpm(58, 54.25 - 36, redAudience);
        double at55 = ShotTables.staticShotRpm(58, 54.25 - 52, redAudience);
        assertEquals(1.272, at36 / at46, 1e-6);
        assertTrue(at55 < at46);
        // Under the table's first row it stays at that row's scale.
        assertEquals(ShotTables.staticShotRpm(58, 54.25 - 32, redAudience),
                ShotTables.staticShotRpm(58, 54.25 - 25, redAudience), EPS);

        // Straight behind (past the BEHIND spot's angle) the RPM is BEHIND_RPM at any distance.
        assertEquals(2438, ShotTables.staticShotRpm(58, 54.25 + 30, redAudience), EPS);
        assertEquals(2438, ShotTables.staticShotRpm(58, 54.25 + 50, redAudience), EPS);
        // Halfway through the fade, half the scale is left.
        double angle = Math.toRadians((ShotTables.DISTANCE_FULL_DEG + ShotTables.DISTANCE_NONE_DEG) / 2);
        double fade = ShotTables.distanceScale(58 - 36 * Math.sin(angle), 54.25 - 36 * Math.cos(angle), redAudience);
        assertEquals(1 + (1.272 - 1) / 2, fade, 1e-6);
    }

    @Test
    public void aimShiftsTowardTheFarWallFromTheSide() {
        ShotTables.AIM_SHIFT_IN = 2;
        Pose cell = Field.cellTarget(Alliance.RED, Cell.AUDIENCE);
        // Head-on: no shift.
        assertEquals(58, ShotTables.shiftedAim(58, 7.99, cell).x(), EPS);
        // From straight beside it on the red side (-x): the full 2 in., toward +x.
        Pose beside = ShotTables.shiftedAim(20, 54.25, cell);
        assertEquals(60, beside.x(), EPS);
        assertEquals(54.25, beside.y(), EPS);
        // From the GARDEN corner: 2 x sin(48.3 deg), still toward +x; from the other side, toward -x.
        assertEquals(58 + 2 * Math.sin(Math.toRadians(ShotTables.angleOffFront(23.58, 23.58, cell))),
                ShotTables.shiftedAim(23.58, 23.58, cell).x(), EPS);
        assertTrue(ShotTables.shiftedAim(90, 30, cell).x() < 58);

        // A huge shift stops at the HIVE wall on the far side of the opening.
        ShotTables.AIM_SHIFT_IN = 50;
        assertEquals(58 + Field.CELL_OPENING_HALF_WIDTH, ShotTables.shiftedAim(20, 54.25, cell).x(), EPS);
        ShotTables.AIM_SHIFT_IN = 2;

        // The solver aims the turret at the shifted point.
        ShotSolution shot = new ShotSolver().solveShot(new Pose(20, 54.25, 0), new Velocity(0, 0, 0), cell);
        assertEquals(0, shot.turretDeg, EPS);
        assertEquals(40, shot.distance, EPS);
    }

    @Test
    public void aimPointNeverLeavesTheHive() {
        Pose[] cells = {
                Field.cellTarget(Alliance.RED, Cell.AUDIENCE), Field.cellTarget(Alliance.RED, Cell.FAR),
                Field.cellTarget(Alliance.BLUE, Cell.AUDIENCE), Field.cellTarget(Alliance.BLUE, Cell.FAR)};
        // Spots around each cell, as (dx, dy) from it: beside on both sides, corners, head-on, behind.
        double[][] spots = {{-38, 0}, {38, 0}, {-34, -30}, {34, -30}, {-34, 30}, {34, 30},
                {0, -46}, {0, 46}, {-5, 40}, {5, -40}};
        for (double shift : new double[] {50, -50, 1e6}) {
            ShotTables.AIM_SHIFT_IN = shift;
            for (Pose cell : cells) {
                for (double[] spot : spots) {
                    Pose aim = ShotTables.shiftedAim(cell.x() + spot[0], cell.y() + spot[1], cell);
                    assertTrue("shift " + shift + " from " + spot[0] + ", " + spot[1],
                            Math.abs(aim.x() - cell.x()) <= Field.CELL_OPENING_HALF_WIDTH + 1e-9);
                    assertEquals(cell.y(), aim.y(), 0);
                }
            }
        }
    }

    /** Robot-relative turret angle, in degrees, from (x, y) facing heading (radians) toward (aimX, aimY). */
    private static double turretDeg(double x, double y, double heading, double aimX, double aimY) {
        return Angles.wrapDegrees(Math.toDegrees(Math.atan2(aimY - y, aimX - x) - heading));
    }

    @Test
    public void predictOffIsTheSameAsBefore() {
        ShotSolver solver = new ShotSolver();
        Pose robot = new Pose(30, 40, Math.toRadians(30));
        Velocity velocity = new Velocity(20, -10, 1);
        Pose target = new Pose(110, 100, 0);

        // Static: straight at the target, from where the robot is now. 80 in. along x, 60 along y.
        ShotSolution still = solver.solveShot(robot, velocity, target);
        assertEquals(100, still.distance, EPS);
        assertEquals(100, still.targetDistance, EPS);
        assertEquals(Math.toDegrees(Math.atan2(60, 80)) - 30, still.turretDeg, EPS);
        assertEquals(ShotTables.staticShotRpm(30, 40, target), still.rpm, EPS);
        assertTrue(still.valid);

        // The coax hood aims the same way.
        ShotSolution coax = solver.solveCoaxShot(robot, velocity, target, 2300);
        assertEquals(100, coax.distance, EPS);
        assertEquals(still.turretDeg, coax.turretDeg, EPS);

        // LEAD: aim at target - velocity * flight time = (110 - 10, 100 + 5), still from right here.
        ShotSolver.LEAD = true;
        ShotSolution led = solver.solveShot(robot, velocity, target);
        assertEquals(Math.hypot(70, 65), led.distance, EPS);
        assertEquals(100, led.targetDistance, EPS);
        assertEquals(Math.toDegrees(Math.atan2(65, 70)) - 30, led.turretDeg, EPS);

        // The acceleration estimate changes underneath (here a hard brake), but with PREDICT
        // off nothing uses it: every shot is still from right here, with the velocity as read.
        for (int i = 0; i < 20; i++) {
            double vx = 60 - 3 * i;
            ShotSolution s = solver.solve(robot, new Velocity(vx, 0, 0), target, false, 0, i * DT);
            double aimX = 110 - vx * FLIGHT_TIME;
            assertEquals("loop " + i, Math.hypot(aimX - 30, 60), s.distance, EPS);
            assertEquals("loop " + i, turretDeg(30, 40, Math.toRadians(30), aimX, 100), s.turretDeg, EPS);
        }
    }

    @Test
    public void brakingToAStopLeadsNoneInsteadOfBackward() {
        ShotSolver.LEAD = true;
        ShotSolver.PREDICT = true;
        ShotSolver solver = new ShotSolver();
        // Driving straight at the target, so the lead is how much nearer the aim point is than the target.
        Pose robot = new Pose(20, 0, 0);
        Pose target = new Pose(100, 0, 0);

        // Measured: 60 in/s for 10 loops, then 4 in/s less every 20 ms (200 in/s²) down to 0.
        // The robot is really 3 loops (0.06 s, VELOCITY_LATENCY_SEC) ahead of that.
        boolean sawTheStop = false;
        for (int i = 0; i < 40; i++) {
            double measured = Math.max(0, Math.min(60, 60 - 4 * (i - 10)));
            double real = Math.max(0, Math.min(60, 60 - 4 * (i + 3 - 10)));
            ShotSolution s = solver.solve(robot, new Velocity(measured, 0, 0), target, false, 0, i * DT);
            double leadInches = s.targetDistance - s.distance;
            String when = "loop " + i + ", measured " + measured + " in/s, really " + real + ": lead " + leadInches;

            // Never backward, and never more than the plain lead from the measured velocity.
            assertTrue(when, leadInches >= -EPS);
            assertTrue(when, leadInches <= measured * FLIGHT_TIME + EPS);
            // Once the robot has really stopped, no lead, even while the reading still says it's moving.
            if (real == 0) assertEquals(when, 0, leadInches, EPS);
            if (real == 0 && measured > 0) sawTheStop = true;
        }
        assertTrue("the reading lags the stop", sawTheStop);

        // Loop 23: really stopped, measured still 8 in/s. Without PREDICT that is a 4 in. lead.
        ShotSolver.PREDICT = false;
        ShotSolution stale = new ShotSolver().solve(robot, new Velocity(8, 0, 0), target, false, 0, 0);
        assertEquals(8 * FLIGHT_TIME, stale.targetDistance - stale.distance, EPS);
    }

    @Test
    public void turretUsesTheTurretHorizonAndDistanceTheRelease() {
        ShotSolver.PREDICT = true;
        // Constant velocity first, to keep the numbers simple.
        ShotSolver.USE_ACCEL = false;
        ShotSolver solver = new ShotSolver();
        Pose robot = new Pose(20, 30, 0);
        Velocity velocity = new Velocity(40, 0, 2);
        Pose target = new Pose(100, 90, 0);
        ShotSolution s = solver.solve(robot, velocity, target, false, 0, 0);

        // The ball leaves 0.03 s from now, 1.2 in. further along x.
        assertEquals(Math.hypot(100 - 21.2, 60), s.distance, EPS);
        assertEquals(s.distance, s.targetDistance, EPS);
        // The turret gets there 0.08 s from now: 3.2 in. along x, turned 0.16 rad.
        assertEquals(turretDeg(23.2, 30, 0.16, 100, 90), s.turretDeg, EPS);
        // So not both from one pose.
        assertTrue(Math.abs(s.distance - Math.hypot(100 - 23.2, 60)) > 1);
        assertTrue(Math.abs(s.turretDeg - turretDeg(21.2, 30, 0.06, 100, 90)) > 1);
    }

    @Test
    public void accelerationLearnedWithPredictOffIsUsedWhenItIsTurnedOn() {
        ShotSolver.LEAD = true;
        ShotSolver solver = new ShotSolver();
        Pose robot = new Pose(20, 30, 0);
        Pose target = new Pose(100, 90, 0);

        // Speeding up along y at 100 in/s² (2 in/s more every 20 ms) for a second, PREDICT off.
        for (int i = 0; i <= 50; i++) {
            solver.solve(robot, new Velocity(0, 10 + 2 * i, 0), target, false, 0, i * DT);
        }

        // Turned on: measured 112 in/s, really 112 + 100 * 0.06 = 118 now.
        ShotSolver.PREDICT = true;
        ShotSolution s = solver.solve(robot, new Velocity(0, 112, 0), target, false, 0, 51 * DT);
        // At release (0.03 s): y = 30 + 118 * 0.03 + 50 * 0.03², going 118 + 100 * 0.03 = 121 in/s.
        double releaseY = 30 + 118 * 0.03 + 50 * 0.03 * 0.03;
        // LEAD with the velocity at release.
        double aimY = 90 - 121 * FLIGHT_TIME;
        // The turret: y = 30 + 118 * 0.08 + 50 * 0.08².
        double turretY = 30 + 118 * 0.08 + 50 * 0.08 * 0.08;
        // The smoothed estimate is a hair under 100 after 51 loops.
        double tolerance = 1e-5;
        assertEquals(Math.hypot(80, aimY - releaseY), s.distance, tolerance);
        assertEquals(Math.hypot(80, 90 - releaseY), s.targetDistance, tolerance);
        assertEquals(turretDeg(20, turretY, 0, 100, aimY), s.turretDeg, tolerance);
    }
}
