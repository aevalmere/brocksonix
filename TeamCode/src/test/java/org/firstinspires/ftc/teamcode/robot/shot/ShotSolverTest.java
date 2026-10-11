package org.firstinspires.ftc.teamcode.robot.shot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

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
    private double[][] shotRpm, shotFlightTime, shotFeedPower;

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
        shotRpm = ShotTables.SHOT_RPM;
        shotFlightTime = ShotTables.SHOT_FLIGHT_TIME;
        shotFeedPower = ShotTables.SHOT_FEED_POWER;

        // Our own tables, so these numbers don't change when ShotTables is tuned.
        ShotTables.SHOT_RPM = new double[][]{{20, 2000}, {120, 4000}};
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
        ShotTables.SHOT_RPM = shotRpm;
        ShotTables.SHOT_FLIGHT_TIME = shotFlightTime;
        ShotTables.SHOT_FEED_POWER = shotFeedPower;
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
        assertEquals(3600, still.rpm, EPS);
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
