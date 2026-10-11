package org.firstinspires.ftc.teamcode.robot.shot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * MotionPredictor: constant velocity predicts a straight line, a braking robot stops and
 * never backs up, the lag correction never flips the sign, and the acceleration is the
 * smoothed one-loop difference, capped, skipping samples too close together, bad
 * readings and old samples.
 */
public class MotionPredictorTest {
    private static final double EPS = 1e-9;
    // Accelerations are a difference divided by a time step, so they carry a little rounding.
    private static final double ACCEL_EPS = 1e-6;
    private static final double DT = 0.02;

    private boolean useAccel;
    private double accelFilter, maxAccel, maxAngularAccel, velocityLatency;

    @Before
    public void remember() {
        useAccel = ShotSolver.USE_ACCEL;
        accelFilter = ShotSolver.ACCEL_FILTER;
        maxAccel = ShotSolver.MAX_ACCEL;
        maxAngularAccel = ShotSolver.MAX_ANGULAR_ACCEL;
        velocityLatency = ShotSolver.VELOCITY_LATENCY_SEC;
        // The defaults, in case they were tuned.
        ShotSolver.USE_ACCEL = true;
        ShotSolver.ACCEL_FILTER = 0.7;
        ShotSolver.MAX_ACCEL = 200;
        ShotSolver.MAX_ANGULAR_ACCEL = 20;
        ShotSolver.VELOCITY_LATENCY_SEC = 0.06;
    }

    @After
    public void putBack() {
        ShotSolver.USE_ACCEL = useAccel;
        ShotSolver.ACCEL_FILTER = accelFilter;
        ShotSolver.MAX_ACCEL = maxAccel;
        ShotSolver.MAX_ANGULAR_ACCEL = maxAngularAccel;
        ShotSolver.VELOCITY_LATENCY_SEC = velocityLatency;
    }

    /**
     * Two samples DT apart, with ACCEL_FILTER 0, give the acceleration (to, at DT) - (from, at 0)
     * divided by DT: here -200 in/s² along x, +200 along y and -10 rad/s² turning.
     */
    private static MotionPredictor braking() {
        ShotSolver.ACCEL_FILTER = 0;
        MotionPredictor motion = new MotionPredictor();
        motion.update(new Velocity(14, -14, 1.2), 0);
        motion.update(new Velocity(10, -10, 1.0), DT);
        assertEquals(-200, motion.ax(), ACCEL_EPS);
        assertEquals(200, motion.ay(), ACCEL_EPS);
        assertEquals(-10, motion.alpha(), ACCEL_EPS);
        return motion;
    }

    @Test
    public void constantVelocityIsAStraightLine() {
        MotionPredictor motion = new MotionPredictor();
        Velocity velocity = new Velocity(30, -20, 0.5);
        for (int i = 0; i < 10; i++) motion.update(velocity, i * DT);
        assertEquals(0, motion.ax(), EPS);
        assertEquals(0, motion.ay(), EPS);
        assertEquals(0, motion.alpha(), EPS);

        // Nothing is changing, so the lag correction changes nothing.
        Velocity now = motion.compensated(velocity);
        assertEquals(30, now.vx, EPS);
        assertEquals(-20, now.vy, EPS);
        assertEquals(0.5, now.omega, EPS);

        // Pose + velocity * time.
        MotionPredictor.Prediction later = motion.predict(new Pose(10, 20, 1.0), velocity, 0.1);
        assertEquals(13, later.pose.x(), EPS);
        assertEquals(18, later.pose.y(), EPS);
        assertEquals(1.05, later.pose.heading(), EPS);
        assertEquals(30, later.velocity.vx, EPS);
        assertEquals(-20, later.velocity.vy, EPS);
        assertEquals(0.5, later.velocity.omega, EPS);

        // A negative or NaN time (typed into Panels) counts as 0: right where it is.
        double[] badTimes = {-0.1, Double.NaN};
        for (double seconds : badTimes) {
            MotionPredictor.Prediction here = motion.predict(new Pose(10, 20, 1.0), velocity, seconds);
            assertEquals(10, here.pose.x(), EPS);
            assertEquals(20, here.pose.y(), EPS);
            assertEquals(1.0, here.pose.heading(), EPS);
        }
    }

    @Test
    public void brakingStopsAtZeroAndNeverBacksUp() {
        MotionPredictor motion = braking();
        // No lag correction here, so the numbers are just the braking (the next test has it).
        ShotSolver.VELOCITY_LATENCY_SEC = 0;
        Pose start = new Pose(50, 60, 0.5);
        Velocity measured = new Velocity(10, -10, 1.0);

        // 0.03 s in it's still slowing: 10 - 200 * 0.03 = 4 in/s, and 10 * 0.03 - 100 * 0.03² = 0.21 in. along.
        MotionPredictor.Prediction soon = motion.predict(start, measured, 0.03);
        assertEquals(50.21, soon.pose.x(), EPS);
        assertEquals(59.79, soon.pose.y(), EPS);
        assertEquals(4, soon.velocity.vx, ACCEL_EPS);
        assertEquals(-4, soon.velocity.vy, ACCEL_EPS);

        // x and y stop after 0.05 s, 0.25 in. along. Turning stops after 0.1 s, 0.05 rad on.
        double lastX = start.x();
        for (double seconds = 0; seconds <= 0.5; seconds += 0.01) {
            MotionPredictor.Prediction p = motion.predict(start, measured, seconds);
            String when = seconds + " s";
            assertTrue(when, p.pose.x() >= lastX - EPS && p.pose.x() <= 50.25 + EPS);
            assertTrue(when, p.pose.y() >= 59.75 - EPS && p.pose.y() <= 60 + EPS);
            assertTrue(when, p.pose.heading() >= 0.5 - EPS && p.pose.heading() <= 0.55 + EPS);
            assertTrue(when, p.velocity.vx >= 0 && p.velocity.vy <= 0 && p.velocity.omega >= 0);
            lastX = p.pose.x();
        }
        MotionPredictor.Prediction stopped = motion.predict(start, measured, 0.5);
        assertEquals(50.25, stopped.pose.x(), EPS);
        assertEquals(59.75, stopped.pose.y(), EPS);
        assertEquals(0.55, stopped.pose.heading(), EPS);
        assertEquals(0, stopped.velocity.vx, 0);
        assertEquals(0, stopped.velocity.vy, 0);
        assertEquals(0, stopped.velocity.omega, 0);
    }

    @Test
    public void lagCorrectionThatWouldFlipTheSignGivesZero() {
        MotionPredictor motion = braking();
        // -200 in/s² for 0.06 s takes 12 in/s off: 10 would become -2, so 0.
        // -10 rad/s² for 0.06 s takes 0.6 rad/s off: 0.5 would become -0.1, so 0.
        Velocity measured = new Velocity(10, -10, 0.5);
        Velocity now = motion.compensated(measured);
        assertEquals(0, now.vx, 0);
        assertEquals(0, now.vy, 0);
        assertEquals(0, now.omega, 0);

        // Farther from a stop, it is just velocity + acceleration * lag.
        Velocity fast = motion.compensated(new Velocity(30, -30, 2));
        assertEquals(18, fast.vx, ACCEL_EPS);
        assertEquals(-18, fast.vy, ACCEL_EPS);
        assertEquals(1.4, fast.omega, ACCEL_EPS);

        // And predict() keeps it where it is: stopped, not backing up.
        MotionPredictor.Prediction later = motion.predict(new Pose(50, 60, 0.5), measured, 0.1);
        assertEquals(50, later.pose.x(), EPS);
        assertEquals(60, later.pose.y(), EPS);
        assertEquals(0.5, later.pose.heading(), EPS);
        assertEquals(0, later.velocity.vx, 0);
        assertEquals(0, later.velocity.vy, 0);
        assertEquals(0, later.velocity.omega, 0);
    }

    @Test
    public void accelerationIsTheSmoothedOneLoopDifference() {
        ShotSolver.ACCEL_FILTER = 0.7;
        MotionPredictor motion = new MotionPredictor();
        motion.update(new Velocity(10, 20, 1), 0);
        // Nothing to compare with yet.
        assertEquals(0, motion.ax(), EPS);

        // The differences are 100 in/s², -50 in/s² and 5 rad/s². Smoothed from 0, 0.3 of that.
        motion.update(new Velocity(12, 19, 1.1), DT);
        assertEquals(30, motion.ax(), ACCEL_EPS);
        assertEquals(-15, motion.ay(), ACCEL_EPS);
        assertEquals(1.5, motion.alpha(), ACCEL_EPS);

        // Then 50, 0 and 10: 0.7 * the old estimate + 0.3 * the new difference.
        motion.update(new Velocity(13, 19, 1.3), 2 * DT);
        assertEquals(0.7 * 30 + 0.3 * 50, motion.ax(), ACCEL_EPS);
        assertEquals(0.7 * -15, motion.ay(), ACCEL_EPS);
        assertEquals(0.7 * 1.5 + 0.3 * 10, motion.alpha(), ACCEL_EPS);
    }

    @Test
    public void samplesTooCloseTogetherAreSkipped() {
        ShotSolver.ACCEL_FILTER = 0;
        MotionPredictor motion = new MotionPredictor();
        motion.update(new Velocity(10, 10, 1), 0);
        motion.update(new Velocity(12, 10, 1), DT);
        assertEquals(100, motion.ax(), ACCEL_EPS);

        // 5 ms later: too soon, skipped.
        motion.update(new Velocity(50, 10, 1), DT + 0.005);
        assertEquals(100, motion.ax(), ACCEL_EPS);

        // The next one is compared with the last sample kept (12 at DT), not the skipped one.
        motion.update(new Velocity(13, 10, 1), 2 * DT);
        assertEquals(50, motion.ax(), ACCEL_EPS);
    }

    @Test
    public void accelerationIsCapped() {
        ShotSolver.ACCEL_FILTER = 0;
        MotionPredictor motion = new MotionPredictor();
        motion.update(new Velocity(10, -10, 0.5), 0);
        // 2500 in/s² and 125 rad/s²: far more than the robot can do.
        motion.update(new Velocity(60, -60, 3), DT);
        assertEquals(200, motion.ax(), EPS);
        assertEquals(-200, motion.ay(), EPS);
        assertEquals(20, motion.alpha(), EPS);

        // A new cap from Panels applies on the next sample.
        ShotSolver.MAX_ACCEL = 50;
        ShotSolver.MAX_ANGULAR_ACCEL = 5;
        motion.update(new Velocity(110, -110, 5.5), 2 * DT);
        assertEquals(50, motion.ax(), EPS);
        assertEquals(-50, motion.ay(), EPS);
        assertEquals(5, motion.alpha(), EPS);
    }

    @Test
    public void badReadingsDontPoisonTheEstimate() {
        ShotSolver.ACCEL_FILTER = 0;
        MotionPredictor motion = new MotionPredictor();
        motion.update(new Velocity(10, 10, 1), 0);
        motion.update(new Velocity(12, 10, 1), DT);
        assertEquals(100, motion.ax(), ACCEL_EPS);

        // An odometry glitch: skipped.
        motion.update(new Velocity(Double.NaN, 10, 1), 2 * DT);
        motion.update(new Velocity(12, Double.POSITIVE_INFINITY, 1), 2 * DT);
        motion.update(new Velocity(12, 10, Double.NaN), 2 * DT);
        motion.update(new Velocity(12, 10, 1), Double.NaN);
        assertEquals(100, motion.ax(), ACCEL_EPS);
        assertEquals(0, motion.ay(), EPS);
        assertEquals(0, motion.alpha(), EPS);

        // The next good sample is compared with the last good one.
        motion.update(new Velocity(13, 10, 1), 2 * DT);
        assertEquals(50, motion.ax(), ACCEL_EPS);

        // A NaN typed into Panels doesn't stay in the estimate either.
        ShotSolver.ACCEL_FILTER = Double.NaN;
        motion.update(new Velocity(14, 10, 1), 3 * DT);
        assertFalse(Double.isNaN(motion.ax()));
        ShotSolver.ACCEL_FILTER = 0;
        ShotSolver.MAX_ACCEL = Double.NaN;
        motion.update(new Velocity(15, 10, 1), 4 * DT);
        assertFalse(Double.isNaN(motion.ax()));
        ShotSolver.MAX_ACCEL = 200;
        motion.update(new Velocity(16, 10, 1), 5 * DT);
        assertEquals(50, motion.ax(), ACCEL_EPS);
    }

    @Test
    public void aLongGapStartsAgain() {
        ShotSolver.ACCEL_FILTER = 0;
        MotionPredictor motion = new MotionPredictor();
        motion.update(new Velocity(10, 0, 0), 0);
        motion.update(new Velocity(12, 0, 0), DT);
        assertEquals(100, motion.ax(), ACCEL_EPS);

        // Half a second with no shot solved (passing, say): too old to compare with.
        motion.update(new Velocity(40, 0, 0), DT + 0.5);
        assertEquals(0, motion.ax(), EPS);
        motion.update(new Velocity(41, 0, 0), 2 * DT + 0.5);
        assertEquals(50, motion.ax(), ACCEL_EPS);
    }

    @Test
    public void aRobotThatJustStoppedDoesntBackUp() {
        // Measured: braking at 200 in/s² (4 in/s less every 20 ms) down to a stop, then
        // standing still. The smoothed estimate is still braking when the reading gets to 0.
        MotionPredictor motion = new MotionPredictor();
        Pose start = new Pose(50, 60, 0);
        double t = 0;
        for (int i = 0; i <= 15; i++) {
            Velocity measured = new Velocity(Math.max(0, 40 - 4 * i), 0, 0);
            motion.update(measured, t);
            MotionPredictor.Prediction p = motion.predict(start, measured, 0.1);
            String when = "loop " + i + ", measured " + measured.vx;
            assertTrue(when + ": " + p.pose.x(), p.pose.x() >= start.x() - EPS);
            assertTrue(when + ": " + p.velocity.vx, p.velocity.vx >= 0);
            assertTrue(when, motion.compensated(measured).vx >= 0);
            t += DT;
        }
        // Reaching 0 ended the brake: standing still predicts standing still.
        assertEquals(0, motion.ax(), EPS);
        MotionPredictor.Prediction p = motion.predict(start, new Velocity(0, 0, 0), 0.1);
        assertEquals(50, p.pose.x(), EPS);
        assertEquals(0, p.velocity.vx, 0);
    }

    @Test
    public void withUseAccelOffItIsConstantVelocity() {
        MotionPredictor motion = braking();
        ShotSolver.USE_ACCEL = false;
        // Still estimated, so turning USE_ACCEL back on works at once.
        motion.update(new Velocity(6, -6, 0.8), 2 * DT);
        assertEquals(-200, motion.ax(), ACCEL_EPS);

        Velocity measured = new Velocity(10, -10, 1.0);
        Velocity now = motion.compensated(measured);
        assertEquals(10, now.vx, EPS);
        assertEquals(-10, now.vy, EPS);
        assertEquals(1.0, now.omega, EPS);
        MotionPredictor.Prediction later = motion.predict(new Pose(0, 0, 0), measured, 0.1);
        assertEquals(1, later.pose.x(), EPS);
        assertEquals(-1, later.pose.y(), EPS);
        assertEquals(0.1, later.pose.heading(), EPS);
        assertEquals(10, later.velocity.vx, EPS);
    }
}
