package org.firstinspires.ftc.teamcode.robot.shot;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.util.PIDF;

/**
 * Predicts where the robot will be a moment from now and how fast it will be going,
 * for ShotSolver's PREDICT layer. No hardware: the caller passes the time, so the
 * desktop tests can run it. Its settings are on ShotSolver's Panels page, next to the
 * other shooting-while-moving knobs.
 *
 * - Acceleration: update() takes the change in velocity since the last loop divided by
 *   the time between them, capped at MAX_ACCEL / MAX_ANGULAR_ACCEL, and smoothed
 *   (ACCEL_FILTER). Only one loop back: an average over many loops still shows a brake
 *   that has already ended, and the shot over-leads.
 * - Lag: Pedro's velocity is a smoothed difference, so it is about VELOCITY_LATENCY_SEC
 *   old. compensated() moves it forward that much with the acceleration. A brake slows
 *   the robot to a stop, it doesn't turn it around, so if that would flip the sign the
 *   answer is 0.
 * - Stopping: predict() runs each axis (x, y and heading) forward with a constant
 *   acceleration, but a braking axis stops at 0 and stays there. Without this, a robot
 *   that brakes and fires would be predicted to back up, and the shot would lead the
 *   wrong way.
 *
 * With USE_ACCEL off the acceleration counts as 0: constant velocity and no lag
 * correction. The estimate still updates, so turning it back on works at once.
 *
 * The idea comes from GNCE's DECODE (2025-26) robot Ditto, https://github.com/GNCE/DECODEv2-Ditto:
 * ShotPlanner's plan, predictWithStop and compensateVel in
 * https://github.com/GNCE/DECODEv2-Ditto/blob/c78b5268930d55c6d5a59c23922d9a8baf9f96e6/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/core/util/ShotPlanner.java
 * Two things are ours: an axis that compensated() finds stopped stays stopped in
 * predict(), and an axis's acceleration starts again from 0 when its velocity reaches
 * or crosses 0. Without them, a robot that just stopped would look like it's backing
 * up for a few loops, while the smoothed braking acceleration fades.
 */
public class MotionPredictor {
    /** A predicted pose, and the velocity at that moment. */
    public static class Prediction {
        public final Pose pose;
        public final Velocity velocity;

        Prediction(Pose pose, Velocity velocity) {
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    // Samples closer together than this give a noisy difference, so they are skipped.
    private static final double MIN_SAMPLE_SPACING_SEC = 0.008;
    // A longer gap (no shot solved for a while, like while passing) is too old to compare with: start again.
    private static final double MAX_SAMPLE_GAP_SEC = 0.25;

    private boolean havePrevious = false;
    private double previousTime = 0;
    private double previousVx = 0;
    private double previousVy = 0;
    private double previousOmega = 0;

    // The acceleration estimate: in/s² along field x and y, rad/s² for turning.
    private double ax = 0;
    private double ay = 0;
    private double alpha = 0;

    /** Call once per loop with the measured field velocity and the time in seconds. */
    public void update(Velocity measured, double nowSeconds) {
        // A glitch would put a huge spike in the estimate, so skip it.
        if (!Double.isFinite(measured.vx) || !Double.isFinite(measured.vy)
                || !Double.isFinite(measured.omega) || !Double.isFinite(nowSeconds)) return;

        double dt = nowSeconds - previousTime;
        if (!havePrevious || dt < 0 || dt > MAX_SAMPLE_GAP_SEC) {
            // Nothing recent to compare with: start again.
            ax = 0;
            ay = 0;
            alpha = 0;
        } else if (dt < MIN_SAMPLE_SPACING_SEC) {
            // Too soon. Keep the previous sample, so the next difference spans enough time.
            return;
        } else {
            double maxAccel = Math.abs(ShotSolver.MAX_ACCEL);
            double maxAlpha = Math.abs(ShotSolver.MAX_ANGULAR_ACCEL);
            ax = nextAccel(ax, previousVx, measured.vx, dt, maxAccel);
            ay = nextAccel(ay, previousVy, measured.vy, dt, maxAccel);
            alpha = nextAccel(alpha, previousOmega, measured.omega, dt, maxAlpha);
        }
        havePrevious = true;
        previousTime = nowSeconds;
        previousVx = measured.vx;
        previousVy = measured.vy;
        previousOmega = measured.omega;
    }

    /** Acceleration estimate along field x, in/s². This is the estimate even with USE_ACCEL off. */
    public double ax() {
        return ax;
    }

    /** Acceleration estimate along field y, in/s². */
    public double ay() {
        return ay;
    }

    /** Turning acceleration estimate, rad/s². */
    public double alpha() {
        return alpha;
    }

    /** The measured velocity moved forward VELOCITY_LATENCY_SEC, so it is what the robot does now. */
    public Velocity compensated(Velocity measured) {
        double lag = notNegative(ShotSolver.VELOCITY_LATENCY_SEC);
        return new Velocity(
                compensate(measured.vx, used(ax), lag),
                compensate(measured.vy, used(ay), lag),
                compensate(measured.omega, used(alpha), lag));
    }

    /**
     * Where the robot will be `seconds` from now, and its velocity then. Pass the measured
     * velocity: it is compensated here first (see compensated()), and an axis that has
     * already stopped stays where it is.
     */
    public Prediction predict(Pose pose, Velocity measured, double seconds) {
        double t = notNegative(seconds);
        double lag = notNegative(ShotSolver.VELOCITY_LATENCY_SEC);
        double[] x = axis(pose.x(), measured.vx, used(ax), lag, t);
        double[] y = axis(pose.y(), measured.vy, used(ay), lag, t);
        double[] heading = axis(pose.heading(), measured.omega, used(alpha), lag, t);
        return new Prediction(new Pose(x[0], y[0], heading[0]), new Velocity(x[1], y[1], heading[1]));
    }

    /** One axis: the change since the last sample, per second, capped and smoothed. */
    private static double nextAccel(double accel, double previous, double now, double dt, double max) {
        // It reached 0 or went through it, so any brake is over. Keeping the old braking
        // acceleration would make a robot that just stopped look like it's backing up.
        if ((previous > 0 && now <= 0) || (previous < 0 && now >= 0)) return 0;
        double raw = Math.max(-max, Math.min(max, (now - previous) / dt));
        double smoothed = PIDF.smooth(accel, raw, ShotSolver.ACCEL_FILTER);
        // A NaN typed into Panels would stay in the estimate for good, so forget it.
        return Double.isNaN(smoothed) ? 0 : smoothed;
    }

    /**
     * One axis after `seconds`, from a velocity measured `lag` seconds ago, with a constant
     * acceleration. Returns {position, velocity}. A braking axis stops at 0 and stays there.
     */
    private static double[] axis(double position, double measured, double accel, double lag, double seconds) {
        double v = compensate(measured, accel, lag);
        // compensate() found it has stopped by now: it stays put. Running the braking
        // acceleration on from 0 would send it backward.
        if (v == 0 && measured != 0) return new double[]{position, 0};

        double t = seconds;
        // Velocity and acceleration in opposite directions: braking. Only move until it stops.
        if (v * accel < 0) t = Math.min(t, -v / accel);
        double end = position + v * t + 0.5 * accel * t * t;
        double endVelocity = t < seconds ? 0 : v + accel * seconds;
        return new double[]{end, endVelocity};
    }

    /** The velocity `lag` seconds after it was measured. A sign flip gives 0: a brake stops, it doesn't reverse. */
    private static double compensate(double measured, double accel, double lag) {
        double v = measured + accel * lag;
        boolean flipped = (measured > 0 && v < 0) || (measured < 0 && v > 0);
        return flipped ? 0 : v;
    }

    /** The acceleration to predict with: the estimate, or 0 with USE_ACCEL off. */
    private static double used(double estimate) {
        return ShotSolver.USE_ACCEL ? estimate : 0;
    }

    /** Seconds can't be negative. A negative or NaN value (typed into Panels) counts as 0. */
    private static double notNegative(double seconds) {
        return seconds > 0 ? seconds : 0;
    }
}
