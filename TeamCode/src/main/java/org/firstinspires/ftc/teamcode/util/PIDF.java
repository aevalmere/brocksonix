package org.firstinspires.ftc.teamcode.util;

/**
 * Our own PIDF controller. It works out the power that moves something (like the
 * turret) to a target and keeps it there. There is no hardware in here, so the
 * desktop tests can run it: the caller reads the sensor, passes the seconds since
 * the last call, and sends the result to the hardware.
 *
 * Call update() once per loop. Call the setters every loop before it, so Panels
 * edits apply at once and the caller can switch gain sets (like the turret's FAR
 * and CLOSE). The output is the sum of these parts:
 *
 *   P = kP * error. The error is the target minus where we are. Far away it pushes
 *       hard, close by it pushes gently.
 *   I = kI * integral. The integral adds up the error over time (error times
 *       seconds). When P is too weak to finish the last bit (friction), the integral
 *       keeps growing until the push is big enough. It is kept in check:
 *       - it only adds up while the error is under the integral zone, so it doesn't
 *         build up during a big move (and starts from 0 again outside the zone);
 *       - it is capped, so the I part never pushes more than maxIntegralOutput;
 *       - it stops adding up while the output is already at its limit the same way
 *         as the error ("anti-windup"): more push can't help then, it would only
 *         overshoot later;
 *       - it starts again from 0 when the error changes sign (we crossed the
 *         target) and when the target jumps;
 *       - the caller can pause it (setIntegralPaused) or empty it (resetIntegral)
 *         while something else sets the output.
 *   D = kD * (target speed - measured speed). It brakes when we move toward the
 *       target faster than the target moves, so we don't overshoot. It looks at how
 *       fast we really move, not at how fast the error changes, so a target that
 *       jumps or an offset (trim) change doesn't kick it. Sensor noise makes the
 *       measured speed jumpy, so it can be smoothed (dFilter).
 *   F = kF * target speed. Feedforward: when the target moves (the robot turns
 *       under the turret), push along with it before an error builds up. The target
 *       speed is smoothed (targetRateFilter) and capped (maxTargetRate). A target
 *       that moves more than targetJump in one loop didn't move, it jumped (a new
 *       target was picked): its speed counts as 0, or that loop would get a huge push.
 *
 * For angles that wrap around, like the turret, use new PIDF(true): every difference
 * then takes the shortest way around, so 179 to -179 degrees is 2 degrees, not 358.
 *
 * The very first update() has nothing to compare with: no speeds and no integral yet.
 * The output is not clipped here; the caller clips it (setOutputLimit only tells the
 * integral where the limit is).
 */
public class PIDF {
    // A long loop (a hiccup) would add a big chunk to the integral at once, so the
    // integral counts at most this many seconds per loop. Speeds use the real time:
    // they are a change divided by the time it took.
    private static final double MAX_INTEGRAL_DT = 0.1;
    // Guards against dividing by 0 if the caller passes 0 seconds.
    private static final double MIN_DT = 1e-3;
    // A filter of 1 would never change, so smoothing stops just short of it.
    private static final double MAX_FILTER = 0.99;

    private final boolean wrapDegrees;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;
    private double integralZone = Double.POSITIVE_INFINITY;
    private double maxIntegralOutput = Double.POSITIVE_INFINITY;
    private double dFilter = 0;
    private double targetRateFilter = 0;
    private double targetJump = Double.POSITIVE_INFINITY;
    private double maxTargetRate = Double.POSITIVE_INFINITY;
    private double maxMeasuredRate = Double.POSITIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;
    private boolean integralPaused = false;

    private boolean firstUpdate = true;
    private double lastTarget = 0;
    private double lastMeasured = 0;
    private double lastError = 0;
    private double integral = 0;
    private double targetRate = 0;
    private double measuredRate = 0;

    private double lastP = 0;
    private double lastI = 0;
    private double lastD = 0;
    private double lastF = 0;

    /**
     * wrapDegrees: true when the target and the measured value are angles in degrees
     * that wrap around (see the class comment). False for anything else.
     */
    public PIDF(boolean wrapDegrees) {
        this.wrapDegrees = wrapDegrees;
    }

    /**
     * kP: output per unit of error. kI: output per unit-second of error.
     * kD: output per unit/second of speed difference. kF: output per unit/second of target speed.
     */
    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** The integral only adds up while the error is under zone, and the I part never pushes more than maxIntegralOutput. */
    public void setIntegralLimits(double zone, double maxIntegralOutput) {
        this.integralZone = zone;
        this.maxIntegralOutput = Math.max(maxIntegralOutput, 0);
    }

    /** Smoothing for the measured speed (D) and the target speed (D and F): 0 = none, closer to 1 = smoother but slower. */
    public void setFilters(double dFilter, double targetRateFilter) {
        this.dFilter = dFilter;
        this.targetRateFilter = targetRateFilter;
    }

    /** A target step bigger than jump in one loop is a jump (speed 0 that loop). The target speed is capped at maxRate. */
    public void setTargetRateLimits(double jump, double maxRate) {
        this.targetJump = jump;
        this.maxTargetRate = Math.max(maxRate, 0);
    }

    /** A measured change faster than maxRate (units per second) is a bad reading: that loop keeps the last speed. */
    public void setMaxMeasuredRate(double maxRate) {
        this.maxMeasuredRate = maxRate;
    }

    /** The most the caller will send. The integral stops growing while the output is at it (anti-windup). */
    public void setOutputLimit(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    /**
     * While paused, the integral keeps its value instead of adding up. It still starts
     * again from 0 when we cross the target, the target jumps, or the error leaves the
     * zone. For when something else sets the output for a moment.
     */
    public void setIntegralPaused(boolean paused) {
        this.integralPaused = paused;
    }

    /** Same as the full update() with no offset and no extra. */
    public double update(double target, double measured, double dt) {
        return update(target, 0, measured, 0, dt);
    }

    /**
     * Works out this loop's output. Call it once per loop.
     *
     * @param target   where we want to be
     * @param offset   a small nudge added to the target (the turret's trim). It changes the error,
     *                 but changing it doesn't count as the target moving, so it doesn't kick D or F.
     * @param measured where we are, from the sensor
     * @param extra    any other push the caller adds (the turret's KS push). It is added to the
     *                 output, and counts when checking whether the output is at its limit.
     * @param dt       seconds since the last update()
     * @return P + I + D + F + extra, not clipped
     */
    public double update(double target, double offset, double measured, double extra, double dt) {
        if (Double.isNaN(dt) || dt < MIN_DT) dt = MIN_DT;
        boolean first = firstUpdate;
        firstUpdate = false;
        double error = difference(target + offset, measured);

        if (first) {
            // Nothing to compare with yet: no speeds, and the integral starts at 0.
            targetRate = 0;
            measuredRate = 0;
            integral = 0;
        } else {
            // Target speed, for D and F. The offset is left out on purpose.
            double targetStep = difference(target, lastTarget);
            if (Double.isNaN(targetStep) || Math.abs(targetStep) > targetJump) {
                // A new target: speed 0, and the integral was for the old target.
                // (A NaN step means last loop's target was NaN; treat it the same.)
                targetRate = 0;
                integral = 0;
            } else {
                double rate = clip(targetStep / dt, maxTargetRate);
                targetRate = smooth(targetRate, rate, targetRateFilter);
            }

            // Measured speed, for D. Too fast to be real is a bad reading: keep last loop's speed.
            double measuredStep = difference(measured, lastMeasured);
            if (Math.abs(measuredStep) <= maxMeasuredRate * dt) {
                measuredRate = smooth(measuredRate, measuredStep / dt, dFilter);
            }

            // Opposite signs: we crossed the target, so the integral starts again from 0.
            if (error * lastError < 0) integral = 0;
        }

        double p = kP * error;
        double d = kD * (targetRate - measuredRate);
        double f = kF * targetRate;

        if (kI <= 0 || Math.abs(error) >= integralZone) {
            // I is off, or we are too far from the target: nothing adds up.
            integral = 0;
        } else if (!first) {
            double maxIntegral = maxIntegralOutput / kI;
            // Clip first too, in case kI was just raised.
            integral = clip(integral, maxIntegral);
            double outputBefore = p + kI * integral + d + f + extra;
            boolean atLimit = Math.abs(outputBefore) >= maxOutput && outputBefore * error > 0;
            if (!atLimit && !integralPaused) {
                integral = clip(integral + error * Math.min(dt, MAX_INTEGRAL_DT), maxIntegral);
            }
        }
        double i = kI * integral;

        // A NaN (NaN target, bad Panels value) would stay here for good, so forget it.
        if (Double.isNaN(integral)) integral = 0;
        if (Double.isNaN(targetRate)) targetRate = 0;
        if (Double.isNaN(measuredRate)) measuredRate = 0;

        lastTarget = target;
        lastMeasured = measured;
        lastError = error;
        lastP = p;
        lastI = i;
        lastD = d;
        lastF = f;
        return p + i + d + f + extra;
    }

    /** Empties the integral. Call it whenever something else drives the output (manual mode, a safety check). */
    public void resetIntegral() {
        integral = 0;
    }

    /** The P part of the last update(). */
    public double lastP() {
        return lastP;
    }

    /** The I part of the last update(). */
    public double lastI() {
        return lastI;
    }

    /** The D part of the last update(). */
    public double lastD() {
        return lastD;
    }

    /** The F part of the last update(). */
    public double lastF() {
        return lastF;
    }

    private double difference(double a, double b) {
        return wrapDegrees ? Angles.wrapDegrees(a - b) : a - b;
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    /**
     * One step of smoothing: filter 0 takes the new value as is, closer to 1 keeps more of the
     * old one. The filter is kept to 0..0.99, so a Panels typo can't freeze the value (1) or
     * make it swing (outside 0..1). Shooter and MotionPredictor smooth with it too.
     */
    public static double smooth(double old, double now, double filter) {
        double f = Math.min(Math.max(filter, 0), MAX_FILTER);
        return f * old + (1 - f) * now;
    }
}
