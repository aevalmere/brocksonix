package org.firstinspires.ftc.teamcode.agateflow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;

/**
 * How the drivetrain moves, for estimating drive times and predicting where
 * Foresight will really drive. The physics:
 *
 * <ul>
 * <li>Each drive motor is a DC motor: at full power it settles at a free speed
 *     proportional to the voltage it gets, and approaches it with a time
 *     constant (acceleration = (free speed - speed) / time constant).</li>
 * <li>The battery sags under load: volts at the hub = battery volts minus
 *     battery resistance times current, and current is highest when the motors
 *     are slow and pushing hard (starting off, or turning hard).</li>
 * <li>Mecanum: forward and strafe have different top speeds, and turning uses
 *     up part of each wheel's speed. Pushing sideways (to follow a curve) takes
 *     part of the power too, so a curve driven fast leaves less for speed.</li>
 * <li>Grip: the total acceleration can't pass TRACTION_ACCEL or the wheels slip.</li>
 * <li>Braking and look-ahead come from Foresight's own tuned brake numbers, the
 *     same ones the follower uses to decide when to brake.</li>
 * </ul>
 *
 * capture() reads the static fields and Foresight's config into one fixed
 * snapshot, so a Panels edit applies to the next plan and never halfway
 * through one.
 */
@Configurable
public final class DriveModel {
    // ---- From DriveModelTuner (pedro/procedures). Until then these are guesses. ----
    // TODO(2): Run DriveModelTuner (after ForesightTuner) and paste its output over these.
    /** Top forward speed per battery volt at full power, inches/second/volt. 0: use Foresight's max forward velocity / TUNED_VOLTS. */
    public static double FORWARD_SPEED_PER_VOLT = 0;
    /** Same for strafing. 0: use Foresight's max strafe velocity / TUNED_VOLTS. */
    public static double STRAFE_SPEED_PER_VOLT = 0;
    /** Top turning speed per volt, radians/second/volt. */
    public static double TURN_SPEED_PER_VOLT = 0.5;
    /** Seconds to reach about 63% of top speed from a stop, driving forward. */
    public static double FORWARD_TIME_CONSTANT = 0.35;
    public static double STRAFE_TIME_CONSTANT = 0.40;
    public static double TURN_TIME_CONSTANT = 0.25;
    /** Battery plus wiring resistance, ohms. */
    public static double BATTERY_OHMS = 0.10;

    // ---- Guesses to check ----
    // TODO(2): Write down the battery voltage while ForesightTuner runs.
    /** Battery volts while ForesightTuner measured its max velocities and gains. */
    public static double TUNED_VOLTS = 12.5;
    /** One drive motor's winding resistance, ohms. goBILDA 6000 RPM: 12 V / 9.2 A stall = 1.3. */
    public static double MOTOR_OHMS = 1.3;
    public static int DRIVE_MOTORS = 4;
    // TODO(2): Drive full power forward from a stop on the field tiles. If the wheels spin, lower this until the estimate matches.
    /** Most acceleration (in/s²) the wheels can grip with, any direction. About 0.8 g. */
    public static double TRACTION_ACCEL = 300;
    /** Time from reaching the end of a path until the follower is settled and says it is done. */
    public static double SETTLE_SECONDS = 0.10;
    // ---- Foresight's gains. They are read from the follower (see readGains); these are used only if that fails. ----
    /** Foresight's heading gain, power per radian of heading error. Used to predict turning while driving. */
    public static double HEADING_GAIN = 2.0;
    /**
     * Foresight's sideways correction gains, power per inch of error, which MotionModel
     * copies to predict where the robot really drives. NEAR is ForesightTuner's
     * Secondary kP (under 2.5 in. of error), FAR its Primary kP (past 2.5 in.). 0: what
     * ForesightTuner would compute from the time constants above. If these don't match
     * Foresight's, the predicted path is off by inches near tight spots.
     */
    public static double FORWARD_KP_NEAR = 0;
    public static double FORWARD_KP_FAR = 0;
    public static double STRAFE_KP_NEAR = 0;
    public static double STRAFE_KP_FAR = 0;
    /**
     * Foresight's brake controller gain (power per in/s of target speed). 0: what
     * ForesightTuner pastes, 0.85 / max forward velocity. Set it if you change c.brake.
     */
    public static double BRAKE_GAIN = 0;

    // ---- Used only when Foresight isn't set up yet (before tuning, or desktop tests) ----
    public static double FALLBACK_FORWARD_SPEED = 85;
    public static double FALLBACK_STRAFE_SPEED = 70;
    public static double FALLBACK_BRAKE_LINEAR_FORWARD = 0.06;
    public static double FALLBACK_BRAKE_LINEAR_STRAFE = 0.08;
    public static double FALLBACK_BRAKE_QUADRATIC_FORWARD = 0.0020;
    public static double FALLBACK_BRAKE_QUADRATIC_STRAFE = 0.0028;

    // ---- The snapshot ----
    final double forwardPerVolt, strafePerVolt, turnPerVolt;
    final double forwardTau, strafeTau, turnTau;
    final double batteryOhms, motorOhms, motors, traction, settleSeconds;
    final double brakeLinearForward, brakeLinearStrafe, brakeQuadraticForward, brakeQuadraticStrafe;
    /** Foresight ends a path this far before the end, as a fraction of the last segment's t. */
    final double endFraction;
    /** Foresight's own limits, +infinity if off. */
    final double speedLimit, accelLimit, speedFraction;
    /** Foresight's braking: power per in/s of target speed, and the most reverse power it uses. */
    final double brakeGain, maxBrakingPower;
    /** Foresight's heading control: gain, and its heading brake model (linear, quadratic). */
    final double headingGain, headingBrakeLinear, headingBrakeQuadratic;
    /** Foresight's sideways correction gains (power per inch): forward and strafe, under and past 2.5 in. of error. */
    final double forwardGainNear, forwardGainFar, strafeGainNear, strafeGainFar;
    /** Foresight's coast controller: power per in/s of a speed limit (a Modifier's maxVelocityConstraint). */
    final double coastGain;
    /** Foresight's maxAchievableForwardVelocity and maxAchievableStrafeVelocity, in/s. */
    final double foresightForward, foresightStrafe;
    /**
     * Foresight's power sharing: past these errors (radians of heading, inches off the
     * path) it gives turning, or pulling back onto the path, its wheel power first.
     * And the part of the heading correction it counts as feedforward.
     */
    final double headingTolerance, sideTolerance, headingDriveRatio;
    /** Foresight's maxVelocityConstraint, as last read while no path was being followed. */
    private static double lastSpeedLimit = Double.POSITIVE_INFINITY;
    /** Foresight's gains as last read (see readGains), or null. Only the main thread uses it. */
    private static Gains lastGains;
    /** The follower's Foresight, for adding speed limits to path segments. Null if it isn't Foresight. */
    final Foresight foresight;

    private DriveModel(Follower follower) {
        Foresight found = follower != null && follower.algorithm() instanceof Foresight ? (Foresight) follower.algorithm() : null;
        ForesightConfig c = found == null ? null : found.config;

        double forwardSpeed = read(c == null ? null : c.maxAchievableForwardVelocity, FALLBACK_FORWARD_SPEED);
        double strafeSpeed = read(c == null ? null : c.maxAchievableStrafeVelocity, FALLBACK_STRAFE_SPEED);
        forwardPerVolt = FORWARD_SPEED_PER_VOLT > 0 ? FORWARD_SPEED_PER_VOLT : forwardSpeed / TUNED_VOLTS;
        strafePerVolt = STRAFE_SPEED_PER_VOLT > 0 ? STRAFE_SPEED_PER_VOLT : strafeSpeed / TUNED_VOLTS;
        turnPerVolt = TURN_SPEED_PER_VOLT;
        forwardTau = Math.max(FORWARD_TIME_CONSTANT, 0.01);
        strafeTau = Math.max(STRAFE_TIME_CONSTANT, 0.01);
        turnTau = Math.max(TURN_TIME_CONSTANT, 0.01);
        batteryOhms = Math.max(BATTERY_OHMS, 0);
        motorOhms = Math.max(MOTOR_OHMS, 0.01);
        motors = Math.max(DRIVE_MOTORS, 1);
        traction = TRACTION_ACCEL;

        double linearForward = FALLBACK_BRAKE_LINEAR_FORWARD, linearStrafe = FALLBACK_BRAKE_LINEAR_STRAFE;
        double quadraticForward = FALLBACK_BRAKE_QUADRATIC_FORWARD, quadraticStrafe = FALLBACK_BRAKE_QUADRATIC_STRAFE;
        double end = 0.025, timeout = Double.POSITIVE_INFINITY;
        double limit = Double.POSITIVE_INFINITY, accel = Double.POSITIVE_INFINITY, fraction = Double.POSITIVE_INFINITY;
        // Without Foresight's numbers: a spinning robot stops in about one time constant's worth of turning.
        double reverse = 0.2, headingLinear = TURN_TIME_CONSTANT, headingQuadratic = 0;
        double headingTol = Math.toRadians(11.25), sideTol = 2.5, driveRatio = 0.5;
        if (c != null) {
            try {
                Matrix linear = c.linearBrakeCoefficients.get(), quadratic = c.quadraticBrakeCoefficients.get();
                linearForward = linear.get(0, 0);
                linearStrafe = linear.get(1, 1);
                quadraticForward = quadratic.get(0, 0);
                quadraticStrafe = quadratic.get(1, 1);
            } catch (RuntimeException notSet) {
                // Keep the fallbacks.
            }
            end = read(c.parametricTConstraint, end);
            timeout = read(c.timeoutConstraint, 100.0) / 1000;
            // While a path is followed, a speed-limit Modifier may be holding this at its own value.
            if (!follower.following()) lastSpeedLimit = read(c.maxVelocityConstraint, limit);
            limit = lastSpeedLimit;
            accel = read(c.maxAccelerationConstraint, accel);
            fraction = read(c.maxPathSpeed, fraction);
            reverse = read(c.maxBrakingPower, reverse);
            headingTol = read(c.headingDeviationTolerance, headingTol);
            sideTol = read(c.translationalDeviationTolerance, sideTol);
            driveRatio = read(c.headingDriveRatio, driveRatio);
            try {
                headingLinear = c.headingBrakeCoefficients.get().x();
                headingQuadratic = c.headingBrakeCoefficients.get().y();
            } catch (RuntimeException notSet) {
                // Keep the guesses.
            }
        }
        brakeLinearForward = Math.max(linearForward, 1e-4);
        brakeLinearStrafe = Math.max(linearStrafe, 1e-4);
        brakeQuadraticForward = Math.max(quadraticForward, 1e-6);
        brakeQuadraticStrafe = Math.max(quadraticStrafe, 1e-6);
        endFraction = end;
        settleSeconds = Math.min(SETTLE_SECONDS, timeout);
        speedLimit = limit;
        accelLimit = accel;
        speedFraction = fraction;
        brakeGain = BRAKE_GAIN > 0 ? BRAKE_GAIN : 0.85 / forwardSpeed;
        maxBrakingPower = reverse;
        // ForesightTuner pastes coast kV = 1 / top speed and brake kV = 0.85 / top speed.
        coastGain = brakeGain / 0.85;
        foresightForward = forwardSpeed;
        foresightStrafe = strafeSpeed;
        headingTolerance = headingTol;
        sideTolerance = sideTol;
        headingDriveRatio = driveRatio;
        // Foresight's own gains if they could be read; else the numbers above; else (translational)
        // what ForesightTuner computes: kP = time constant x alpha^2 / top speed, alpha 6.2 under
        // 2.5 in. of error and 10.2 past it.
        Gains read = c == null ? null : readGains(follower, c);
        headingGain = read != null && read.heading > 0 ? read.heading : HEADING_GAIN;
        forwardGainNear = read != null && read.forwardNear > 0 ? read.forwardNear
                : FORWARD_KP_NEAR > 0 ? FORWARD_KP_NEAR : forwardTau * 6.2 * 6.2 / (forwardPerVolt * TUNED_VOLTS);
        forwardGainFar = read != null && read.forwardFar > 0 ? read.forwardFar
                : FORWARD_KP_FAR > 0 ? FORWARD_KP_FAR : forwardTau * 10.2 * 10.2 / (forwardPerVolt * TUNED_VOLTS);
        strafeGainNear = read != null && read.strafeNear > 0 ? read.strafeNear
                : STRAFE_KP_NEAR > 0 ? STRAFE_KP_NEAR : strafeTau * 6.2 * 6.2 / (strafePerVolt * TUNED_VOLTS);
        strafeGainFar = read != null && read.strafeFar > 0 ? read.strafeFar
                : STRAFE_KP_FAR > 0 ? STRAFE_KP_FAR : strafeTau * 10.2 * 10.2 / (strafePerVolt * TUNED_VOLTS);
        headingBrakeLinear = headingLinear;
        headingBrakeQuadratic = headingQuadratic;
        foresight = found;
    }

    /**
     * Reads every setting now. Pass the robot's follower, or null to use only the
     * static fields. Call it on the main (OpMode) thread: it reads the follower, and
     * may read Foresight's gains (see readGains).
     */
    public static DriveModel capture(Follower follower) {
        return new DriveModel(follower);
    }

    /** Foresight's gains, read from its controllers, and the controllers they came from. NaN: couldn't be read. */
    private static final class Gains {
        final Controller forwardController, strafeController, headingController;
        final double forwardNear, forwardFar, strafeNear, strafeFar, heading;

        Gains(Controller forwardController, Controller strafeController, Controller headingController) {
            this.forwardController = forwardController;
            this.strafeController = strafeController;
            this.headingController = headingController;
            forwardNear = probe(forwardController, 0.5, 1);
            forwardFar = probe(forwardController, 5, 10);
            strafeNear = probe(strafeController, 0.5, 1);
            strafeFar = probe(strafeController, 5, 10);
            heading = probe(headingController, 0.05, 0.1);
        }
    }

    /**
     * Foresight's translational and heading gains, read from its own controllers so
     * nobody has to copy them over by hand. A controller's gain isn't always a field
     * (a PiecewiseController keeps its pieces private), so each is probed: its output
     * for an error, divided by the error.
     *
     * Why probing is safe: it happens only while the follower is idle or driven by
     * hand (in init, say: AgateFlowCommands reads them when it is made), so Foresight
     * isn't using these controllers right then; follower.follow() resets them before
     * the next path anyway. Never while a path is followed or a pose held (hold()
     * doesn't reset them, and a PID's integral and last error would change how the
     * robot drives), and never on AgateFlow's planning thread: capture() runs on the
     * main thread. A PID keeps state (integral, last error, last time), so every probe
     * starts and ends with reset(): just after a reset its output is only kP x error
     * (no integral yet, no derivative on the first update), and it is left like a new
     * one. The result is kept, so later plans (made while driving) use it.
     *
     * Returns null if the controllers can't be read now and weren't before. Then, and
     * for any gain that isn't simply proportional (see probe), the static fields above
     * are used.
     */
    private static Gains readGains(Follower follower, ForesightConfig c) {
        Controller forward, strafe, heading;
        try {
            forward = c.forwardTranslational.get();
            strafe = c.strafeTranslational.get();
            heading = c.headingFeedback.get();
        } catch (RuntimeException notSet) {
            return null;
        }
        Gains known = lastGains;
        if (known != null && known.forwardController == forward && known.strafeController == strafe
                && known.headingController == heading) {
            return known;
        }
        if (!follower.idle() && !follower.manual()) return null;
        lastGains = new Gains(forward, strafe, heading);
        return lastGains;
    }

    /**
     * The controller's output per unit of error, from errors a and b (both under, or
     * both past, the 2.5 in. where ForesightTuner's kP changes). NaN unless both give
     * the same positive gain within 1%: a controller with a static part, or one that
     * isn't proportional, isn't something MotionModel can copy.
     */
    private static double probe(Controller controller, double a, double b) {
        try {
            controller.reset();
            double first = controller.calculate(0, a) / a;
            controller.reset();
            double second = controller.calculate(0, b) / b;
            controller.reset();
            boolean usable = first > 0 && !Double.isInfinite(first) && Math.abs(second - first) <= 0.01 * first;
            return usable ? (first + second) / 2 : Double.NaN;
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    private static double read(com.pedropathing.config.ConfigVar<Double> value, double fallback) {
        if (value == null) return fallback;
        try {
            Double v = value.get();
            return v == null || v.isNaN() ? fallback : v;
        } catch (RuntimeException notSet) {
            return fallback;
        }
    }

    // ---- Direction helpers. angle is the direction of travel relative to the robot's front, radians. ----

    /** Top speed per volt in this direction (mecanum: forward and strafe speeds share each wheel). */
    double speedPerVolt(double angle) {
        return 1 / (Math.abs(Math.cos(angle)) / forwardPerVolt + Math.abs(Math.sin(angle)) / strafePerVolt);
    }

    /**
     * Speed along the path per volt of power pushed along it, when that power is
     * below what the wheels can take (a speed limit), so the robot isn't at full
     * power. Forward and strafe answer separately to their share of the push,
     * and Foresight's sideways pull keeps the robot on the path, which works out
     * to forward x strafe / (forward x sin^2 + strafe x cos^2). Straight forward
     * it is forwardPerVolt, straight sideways strafePerVolt.
     */
    double steadySpeedPerVolt(double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return forwardPerVolt * strafePerVolt / (forwardPerVolt * s * s + strafePerVolt * c * c);
    }

    /** Foresight's top speed in this direction (its interpolateVelocity over the two maxAchievable velocities). */
    double foresightTopSpeed(double angle) {
        return 1 / (Math.abs(Math.cos(angle)) / foresightForward + Math.abs(Math.sin(angle)) / foresightStrafe);
    }

    double timeConstant(double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return forwardTau * c * c + strafeTau * s * s;
    }

    /**
     * Distance Foresight expects to need to stop from this speed: the same
     * quadratic + linear model Foresight brakes with (getVelocityToBrakeInTime).
     */
    double brakeDistance(double speed, double angle) {
        double c = Math.abs(Math.cos(angle)), s = Math.abs(Math.sin(angle));
        double quadratic = brakeQuadraticForward * c * c * c + brakeQuadraticStrafe * s * s * s;
        double linear = brakeLinearForward * c * c + brakeLinearStrafe * s * s;
        return quadratic * speed * speed + linear * speed;
    }

    /** The highest speed Foresight's brake curve allows with this far left to go. */
    double brakeSpeed(double remaining, double angle) {
        if (remaining <= 0) return 0;
        double c = Math.abs(Math.cos(angle)), s = Math.abs(Math.sin(angle));
        double quadratic = brakeQuadraticForward * c * c * c + brakeQuadraticStrafe * s * s * s;
        double linear = brakeLinearForward * c * c + brakeLinearStrafe * s * s;
        return (-linear + Math.sqrt(linear * linear + 4 * quadratic * remaining)) / (2 * quadratic);
    }

    /**
     * How far ahead Foresight looks at this speed: the length of its brake
     * displacement (getBrakeDisplacement). Foresight steers so that point lands
     * on the path, which is why it cuts inside on curves.
     */
    double lookAhead(double speed, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        double forward = brakeQuadraticForward * c * Math.abs(c) * speed * speed + brakeLinearForward * c * speed;
        double strafe = brakeQuadraticStrafe * s * Math.abs(s) * speed * speed + brakeLinearStrafe * s * speed;
        return Geometry.length(forward, strafe);
    }

    /**
     * Foresight's sideways correction gain (power per inch of error) in this
     * direction: FORWARD_KP_* along the robot, STRAFE_KP_* across it, the NEAR one
     * under 2.5 in. of error and the FAR one past it, like Foresight's piecewise kP.
     */
    double correctionGain(double error, double angle) {
        double forward = error > 2.5 ? forwardGainFar : forwardGainNear;
        double strafe = error > 2.5 ? strafeGainFar : strafeGainNear;
        double c = Math.cos(angle), s = Math.sin(angle);
        return forward * c * c + strafe * s * s;
    }

    /**
     * Volts at the motors while driving at this speed with this much power.
     * Solves "battery volts minus sag" together with the motor current it causes.
     *
     * @param batteryVolts the hub's battery reading with the drive idle
     * @param power        drive power, 0 to 1
     * @param backVolts    the motors' back-EMF at this speed (speed / speedPerVolt)
     */
    double motorVolts(double batteryVolts, double power, double backVolts) {
        double k = batteryOhms * motors / motorOhms;
        double loaded = (batteryVolts + k * backVolts) / (1 + k * power);
        // If the power is below what the speed needs, the motors aren't drawing current.
        return power * loaded >= backVolts ? loaded : batteryVolts;
    }

    /** Current the drive motors draw at these volts, power and speed, amps. */
    double driveAmps(double volts, double power, double backVolts) {
        return Math.max(0, power * volts - backVolts) * motors / motorOhms;
    }
}
