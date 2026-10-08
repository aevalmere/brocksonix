package org.firstinspires.ftc.teamcode.agateflow;

import org.firstinspires.ftc.teamcode.util.Angles;

import java.util.Arrays;

/**
 * Drives a Track in simulation the way Foresight drives a path, to estimate
 * the speed everywhere, the time, and where the robot will really go.
 *
 * What Foresight does, and so what this copies:
 * <ul>
 * <li>Full power along the path the whole way. It does not slow down for
 *     curves on its own: on a curve, part of the power goes to pushing
 *     sideways, so the robot is slower there because it has to be (see
 *     DriveModel), not because it planned to.</li>
 * <li>A speed limit on a segment (a Pedro Modifier) makes it drive at that
 *     speed instead.</li>
 * <li>It brakes once its brake distance reaches the end of the path, aiming
 *     for the speed that stops in what is left. The path counts as done a
 *     little before the end (parametricTConstraint), then it holds the pose
 *     to settle.</li>
 * <li>It steers so the point one brake distance ahead of the robot lands on
 *     the path. That is a look-ahead, so on curves the robot cuts inside the
 *     line, more the faster it goes.</li>
 * </ul>
 *
 * run() works out speed and time along a track: a forward pass at full power
 * (distance steps of about an inch, midpoint method), then the braking in
 * small time steps. It is quick, and AgateFlow uses it to rank candidates and
 * to place speed limits. pursue() runs a small copy of Foresight's control law
 * in 2D, in 10 ms time steps, to predict the real path, which is what gets
 * checked against the zones. driven() turns that predicted path into a track
 * with pursue()'s times: that is the predicted time. It is the better one for
 * time because it turns and drives at once the way Foresight does (run() can't
 * turn in place: at a sharp turn from a stop it guesses long).
 */
final class MotionModel {
    private MotionModel() {}

    /** Below this speed, time per step uses this instead, so a step from a stop isn't infinitely long. */
    private static final double CRAWL = 0.5;
    /**
     * A stop counts as reached when the robot is this close (inches, degrees) and
     * this slow (in/s). DriveRoute waits for it before it calls a leg done.
     */
    static final double ARRIVED_INCHES = 1, ARRIVED_DEGREES = 2, ARRIVED_SPEED = 2;
    /** Longest the robot is predicted to hold before it is at a stop, seconds. */
    private static final double MAX_SETTLE = 3;

    static final class Profile {
        /** Speed at each sample, inches/second. */
        final double[] speed;
        /** Time at each sample, seconds from the start of the leg. */
        final double[] time;
        /** From the start until the follower says it is done (path end plus settling). */
        final double seconds;
        /** Time when Foresight hands over to holding the end pose. */
        final double handoffSeconds;
        final double lowestVolts, peakAmps, topSpeed;

        Profile(double[] speed, double[] time, double seconds, double handoffSeconds,
                double lowestVolts, double peakAmps, double topSpeed) {
            this.speed = speed;
            this.time = time;
            this.seconds = seconds;
            this.handoffSeconds = handoffSeconds;
            this.lowestVolts = lowestVolts;
            this.peakAmps = peakAmps;
            this.topSpeed = topSpeed;
        }

        /** Seconds until done from this far along the planned path. */
        double secondsLeftFrom(Track track, double plannedDistance) {
            int i = track.indexAtPlanned(plannedDistance);
            if (i >= track.count - 1) return Math.max(0, seconds - time[track.count - 1]);
            double span = track.plannedS[i + 1] - track.plannedS[i];
            double f = span > 1e-9 ? Math.min(1, (plannedDistance - track.plannedS[i]) / span) : 0;
            double at = time[i] + f * (time[i + 1] - time[i]);
            return Math.max(0, seconds - at);
        }
    }

    /**
     * @param startSpeed   speed along the path at the start, inches/second
     * @param batteryVolts hub battery reading with the drive idle
     * @param handoff      distance before the end where Foresight's path is done (see AgateFlow)
     */
    static Profile run(Track track, DriveModel model, double startSpeed, double batteryVolts, double handoff) {
        return run(track, model, startSpeed, batteryVolts, handoff, model.settleSeconds);
    }

    /** Same, with the time from Foresight's handoff until the robot is at the stop (from pursue()). */
    static Profile run(Track track, DriveModel model, double startSpeed, double batteryVolts, double handoff, double settle) {
        int n = track.count;
        double[] v = new double[n];
        v[0] = Math.max(0, Math.min(startSpeed, holdSpeed(track, model, 0, batteryVolts)));
        double lowestVolts = batteryVolts, peakAmps = 0;

        // Forward: full power, limited by curves, turning, grip and speed limits.
        for (int i = 0; i < n - 1; i++) {
            double ds = track.s[i + 1] - track.s[i];
            if (ds <= 0) {
                v[i + 1] = v[i];
                continue;
            }
            double a1 = acceleration(track, model, i, v[i], batteryVolts);
            double vMid = Math.sqrt(Math.max(v[i] * v[i] + a1 * ds, 0));
            double a2 = acceleration(track, model, i, vMid, batteryVolts);
            double next = Math.sqrt(Math.max(v[i] * v[i] + 2 * a2 * ds, 0));
            if (a1 * a2 < 0) {
                // The speed where the power left after turning, curving or a speed limit just
                // holds it is inside this step (a sharp turn in place, say), and one big step
                // jumps past it. That speed is where it ends up: find it by halving.
                double low = a1 > 0 ? v[i] : Math.sqrt(Math.max(v[i] * v[i] + 2 * a1 * ds, 0));
                double high = a1 > 0 ? Math.sqrt(v[i] * v[i] + 2 * a1 * ds) : v[i];
                for (int k = 0; k < 12; k++) {
                    double mid = (low + high) / 2;
                    if (acceleration(track, model, i, mid, batteryVolts) > 0) low = mid;
                    else high = mid;
                }
                next = (low + high) / 2;
            }
            v[i + 1] = Math.min(next, holdSpeed(track, model, i + 1, batteryVolts));

            double angle = track.travel[i] - track.heading[i];
            double back = v[i] / model.speedPerVolt(angle);
            double volts = model.motorVolts(batteryVolts, 1, back);
            if (a1 > 0) {
                lowestVolts = Math.min(lowestVolts, volts);
                peakAmps = Math.max(peakAmps, model.driveAmps(volts, 1, back));
            }
        }

        // Time at full power, up to where Foresight starts braking: its brake distance reaches the end.
        double length = track.length();
        double[] t = new double[n];
        int brakeAt = n - 1;
        for (int i = 0; i < n - 1; i++) {
            double angle = track.travel[i] - track.heading[i];
            if (model.brakeDistance(v[i], angle) >= length - track.s[i]) {
                brakeAt = i;
                break;
            }
            t[i + 1] = t[i] + stepTime(track.s[i + 1] - track.s[i], v[i], v[i + 1]);
        }

        // From one sample before that, small time steps, so braking starts at the right spot.
        // Braking the way Foresight does it: aim for the speed that would stop in the distance
        // left after the current brake distance; power is that times the brake gain, and
        // reverse power is capped at maxBrakingPower.
        double handoffAt = Math.max(0, length - Math.max(handoff, 0));
        int i = Math.max(brakeAt - 1, 0);
        double s = track.s[i], speed = v[i], time = t[i];
        double handoffSeconds = Double.NaN;
        boolean braking = false;
        double dt = 0.004;
        for (int k = 0; k < 5000; k++) {
            if (s >= handoffAt) {
                handoffSeconds = time;
                break;
            }
            double angle = track.travel[i] - track.heading[i];
            double left = length - s;
            double beyond = left - model.brakeDistance(speed, angle);
            if (beyond <= 0) braking = true;
            double a;
            if (braking) {
                double target = beyond >= 0 ? model.brakeSpeed(beyond, angle) : -model.brakeSpeed(-beyond, angle);
                target = Math.min(target, speedLimit(track, model, track.indexAt(s + model.lookAhead(speed, angle)), batteryVolts, angle));
                double power = Math.max(model.brakeGain * target, -model.maxBrakingPower);
                a = (power * batteryVolts * model.speedPerVolt(angle) - speed) / model.timeConstant(angle);
                a = Math.max(-model.traction, Math.min(model.traction, a));
            } else {
                a = acceleration(track, model, i, speed, batteryVolts);
            }
            double next = Math.max(0, speed + a * dt);
            s += (speed + next) / 2 * dt;
            speed = next;
            time += dt;
            while (i < n - 1 && track.s[i + 1] <= s) {
                i++;
                v[i] = speed;
                t[i] = time;
            }
        }
        if (Double.isNaN(handoffSeconds)) handoffSeconds = time;
        for (int q = i + 1; q < n; q++) {
            v[q] = speed;
            t[q] = handoffSeconds;
        }
        double top = 0;
        for (double speedThere : v) top = Math.max(top, speedThere);
        return new Profile(v, t, handoffSeconds + settle, handoffSeconds, lowestVolts, peakAmps, top);
    }

    /** Time to cover ds at full power while the speed goes from a to b (steady acceleration over the step). */
    private static double stepTime(double ds, double a, double b) {
        if (ds <= 0) return 0;
        return 2 * ds / Math.max(a + b, 2 * CRAWL);
    }

    /** Acceleration along the path at sample i if the robot is going this fast there, in/s². */
    private static double acceleration(Track track, DriveModel model, int i, double speed, double batteryVolts) {
        double angle = track.travel[i] - track.heading[i];
        double across = angle + Math.PI / 2;
        double perVolt = model.speedPerVolt(angle);
        double volts = model.motorVolts(batteryVolts, 1, speed / perVolt);

        // Share of the wheels' power that turning and pushing sideways (around a curve) take.
        double turning = Math.abs(track.turnRate[i]) * speed / (volts * model.turnPerVolt);
        double sideways = speed * speed * Math.abs(track.bend[i]) * model.timeConstant(across)
                / (volts * model.speedPerVolt(across));
        double left = 1 - turning;
        double along = left > sideways ? Math.sqrt(left * left - sideways * sideways) : 0;

        double target = volts * along * perVolt;
        // Foresight moves on to the next segment (and its speed limit) once its projected pose,
        // one brake distance ahead, gets there. So the limit that counts is the one up ahead.
        double limit = Double.POSITIVE_INFINITY;
        if (track.hasLimits() || model.speedLimit != Double.POSITIVE_INFINITY || model.speedFraction != Double.POSITIVE_INFINITY) {
            int ahead = track.indexAt(track.s[i] + model.lookAhead(speed, angle));
            limit = speedLimit(track, model, ahead, batteryVolts, angle);
        }
        if (target > limit) target = limit;
        double a = (target - speed) / model.timeConstant(angle);

        // Grip: whatever the sideways (curve) acceleration leaves.
        double sidewaysAccel = speed * speed * Math.abs(track.bend[i]);
        double grip = Math.sqrt(Math.max(model.traction * model.traction - sidewaysAccel * sidewaysAccel, 0));
        a = Math.max(-grip, Math.min(grip, a));
        return Math.min(a, model.accelLimit);
    }

    /**
     * Foresight's drive power along the path at sample i: 1 (full power), or under
     * a speed limit (a Modifier, or Foresight's own maxVelocityConstraint or
     * maxPathSpeed) its coast kV x the limit.
     */
    private static double limitPower(Track track, DriveModel model, int i, double angle) {
        double fastest = model.foresightTopSpeed(angle);
        double limit = Math.min(track.cap[i], model.speedLimit);
        if (model.speedFraction != Double.POSITIVE_INFINITY) limit = Math.min(limit, model.speedFraction * fastest);
        if (limit >= fastest) return 1;
        return Math.min(1, model.coastGain * limit);
    }

    /**
     * How fast the robot settles at here because of a speed limit, +infinity if
     * none: limitPower's power, steered along the path (see DriveModel.steadySpeedPerVolt).
     * On a mecanum that is a little under the limit when strafing.
     */
    private static double speedLimit(Track track, DriveModel model, int i, double batteryVolts, double angle) {
        double power = limitPower(track, model, i, angle);
        if (power >= 1) return Double.POSITIVE_INFINITY;
        return Math.min(power * batteryVolts * model.steadySpeedPerVolt(angle), batteryVolts * model.speedPerVolt(angle));
    }

    /**
     * Fastest steady speed at sample i: the speed where the power left after
     * turning and curving just holds it. Above this the robot slides off the curve.
     * Speed limits aren't in it: Foresight eases down to those (see acceleration).
     */
    static double holdSpeed(Track track, DriveModel model, int i, double batteryVolts) {
        double angle = track.travel[i] - track.heading[i];
        double perVolt = model.speedPerVolt(angle);
        double free = batteryVolts * perVolt;
        double bend = Math.abs(track.bend[i]), turnRate = Math.abs(track.turnRate[i]);
        double limit = free;
        if (bend > 0) limit = Math.min(limit, Math.sqrt(model.traction / bend));
        if (bend < 1e-9 && turnRate < 1e-9) return limit;

        double across = angle + Math.PI / 2;
        double tauAcross = model.timeConstant(across), perVoltAcross = model.speedPerVolt(across);
        double low = 0, high = limit;
        for (int k = 0; k < 16; k++) {
            double mid = (low + high) / 2;
            double volts = model.motorVolts(batteryVolts, 1, mid / perVolt);
            double hold = mid / (volts * perVolt);
            double sideways = mid * mid * bend * tauAcross / (volts * perVoltAcross);
            double turning = turnRate * mid / (volts * model.turnPerVolt);
            if (Geometry.length(hold, sideways) + turning <= 1) low = mid;
            else high = mid;
        }
        return low;
    }

    /** Result of pursue(): the predicted path and the least spare room along it. */
    static final class Check {
        /** Least slack (see ClearanceRule). Negative means a collision is predicted. */
        final double worstSlack;
        /** Track sample nearest the worst point, or -1. */
        final int worstIndex;
        /** Track sample the robot was aiming at (nearest its projected pose) at the worst point. */
        final int worstAim;
        /** The predicted path as x, y pairs, about every half inch. */
        final double[] path;
        /** For each predicted point, the nearest sample of the planned track. */
        final int[] nearest;
        /** For each predicted point, the robot's predicted heading there (radians, unwrapped). */
        final double[] heading;
        /** Predicted points up to Foresight's handoff to holding; the rest are the hold. */
        final int handoffCount;
        /** Seconds from the handoff until the robot is at the stop (ARRIVED_*), and how far from it it was at the handoff. */
        final double settleSeconds, handoffGap;
        /** For each predicted point, seconds from the start and the speed there (in/s). */
        final double[] time, speed;
        /** Seconds from the start to the handoff. */
        final double driveSeconds;
        /** Lowest motor volts and most drive current while driving, and the top speed. */
        final double lowestVolts, peakAmps, topSpeed;

        Check(double worstSlack, int worstIndex, int worstAim, double[] path, int[] nearest, double[] heading,
              int handoffCount, double settleSeconds, double handoffGap, double[] time, double[] speed,
              double driveSeconds, double lowestVolts, double peakAmps, double topSpeed) {
            this.worstSlack = worstSlack;
            this.worstIndex = worstIndex;
            this.worstAim = worstAim;
            this.path = path;
            this.nearest = nearest;
            this.heading = heading;
            this.handoffCount = handoffCount;
            this.settleSeconds = settleSeconds;
            this.handoffGap = handoffGap;
            this.time = time;
            this.speed = speed;
            this.driveSeconds = driveSeconds;
            this.lowestVolts = lowestVolts;
            this.peakAmps = peakAmps;
            this.topSpeed = topSpeed;
        }

        /** Closest the predicted path comes to (x, y), inches. */
        double closestTo(double x, double y) {
            double best = Double.POSITIVE_INFINITY;
            for (int i = 0; i + 3 < path.length; i += 2) {
                double ax = path[i], ay = path[i + 1], dx = path[i + 2] - ax, dy = path[i + 3] - ay;
                double l2 = dx * dx + dy * dy;
                double f = l2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / l2));
                best = Math.min(best, Geometry.length(x - ax - f * dx, y - ay - f * dy));
            }
            return best;
        }
    }

    /**
     * Predicts where the robot will really drive, by running a small copy of
     * Foresight's control law against the DriveModel physics every 10 ms:
     * <ul>
     * <li>Project the pose ahead by the brake displacement (Foresight's brake model).</li>
     * <li>At the path point nearest the projected pose: turn toward the heading
     *     there, pull the projected pose sideways onto the path (P gains like
     *     ForesightTuner's), and push along the path at full power (or what a
     *     speed limit allows). Once the projected pose is past the end, brake.</li>
     * <li>Share the wheels' power in Foresight's order, each part scaled to what
     *     the wheels have left, like Foresight's allocator with Pedro's maxScaling:
     *     the heading feedforward, the pull, the push, the heading feedback. When the
     *     heading is far off (past Foresight's headingDeviationTolerance), the heading
     *     feedback goes before the pull (or before the push, if the robot is also far
     *     off the path), which can leave little for steering.</li>
     * <li>Like Foresight, never push against the way the robot is moving on an axis
     *     harder than maxBrakingPower. That is why a fast robot runs wide at a sharp
     *     corner: it can't brake its old direction hard.</li>
     * <li>Forward, strafe and turning each speed up toward power x volts x top
     *     speed per volt with their own time constant, within grip.</li>
     * </ul>
     * That lag is why the robot swings wide entering a fast curve and cuts inside
     * through it. Every step is checked against its stretch's ClearanceRule (with
     * the poses in between where it moved or turned a lot, see stepSlack), including
     * how fast the outline reaches a wall (WALL_CONTACT_SPEED). If it never reaches
     * the end, that counts as unsafe. The path counts as done
     * (the handoff) once the robot itself, not just its nearest track sample, is
     * within {@code handoff} of the end, like Foresight's parametric end.
     *
     * @param startVx      field-frame velocity at the start, in/s
     * @param startHeading the robot's heading at the start
     * @param handoff      distance before the end where Foresight's path is done
     */
    static Check pursue(Track track, DriveModel model, ClearanceRule[] rules, double startVx, double startVy,
                        double startHeading, double batteryVolts, double handoff) {
        int n = track.count;
        double length = track.length();
        double endX = track.x[n - 1], endY = track.y[n - 1];
        double px = track.x[0], py = track.y[0], vx = startVx, vy = startVy;
        // Heading is kept unwrapped, matching the track's headings.
        double h = track.heading[0] + Angles.wrapRadians(startHeading - track.heading[0]), w = 0;
        int robotNear = 0, aimNear = 0;
        double worst = Double.POSITIVE_INFINITY;
        int worstIndex = -1, worstAim = -1;
        double dt = 0.01;
        double[] path = new double[256];
        int[] near = new int[128];
        double[] headings = new double[128];
        double[] times = new double[128];
        double[] speeds = new double[128];
        path[0] = px;
        path[1] = py;
        headings[0] = h;
        speeds[0] = Geometry.length(vx, vy);
        int stored = 1;
        double lastX = px, lastY = py;
        boolean reachedEnd = false;
        double[] power = new double[3];
        double[] robot = new double[8];
        double lowestVolts = batteryVolts, peakAmps = 0, topSpeed = speeds[0];
        // Give up after 20 s, or longer if a speed limit makes the drive that slow.
        double slowest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < n; i++) slowest = Math.min(slowest, track.cap[i]);
        int steps = (int) Math.ceil(Math.max(20, 3 * length / Math.max(slowest, 5)) / dt);
        int step = 0;

        for (; step < steps; step++) {
            // Look well ahead: cutting across a hairpin, the nearest point jumps forward.
            robotNear = nearestAhead(track, robotNear, px, py, 36);
            if (length - alongAt(track, robotNear, px, py) <= handoff + 1e-6) {
                reachedEnd = true;
                break;
            }
            double c = Math.cos(h), s = Math.sin(h);
            double vForward = vx * c + vy * s, vStrafe = -vx * s + vy * c;

            // Projected pose: Foresight's brake displacement, worked out in the robot's frame.
            double dForward = model.brakeQuadraticForward * vForward * Math.abs(vForward) + model.brakeLinearForward * vForward;
            double dStrafe = model.brakeQuadraticStrafe * vStrafe * Math.abs(vStrafe) + model.brakeLinearStrafe * vStrafe;
            double qx = px + dForward * c - dStrafe * s, qy = py + dForward * s + dStrafe * c;
            double qh = h + model.headingBrakeQuadratic * w * Math.abs(w) + model.headingBrakeLinear * w;
            aimNear = nearestAhead(track, Math.max(aimNear, robotNear), qx, qy, 2 * Geometry.length(dForward, dStrafe) + 12);
            int j = aimNear;
            double tx = Math.cos(track.travel[j]), ty = Math.sin(track.travel[j]);
            double angle = track.travel[j] - h;

            double left = length - track.s[j];
            if (j == n - 1 || left < 0.01) left = tx * (endX - qx) + ty * (endY - qy);
            // Sideways error of the projected pose (positive: the path is to its left), and the pull back.
            double error = -ty * (track.x[j] - qx) + tx * (track.y[j] - qy);
            // Foresight's translational correction: each robot axis has its own kP (near or far,
            // by that axis's error), and the result is pointed back along the error.
            double errorX = -ty * error, errorY = tx * error;
            double errorForward = errorX * c + errorY * s, errorStrafe = -errorX * s + errorY * c;
            double kForward = Math.abs(errorForward) > 2.5 ? model.forwardGainFar : model.forwardGainNear;
            double kStrafe = Math.abs(errorStrafe) > 2.5 ? model.strafeGainFar : model.strafeGainNear;
            double squared = errorForward * errorForward + errorStrafe * errorStrafe;
            double along = squared > 1e-12 ? (kForward * errorForward * errorForward + kStrafe * errorStrafe * errorStrafe) / squared : 0;
            double pullForward = along * errorForward, pullStrafe = along * errorStrafe;
            double push;
            if (left > 0) {
                push = limitPower(track, model, j, angle);
            } else {
                push = Math.max(-model.brakeGain * model.brakeSpeed(-left, angle), -model.maxBrakingPower);
            }
            double pushForward = push * (tx * c + ty * s), pushStrafe = push * (-tx * s + ty * c);

            // Heading, Foresight's way: the projected pose's error sets the whole turning power,
            // the current pose's error the feedback part of it, and the rest is feedforward.
            double headingError = Geometry.wrap(track.heading[j] - qh);
            double total = model.headingGain * headingError;
            double feedback = model.headingGain * Geometry.wrap(track.heading[robotNear] - h);
            double feedforward;
            if (Math.abs(total) <= 0.001) {
                feedforward = 0;
                feedback = 0;
            } else if (total * feedback < 0) {
                feedforward = total;
                feedback = 0;
            } else if (Math.abs(feedback) >= Math.abs(total)) {
                feedforward = 0;
                feedback = total;
            } else {
                feedforward = total - feedback;
            }
            feedforward += feedback * model.headingDriveRatio;
            feedback *= 1 - model.headingDriveRatio;

            // Share out the wheels' power in Foresight's order (see above).
            boolean headingFirst = Math.abs(headingError) > model.headingTolerance;
            boolean pullFirst = Math.abs(error) > model.sideTolerance;
            power[0] = 0;
            power[1] = 0;
            power[2] = 0;
            double brake = model.maxBrakingPower;
            addScaled(power, 0, 0, feedforward, vForward, vStrafe, brake);
            if (headingFirst && pullFirst) {
                addScaled(power, pullForward, pullStrafe, 0, vForward, vStrafe, brake);
                addScaled(power, 0, 0, feedback, vForward, vStrafe, brake);
                addScaled(power, pushForward, pushStrafe, 0, vForward, vStrafe, brake);
            } else if (headingFirst) {
                addScaled(power, 0, 0, feedback, vForward, vStrafe, brake);
                addScaled(power, pullForward, pullStrafe, 0, vForward, vStrafe, brake);
                addScaled(power, pushForward, pushStrafe, 0, vForward, vStrafe, brake);
            } else {
                addScaled(power, pullForward, pullStrafe, 0, vForward, vStrafe, brake);
                addScaled(power, pushForward, pushStrafe, 0, vForward, vStrafe, brake);
                addScaled(power, 0, 0, feedback, vForward, vStrafe, brake);
            }
            power[0] = limitBraking(power[0], vForward, brake);
            power[1] = limitBraking(power[1], vStrafe, brake);

            double x0 = px, y0 = py, h0 = h;
            robot[0] = px;
            robot[1] = py;
            robot[2] = h;
            robot[3] = vx;
            robot[4] = vy;
            robot[5] = w;
            move(robot, power, model, batteryVolts, dt);
            px = robot[0];
            py = robot[1];
            h = robot[2];
            vx = robot[3];
            vy = robot[4];
            w = robot[5];
            lowestVolts = Math.min(lowestVolts, robot[6]);
            peakAmps = Math.max(peakAmps, robot[7]);
            double speed = Geometry.length(vx, vy);
            topSpeed = Math.max(topSpeed, speed);

            double slack = stepSlack(rules[track.stretch[robotNear]], x0, y0, h0, px, py, h, dt);
            if (slack < worst) {
                worst = slack;
                worstIndex = robotNear;
                worstAim = aimNear;
            }
            if (Geometry.length(px - lastX, py - lastY) >= 0.5) {
                if (2 * stored + 2 > path.length) path = Arrays.copyOf(path, path.length * 2);
                if (stored + 1 > near.length) {
                    near = Arrays.copyOf(near, near.length * 2);
                    headings = Arrays.copyOf(headings, headings.length * 2);
                    times = Arrays.copyOf(times, times.length * 2);
                    speeds = Arrays.copyOf(speeds, speeds.length * 2);
                }
                path[2 * stored] = px;
                path[2 * stored + 1] = py;
                near[stored] = robotNear;
                headings[stored] = h;
                times[stored] = (step + 1) * dt;
                speeds[stored] = speed;
                stored++;
                lastX = px;
                lastY = py;
            }
        }
        int handoffCount = stored;
        double driveSeconds = step * dt;
        double handoffGap = Geometry.length(endX - px, endY - py);
        double settle = model.settleSeconds;
        double endHeading = h + Angles.wrapRadians(track.heading[n - 1] - h);
        if (!reachedEnd) {
            worst = Math.min(worst, -1);
            worstIndex = robotNear;
            worstAim = aimNear;
        } else {
            // Foresight holds the stop: steer the projected pose onto it (full translational kP,
            // braking power capped), until the robot is there (ARRIVED_*). It can overshoot
            // here (more if Foresight's brake numbers don't match the drivetrain), so this part
            // is checked against the zones too: against the robot itself, not the safety margin,
            // since the robot is slow by now and the margin would mean crawling into every stop.
            ClearanceRule endRule = rules[track.stretch[n - 1]];
            if (endRule.radius > AgateFlow.ROBOT_RADIUS) endRule = endRule.withRadius(AgateFlow.ROBOT_RADIUS);
            double held = 0;
            while (held < MAX_SETTLE) {
                boolean there = Geometry.length(endX - px, endY - py) < ARRIVED_INCHES
                        && Math.abs(endHeading - h) < Math.toRadians(ARRIVED_DEGREES)
                        && Geometry.length(vx, vy) < ARRIVED_SPEED;
                if (there) break;
                double c = Math.cos(h), s = Math.sin(h);
                double vForward = vx * c + vy * s, vStrafe = -vx * s + vy * c;
                double dForward = model.brakeQuadraticForward * vForward * Math.abs(vForward) + model.brakeLinearForward * vForward;
                double dStrafe = model.brakeQuadraticStrafe * vStrafe * Math.abs(vStrafe) + model.brakeLinearStrafe * vStrafe;
                double qx = px + dForward * c - dStrafe * s, qy = py + dForward * s + dStrafe * c;
                double qh = h + model.headingBrakeQuadratic * w * Math.abs(w) + model.headingBrakeLinear * w;
                double ex = endX - qx, ey = endY - qy;
                double errorForward = ex * c + ey * s, errorStrafe = -ex * s + ey * c;
                double kForward = Math.abs(errorForward) > 2.5 ? model.forwardGainFar : model.forwardGainNear;
                double kStrafe = Math.abs(errorStrafe) > 2.5 ? model.strafeGainFar : model.strafeGainNear;
                double squared = errorForward * errorForward + errorStrafe * errorStrafe;
                double along = squared > 1e-12 ? (kForward * errorForward * errorForward + kStrafe * errorStrafe * errorStrafe) / squared : 0;
                power[0] = limitBraking(along * errorForward, vForward, model.maxBrakingPower);
                power[1] = limitBraking(along * errorStrafe, vStrafe, model.maxBrakingPower);
                power[2] = model.headingGain * (endHeading - qh);
                // The drivetrain scales every wheel down together if one would pass full power.
                double most = Math.abs(power[0]) + Math.abs(power[1]) + Math.abs(power[2]);
                if (most > 1) {
                    power[0] /= most;
                    power[1] /= most;
                    power[2] /= most;
                }
                double x0 = px, y0 = py, h0 = h;
                robot[0] = px;
                robot[1] = py;
                robot[2] = h;
                robot[3] = vx;
                robot[4] = vy;
                robot[5] = w;
                move(robot, power, model, batteryVolts, dt);
                px = robot[0];
                py = robot[1];
                h = robot[2];
                vx = robot[3];
                vy = robot[4];
                w = robot[5];
                held += dt;

                double slack = stepSlack(endRule, x0, y0, h0, px, py, h, dt);
                if (slack < worst) {
                    worst = slack;
                    worstIndex = n - 1;
                    worstAim = n - 1;
                }
                if (Geometry.length(px - lastX, py - lastY) >= 0.5) {
                    if (2 * stored + 2 > path.length) path = Arrays.copyOf(path, path.length * 2);
                    if (stored + 1 > near.length) {
                        near = Arrays.copyOf(near, near.length * 2);
                        headings = Arrays.copyOf(headings, headings.length * 2);
                        times = Arrays.copyOf(times, times.length * 2);
                        speeds = Arrays.copyOf(speeds, speeds.length * 2);
                    }
                    path[2 * stored] = px;
                    path[2 * stored + 1] = py;
                    near[stored] = n - 1;
                    headings[stored] = h;
                    times[stored] = driveSeconds + held;
                    speeds[stored] = Geometry.length(vx, vy);
                    stored++;
                    lastX = px;
                    lastY = py;
                }
            }
            settle = held;
        }
        // Finish the drawn path at the end point, where holding takes it.
        if (2 * stored + 2 > path.length) path = Arrays.copyOf(path, path.length + 2);
        if (stored + 1 > near.length) {
            near = Arrays.copyOf(near, near.length + 1);
            headings = Arrays.copyOf(headings, headings.length + 1);
            times = Arrays.copyOf(times, times.length + 1);
            speeds = Arrays.copyOf(speeds, speeds.length + 1);
        }
        path[2 * stored] = endX;
        path[2 * stored + 1] = endY;
        near[stored] = n - 1;
        headings[stored] = endHeading;
        times[stored] = driveSeconds + settle;
        speeds[stored] = 0;
        stored++;
        return new Check(worst, worstIndex, Math.max(worstAim, worstIndex), Arrays.copyOf(path, 2 * stored),
                Arrays.copyOf(near, stored), Arrays.copyOf(headings, stored), handoffCount, settle, handoffGap,
                Arrays.copyOf(times, stored), Arrays.copyOf(speeds, stored), driveSeconds, lowestVolts, peakAmps, topSpeed);
    }

    /** Where the outline comes this close to a wall, it counts as reaching it (for WALL_CONTACT_SPEED), inches. */
    private static final double WALL_TOUCH = 0.5;

    /**
     * The rule's spare room after one time step from (x0, y0, h0) to (x1, y1, h1).
     * Near a limit, after a step that moved or turned the outline a lot, the poses
     * in between are checked too: a corner swinging round as the robot turns can
     * poke into a wall between two steps. Reaching a wall (outline within WALL_TOUCH
     * of it) faster than WALL_CONTACT_SPEED along its normal counts as a miss of
     * 0.1 in. per in/s over, so the usual slowing down fixes it.
     */
    private static double stepSlack(ClearanceRule rule, double x0, double y0, double h0, double x1, double y1, double h1, double dt) {
        double slack = rule.slack(x1, y1, h1);
        double sweep = Geometry.length(x1 - x0, y1 - y0)
                + Math.hypot(Math.max(rule.front, rule.back), rule.halfWidth) * Math.abs(h1 - h0);
        if (sweep > 0.25 && slack < sweep + 0.5) {
            for (int k = 1; k < 4; k++) {
                double f = k / 4.0;
                slack = Math.min(slack, rule.slack(x0 + f * (x1 - x0), y0 + f * (y1 - y0), h0 + f * (h1 - h0)));
            }
        }
        double room = rule.wallRoom(x1, y1, h1);
        if (room < WALL_TOUCH) {
            double into = (rule.wallRoom(x0, y0, h0) - room) / dt;
            if (into > AgateFlow.WALL_CONTACT_SPEED) slack = Math.min(slack, -0.1 * (into - AgateFlow.WALL_CONTACT_SPEED));
        }
        return slack;
    }

    /**
     * How far along the track the point (x, y) is, near sample i: that sample's
     * distance plus how far the point is past it along the path, kept between the
     * samples on either side (and allowed past the end at the last one).
     */
    private static double alongAt(Track track, int i, double x, double y) {
        double past = (x - track.x[i]) * Math.cos(track.travel[i]) + (y - track.y[i]) * Math.sin(track.travel[i]);
        double along = track.s[i] + past;
        if (i > 0) along = Math.max(along, track.s[i - 1]);
        if (i < track.count - 1) along = Math.min(along, track.s[i + 1]);
        return along;
    }

    /**
     * One time step of the drivetrain (DriveModel's physics): forward, strafe and
     * turning each speed up toward power x volts x top speed per volt with their own
     * time constant, forward and strafe together within grip. The volts are the
     * battery's less the sag while the motors pull hard (DriveModel.motorVolts).
     * Speed is kept in the field's frame, so turning the robot doesn't turn its
     * momentum: the wheels have to push it round.
     *
     * @param robot {x, y, heading, field vx, field vy, turn rate, and after the step:
     *              motor volts, drive amps}, updated in place
     * @param power {forward, strafe, turn}, robot frame
     */
    private static void move(double[] robot, double[] power, DriveModel model, double batteryVolts, double dt) {
        double h = robot[2], c = Math.cos(h), s = Math.sin(h);
        double vForward = robot[3] * c + robot[4] * s, vStrafe = -robot[3] * s + robot[4] * c;
        double back = Math.abs(vForward) / model.forwardPerVolt + Math.abs(vStrafe) / model.strafePerVolt
                + Math.abs(robot[5]) / model.turnPerVolt;
        double load = Math.max(Math.min(1, Math.abs(power[0]) + Math.abs(power[1]) + Math.abs(power[2])), 1e-9);
        double volts = model.motorVolts(batteryVolts, load, back);
        robot[6] = volts;
        robot[7] = model.driveAmps(volts, load, back);
        double aForward = (power[0] * volts * model.forwardPerVolt - vForward) / model.forwardTau;
        double aStrafe = (power[1] * volts * model.strafePerVolt - vStrafe) / model.strafeTau;
        double a = Geometry.length(aForward, aStrafe);
        if (a > model.traction) {
            aForward *= model.traction / a;
            aStrafe *= model.traction / a;
        }
        double aTurn = (power[2] * volts * model.turnPerVolt - robot[5]) / model.turnTau;
        robot[3] += (aForward * c - aStrafe * s) * dt;
        robot[4] += (aForward * s + aStrafe * c) * dt;
        robot[5] += aTurn * dt;
        robot[0] += robot[3] * dt;
        robot[1] += robot[4] * dt;
        robot[2] += robot[5] * dt;
    }

    /**
     * Adds as much of (forward, strafe, turn) to the power as the wheels have room
     * for, like Pedro's Mecanum.maxScaling: each wheel is forward ± strafe ± turn,
     * and none may pass 1. Like Foresight, the room is worked out with braking
     * power capped (see limitBraking), but the uncapped amount is added.
     */
    private static void addScaled(double[] power, double forward, double strafe, double turn,
                                  double vForward, double vStrafe, double maxBraking) {
        double scale = 1;
        double f = limitBraking(power[0], vForward, maxBraking), s = limitBraking(power[1], vStrafe, maxBraking), t = power[2];
        double df = limitBraking(forward, vForward, maxBraking), ds = limitBraking(strafe, vStrafe, maxBraking);
        double[] now = {f - s - t, f + s + t, f + s - t, f - s + t};
        double[] change = {df - ds - turn, df + ds + turn, df + ds - turn, df - ds + turn};
        for (int i = 0; i < 4; i++) {
            if (Math.abs(change[i]) < 1e-9) continue;
            double up = (1 - now[i]) / change[i], down = (-1 - now[i]) / change[i];
            if (up >= 0 && up < scale) scale = up;
            if (down >= 0 && down < scale) scale = down;
        }
        scale = Math.max(0, Math.min(1, scale));
        power[0] += scale * forward;
        power[1] += scale * strafe;
        power[2] += scale * turn;
    }

    /** Foresight's getDrivePowers: power against the way the robot moves on this axis is capped at maxBraking. */
    private static double limitBraking(double power, double velocity, double maxBraking) {
        if (velocity * power >= 0) return power;
        return Math.copySign(Math.min(Math.abs(power), maxBraking), power);
    }

    /** Nearest track sample to (x, y) from {@code from} on, looking up to {@code reach} inches further along. */
    private static int nearestAhead(Track track, int from, double x, double y, double reach) {
        int best = from;
        double bestDistance = distanceSquared(track, from, x, y);
        double limit = track.s[from] + reach;
        for (int i = from + 1; i < track.count && track.s[i] <= limit; i++) {
            double d = distanceSquared(track, i, x, y);
            if (d <= bestDistance) {
                bestDistance = d;
                best = i;
            }
        }
        return best;
    }

    /** A predicted path as a Track, and the predicted speed and time along it. */
    static final class Driven {
        final Track track;
        final Profile profile;

        Driven(Track track, Profile profile) {
            this.track = track;
            this.profile = profile;
        }
    }

    /**
     * The predicted path as a Track of its own, about every inch, with pursue()'s
     * speed and time at each point: where and how the robot really goes, since
     * Foresight's look-ahead cuts short corners (shorter and gentler) and runs long
     * arcs a bit tighter, and the heading is the one pursue() predicted there.
     * Speed limits and stretch come from the nearest planned point. If the path is
     * too short to make a track, the planned one, timed by run().
     */
    static Driven driven(Track planned, Check check, DriveModel model, double startSpeed, double batteryVolts) {
        int points = check.path.length / 2;
        int[] keep = new int[points];
        int count = keep(check, keep);
        if (count < 2) {
            return new Driven(planned, run(planned, model, startSpeed, batteryVolts, check.handoffGap, check.settleSeconds));
        }
        Track track = track(planned, check, keep, count);
        double[] speed = new double[count], time = new double[count];
        for (int i = 0; i < count; i++) {
            // The last point is the stop, reached at the handoff (the hold after it is settleSeconds).
            boolean last = i == count - 1;
            time[i] = last ? check.driveSeconds : check.time[keep[i]];
            speed[i] = last ? 0 : check.speed[keep[i]];
        }
        Profile profile = new Profile(speed, time, check.driveSeconds + check.settleSeconds, check.driveSeconds,
                check.lowestVolts, check.peakAmps, check.topSpeed);
        return new Driven(track, profile);
    }

    /** The predicted points kept for the driven track (about an inch apart, none from the hold but the end), into keep. Returns how many. */
    private static int keep(Check check, int[] keep) {
        int points = check.path.length / 2;
        int count = 0;
        double lastX = Double.NaN, lastY = Double.NaN;
        for (int i = 0; i < points; i++) {
            // The hold after the handoff isn't driven along a path: its time is check.settleSeconds.
            if (i >= check.handoffCount && i < points - 1) continue;
            double x = check.path[2 * i], y = check.path[2 * i + 1];
            boolean last = i == points - 1;
            if (count > 0 && Geometry.length(x - lastX, y - lastY) < (last ? 1e-3 : 0.999)) {
                if (last) keep[count - 1] = i;
                continue;
            }
            keep[count++] = i;
            lastX = x;
            lastY = y;
        }
        return count;
    }

    /** The driven track through the kept predicted points. */
    private static Track track(Track planned, Check check, int[] keep, int count) {
        Track track = new Track(count);
        for (int i = 0; i < count; i++) {
            int p = keep[i], q = check.nearest[p];
            track.x[i] = check.path[2 * p];
            track.y[i] = check.path[2 * p + 1];
            track.s[i] = i == 0 ? 0 : track.s[i - 1] + Geometry.length(track.x[i] - track.x[i - 1], track.y[i] - track.y[i - 1]);
            track.heading[i] = check.heading[p];
            track.cap[i] = planned.cap[q];
            track.stretch[i] = planned.stretch[q];
            track.piece[i] = planned.piece[q];
            track.plannedS[i] = planned.s[q];
        }
        // Direction of travel from neighbors, unwrapped; then curvature from its change, lightly smoothed.
        for (int i = 0; i < count; i++) {
            int a = Math.max(i - 1, 0), b = Math.min(i + 1, count - 1);
            double angle = Math.atan2(track.y[b] - track.y[a], track.x[b] - track.x[a]);
            if (i > 0) angle = track.travel[i - 1] + Angles.wrapRadians(angle - track.travel[i - 1]);
            track.travel[i] = angle;
        }
        double[] raw = new double[count];
        for (int i = 0; i < count; i++) {
            int a = Math.max(i - 2, 0), b = Math.min(i + 2, count - 1);
            double ds = track.s[b] - track.s[a];
            raw[i] = ds > 1e-9 ? (track.travel[b] - track.travel[a]) / ds : 0;
        }
        for (int i = 0; i < count; i++) {
            int a = Math.max(i - 1, 0), b = Math.min(i + 1, count - 1);
            track.bend[i] = (raw[a] + raw[i] + raw[b]) / 3;
            double ds = track.s[b] - track.s[a];
            track.turnRate[i] = ds > 1e-9 ? (track.heading[b] - track.heading[a]) / ds : 0;
        }
        return track;
    }

    private static double distanceSquared(Track track, int i, double px, double py) {
        double dx = track.x[i] - px, dy = track.y[i] - py;
        return dx * dx + dy * dy;
    }
}
