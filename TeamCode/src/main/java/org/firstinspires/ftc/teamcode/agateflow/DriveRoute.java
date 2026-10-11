package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.firstinspires.ftc.teamcode.util.Angles;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.function.LongSupplier;

/**
 * The command AgateFlowCommands.driveTo makes. When it starts, it plans the Route
 * from wherever the robot is (on the background thread), then follows each leg.
 * While driving it:
 * <ul>
 * <li>replans if new zones block the rest of the path (vision saw something),
 *     or the robot is far from where it should be (pushed). It keeps driving
 *     the old path meanwhile, unless that path is blocked close ahead; then it
 *     stops until the new plan is ready. Whenever the zones differ from the
 *     plan's, it looks at the path just ahead every loop (also while a replan is
 *     being made, which can take a while on a Control Hub), so it stops in the
 *     same loop a block shows up close ahead. It stops by braking hard
 *     (STOP_BRAKING_POWER), then holds. Near a stop the robot is meant to get
 *     close to things, so there only a new zone it would hit (closer than
 *     ROBOT_RADIUS) counts: a partner robot parked on a stop makes it replan,
 *     and the replan moves that stop out of the zone (with
 *     AgateFlow.MOVE_TARGETS_OUT_OF_ZONES off it finds no plan, so the robot
 *     holds short of the zone until it goes or the time runs out). A replan
 *     that comes back after the robot moved on to the next leg, or that starts
 *     somewhere the robot no longer is, is out of date and is dropped. A replan
 *     within AgateFlow.FLOWER_INTAKE_RAISE_RUN of a Route.flowerIntakeDown stop
 *     still plans its start with the flower intake coming up.</li>
 * <li>gives up TIMEOUT_SCALE x its estimate + TIMEOUT_EXTRA_SECONDS after the
 *     first plan is ready. A replan can move that deadline up to
 *     REPLAN_EXTRA_SECONDS past the first one. A detour around something new
 *     in the way gets the time its own plan says it needs instead, but only
 *     MAX_DETOURS times, so a route that keeps replanning still gives up. A
 *     robot that is still getting closer to its stop at the deadline gets up to
 *     MAX_OVERTIME_SECONDS more. Then it holds where it is and ends, so an auto
 *     moves on instead of hanging.</li>
 * <li>reports each leg that ran start to finish on its own plan and got to its
 *     stop to EtaCalibration. Not the first leg of a replan, a leg the robot was
 *     pushed or stopped on, or a leg of driveTo(estimate) it joined partway.</li>
 * </ul>
 * It ends when the last stop is reached and settled, or with a status saying why not.
 */
public final class DriveRoute extends CommandBuilder {
    public enum Status {
        /** Hasn't started. */
        WAITING,
        /** Waiting for the first plan. */
        PLANNING,
        DRIVING,
        /** Done: at the last stop. */
        ARRIVED,
        /** Done: no safe plan (see problem()). The robot holds where it is. */
        NO_PLAN,
        /** Done: took too long. The robot holds where it is. */
        TIMED_OUT,
        /** Done: cancelled from outside. */
        STOPPED
    }

    /** The clock DriveRoute, Estimate and ParkGuard time things with, nanoseconds. The tests swap in a virtual one. */
    static LongSupplier NOW = System::nanoTime;

    /** After Foresight hands a leg over to holding, wait at most this much longer than predicted for the robot to be at the stop, seconds. */
    private static final double SETTLE_EXTRA_SECONDS = 0.5;
    /** A blocked point closer than this ahead (along the path) stops the robot until a new plan is ready, inches. */
    private static final double BLOCKED_STOP_DISTANCE = 30;
    /** Checking the path ahead for blocks, points closer together than this are skipped, inches. */
    private static final double CHECK_SPACING = 0.5;
    /** Braking for a block ends below this speed (in/s), or after MAX_BRAKE_SECONDS, and the robot holds. */
    private static final double STOPPED_SPEED = 2, MAX_BRAKE_SECONDS = 1;
    /** A point counts as blocked only if new zones took at least this much room from it, inches (so a zone that moves a hair doesn't stop the robot). */
    private static final double BLOCKED_WORSE_BY = 0.5;
    /** A replan made before the robot stopped for a block is used only if the robot is this close to its predicted path, inches. */
    private static final double PAUSED_ON_PATH = 3;
    /** The robot counts as getting there if it got this much closer to its stop in the last PROGRESS_SECONDS. */
    private static final double PROGRESS_INCHES = 0.5, PROGRESS_SECONDS = 0.5;

    private final AgateFlowCommands owner;
    private final Follower follower;
    private final Route route;

    private Status status = Status.WAITING;
    private String problem;
    private Plan plan;
    private Future<Plan> pending;
    private Alliance alliance;
    private int legIndex;
    private boolean legStarted, started;
    /**
     * EtaCalibration may learn from the current leg: it is not the first leg of a
     * replan, the robot wasn't pushed or stopped on it, and it wasn't joined partway.
     */
    private boolean learnFromLeg;
    /** The last leg that finished got to its stop (false: it gave up settling, see legFinished). */
    private boolean reachedStop;
    /** The first plan comes from a driveTo(estimate) that the robot joined partway along its first leg. */
    private boolean joinedPartway;
    /** Holding still because the path ahead is blocked, until a new plan is ready. */
    private boolean paused;
    /** How far along the current leg the robot was when it paused, inches. */
    private double pausedAt;
    /** Paused and still braking hard (see brake()); holding once the robot is nearly still. */
    private boolean braking;
    private long brakeSinceNanos;
    /** legIndex when the pending replan was asked for. If the robot has moved on to another leg since, that replan is out of date. */
    private int replanLeg;
    /** The pending replan is for something new in the way (not a push), and the robot stopped for it after asking. */
    private boolean replanForBlock, pausedSinceAsked;
    /**
     * The plan being driven, and the pending one, start with the flower intake still
     * coming up (Route.startFlowerDown): a replan soon after a flowerIntakeDown stop.
     */
    private boolean planFlowerStart, pendingFlowerStart;
    /** How many times a detour has moved the latest deadline (see MAX_DETOURS). */
    private int detours;
    /** Progress toward the current stop: the leg, the closest the robot has been to its stop, and when. */
    private int progressLeg = -1;
    private double closestToStop;
    private long progressNanos;
    private long legStartNanos, deadlineNanos, lastCheckNanos;
    /** Foresight finished the current leg's path and started holding its end, and when. */
    private boolean handedOff;
    private long handoffNanos;
    /** When the first plan was ready, and the latest a replan may move the deadline to. */
    private long drivingSinceNanos, latestDeadlineNanos;
    private double allowedSeconds;

    /** A plan made ahead (planFrom, or AgateFlowCommands.driveTo(estimate)), used if the robot is still where it was made from. */
    Estimate prepared;

    DriveRoute(AgateFlowCommands owner, Route route) {
        this.owner = owner;
        this.follower = owner.follower;
        this.route = route;
        requiring(owner.drive);
        setStart(this::begin);
        setExecute(this::step);
        setDone(this::finished);
        setEnd(this::finish);
    }

    /**
     * Starts planning now (in init, say) from where the robot will be when the
     * command starts, in field coordinates. If the robot is still there (within an
     * inch and a few degrees) and nothing else changed, the command uses this plan
     * and doesn't have to wait. Needs the alliance picked first.
     */
    public DriveRoute planFrom(Pose fieldStart) {
        if (RobotState.alliance == null) return this;
        prepared = owner.estimateFrom(fieldStart, route);
        return this;
    }

    public Status status() {
        return status;
    }

    /** Why there is no plan, or why the last replan failed. Null if nothing went wrong. */
    public String problem() {
        return problem;
    }

    /** The plan being driven, or null before the first one is ready. */
    public Plan plan() {
        return plan;
    }

    /**
     * Whether the stretch being driven is planned with the flower intake down
     * (Route.flowerIntakeDown). After the last stop, whether the last stretch was.
     * False before the first plan is ready. RobotCommands.withFlowerIntake lowers
     * and raises the intake with it.
     */
    public boolean flowerIntakeDown() {
        if (plan == null || plan.legs.isEmpty()) return false;
        if (legIndex >= plan.legs.size()) return stopOf(plan.legs.get(plan.legs.size() - 1)).flowerDown;
        Leg leg = plan.legs.get(legIndex);
        // Before the leg starts the follower still holds the last stop, so distanceDone would say "at the end".
        double done = !legStarted ? 0 : paused ? pausedAt : leg.distanceDone(follower);
        for (int i = 0; i < leg.waypoints.size() - 1; i++) {
            if (leg.waypointAt[i] > done) return leg.waypoints.get(i).flowerDown;
        }
        return stopOf(leg).flowerDown;
    }

    /** Predicted seconds until the last stop is settled, calibrated. NaN until there is a plan. */
    public double secondsLeft() {
        if (plan == null) return Double.NaN;
        if (finished()) return 0;
        double left = 0;
        for (int i = legIndex; i < plan.legs.size(); i++) {
            Leg leg = plan.legs.get(i);
            if (i == legIndex && paused) {
                // Holding still, so the follower can't say: use the plan from where it stopped.
                left += leg.secondsLeftFrom(pausedAt);
            } else if (i == legIndex && legStarted) {
                left += leg.path != null ? leg.secondsLeft(follower) : Math.max(0, leg.seconds - secondsSince(legStartNanos));
            } else {
                left += leg.seconds;
            }
        }
        return EtaCalibration.correct(left);
    }

    /** One line for telemetry. */
    public String describe() {
        switch (status) {
            case PLANNING:
                return "Planning";
            case DRIVING:
                return String.format("Leg %d of %d, %.1f s left%s", legIndex + 1, plan.legs.size(), secondsLeft(),
                        paused ? " (blocked, replanning)" : pending != null ? " (replanning)" : "");
            case NO_PLAN:
                return "No plan: " + problem;
            default:
                return status.toString();
        }
    }

    // ---- For the tests ----

    /** Which leg of plan() is being driven (0 is the first). */
    int legIndex() {
        return legIndex;
    }

    /** How many detours have had their own time so far (see MAX_DETOURS). */
    int detours() {
        return detours;
    }

    /** True while a plan or replan is on its way. */
    boolean planPending() {
        return pending != null;
    }

    // ---- The command ----

    private boolean finished() {
        return status == Status.ARRIVED || status == Status.NO_PLAN || status == Status.TIMED_OUT || status == Status.STOPPED;
    }

    private void begin() {
        started = true;
        status = Status.PLANNING;
        problem = null;
        plan = null;
        pending = null;
        legIndex = 0;
        legStarted = false;
        paused = false;
        braking = false;
        replanForBlock = false;
        pausedSinceAsked = false;
        joinedPartway = false;
        detours = 0;
        progressLeg = -1;
        pendingFlowerStart = route.startFlowerDown;
        alliance = RobotState.alliance;
        if (alliance == null) {
            problem = "No alliance picked";
            status = Status.NO_PLAN;
            prepared = null;
            return;
        }
        Pose pose = follower.pose();
        if (canUse(prepared, pose)) {
            pending = prepared.future;
            joinedPartway = !madeHere(prepared, pose);
        } else {
            pending = owner.planDrive(pose, follower.velocity(), route, alliance);
        }
        prepared = null;
    }

    /**
     * A plan made ahead still fits if the robot is where it was made from, or, if
     * it is less than half a second old, the robot is on its predicted path (it
     * kept driving the way the plan expected, so it just joins it partway).
     */
    private boolean canUse(Estimate estimate, Pose pose) {
        if (estimate == null || estimate.alliance != alliance) return false;
        if (madeHere(estimate, pose)) return true;
        Plan plan = estimate.plan();
        if (estimate.age() > 0.5 || plan == null || !plan.ok() || plan.legs.get(0).path == null) return false;
        return Geometry.distanceToPolyline(plan.legs.get(0).predictedPath, pose.x(), pose.y()) < 2;
    }

    /** The robot is where the estimate was made from: within an inch and 5°. */
    private static boolean madeHere(Estimate estimate, Pose pose) {
        return Geometry.length(pose.x() - estimate.from.x(), pose.y() - estimate.from.y()) < 1
                && Math.abs(Angles.wrapRadians(pose.heading() - estimate.from.heading())) < Math.toRadians(5);
    }

    private void step() {
        if (finished() || !started) return;
        long now = NOW.getAsLong();
        // While braking for a block, a new plan waits until the robot is still (see pause()).
        if (pending != null && pending.isDone() && !braking) takePlan(now);
        if (plan == null || finished()) return;

        trackProgress(now);
        if (now > deadlineNanos && !stillGettingThere(now)) {
            problem = String.format("Took longer than the %.1f s allowed (gave up after %.1f s)", allowedSeconds, (now - drivingSinceNanos) / 1e9);
            status = Status.TIMED_OUT;
            holdHere();
            return;
        }

        Leg leg = plan.legs.get(legIndex);
        boolean checkDue = AgateFlowCommands.REPLAN && pending == null
                && (now - lastCheckNanos) / 1e6 >= AgateFlowCommands.REPLAN_EVERY_MS;
        if (paused) {
            if (braking) {
                brake(leg, now);
                return;
            }
            // Stopped: keep asking for a new plan from here until one works (or the time runs out).
            if (checkDue) askForReplan(leg, pausedAt, true, now);
            return;
        }
        if (!legStarted) {
            startLeg(leg, now);
            return;
        }
        if (legFinished(leg, now)) {
            if (learnFromLeg && reachedStop) EtaCalibration.record(leg.seconds, secondsSince(legStartNanos));
            legIndex++;
            learnFromLeg = true;
            if (legIndex >= plan.legs.size()) {
                status = Status.ARRIVED;
                return;
            }
            startLeg(plan.legs.get(legIndex), now);
            return;
        }
        // Every loop, also while a replan is on its way: stop at once for a new block close ahead.
        if (AgateFlowCommands.REPLAN && watchAhead(leg, now)) return;
        if (checkDue) {
            lastCheckNanos = now;
            checkForReplan(leg, now);
        }
    }

    /** Remembers when the robot last got at least PROGRESS_INCHES closer to the stop it is driving to. */
    private void trackProgress(long now) {
        if (plan == null || legIndex >= plan.legs.size()) return;
        Pose end = plan.legs.get(legIndex).end, pose = follower.pose();
        double toStop = Geometry.length(pose.x() - end.x(), pose.y() - end.y());
        if (legIndex != progressLeg || toStop < closestToStop - PROGRESS_INCHES) {
            progressLeg = legIndex;
            closestToStop = toStop;
            progressNanos = now;
        }
    }

    /**
     * Past the deadline, a robot that is still getting closer to its stop (it is just
     * slower than predicted, say before EtaCalibration has learned), or is already
     * holding its stop and settling there (legFinished ends that soon), may go on for
     * up to AgateFlowCommands.MAX_OVERTIME_SECONDS more. A stuck one times out right away.
     */
    private boolean stillGettingThere(long now) {
        boolean moving = (now - progressNanos) / 1e9 < PROGRESS_SECONDS && !paused;
        boolean settling = legStarted && handedOff;
        return (moving || settling) && (now - deadlineNanos) / 1e9 < AgateFlowCommands.MAX_OVERTIME_SECONDS;
    }

    /**
     * Every loop while the zones differ from the plan's: if new zones block the
     * path within BLOCKED_STOP_DISTANCE ahead, stop now. Waiting for the next
     * REPLAN_EVERY_MS check, or for a replan to come back (which may take a second
     * on a slow Control Hub), would drive inches closer first. Only the stretch just
     * ahead is looked at, so this stays quick when vision changes the zones every
     * loop. Returns true if it stopped.
     */
    private boolean watchAhead(Leg leg, long now) {
        FieldMap.Snapshot snap = owner.map().snapshot();
        if (snap.version == plan.zonesVersion) return false;
        if (blockedAhead(snap, follower.pose(), BLOCKED_STOP_DISTANCE) >= BLOCKED_STOP_DISTANCE) return false;
        // A replan already on its way was asked for before this stop, from further back.
        if (pending != null) pausedSinceAsked = true;
        pause(leg, now);
        return true;
    }

    /**
     * Stops for a block: brakes hard (see brake()), then holds where the robot
     * stopped and asks for a new plan from there. Planning from where the robot
     * will really be, standing still, works better than planning from the moving
     * robot heading for the block: from there a fast robot often has no good way
     * (it has to brake anyway), and a plan made for it would be slow.
     */
    private void pause(Leg leg, long now) {
        paused = true;
        pausedAt = leg.distanceDone(follower);
        braking = true;
        brakeSinceNanos = now;
        learnFromLeg = false;
        brake(leg, now);
    }

    /**
     * Called every loop while stopping for a block. Foresight's hold brakes with at
     * most its maxBrakingPower (0.2 by default): from 50 in/s that takes about 11 in.
     * So push straight against the motion with STOP_BRAKING_POWER (about 7 in. at 0.5,
     * see there for why not more) until the robot is nearly still. It doesn't turn the
     * robot, and a path's speed limit doesn't matter here (it only slows down). Then
     * hold there and ask for a new plan, unless one is already on its way.
     */
    private void brake(Leg leg, long now) {
        Twist twist = follower.twist();  // robot frame: vx forward, vy left
        // A NaN speed (a localizer glitch) counts as stopped: hold rather than send NaN powers.
        if (!(Geometry.length(twist.vx, twist.vy) >= STOPPED_SPEED) || (now - brakeSinceNanos) / 1e9 > MAX_BRAKE_SECONDS) {
            braking = false;
            holdHere();
            if (pending == null) askForReplan(leg, pausedAt, true, now);
            return;
        }
        // Scaled so the busiest mecanum wheel (forward + strafe) gets STOP_BRAKING_POWER. No turning.
        double scale = Math.max(0, Math.min(1, AgateFlowCommands.STOP_BRAKING_POWER)) / (Math.abs(twist.vx) + Math.abs(twist.vy));
        follower.manual(-twist.vx * scale, -twist.vy * scale, 0);
    }

    /** Asks for a new plan of the waypoints not reached yet ({@code done} inches along this leg), from where the robot is now. */
    private void askForReplan(Leg leg, double done, boolean forBlock, long now) {
        lastCheckNanos = now;
        replanLeg = legIndex;
        replanForBlock = forBlock;
        pausedSinceAsked = false;
        // Pushed, or stopped for a block: this leg's time says nothing about the estimate.
        learnFromLeg = false;
        pendingFlowerStart = flowerComingUp(done);
        Route rest = remainingRoute(leg, done, pendingFlowerStart);
        pending = owner.planDrive(follower.pose(), follower.velocity(), rest, alliance);
    }

    private void takePlan(long now) {
        Plan fresh;
        try {
            fresh = pending.get();
        } catch (InterruptedException | ExecutionException e) {
            fresh = null;
        }
        pending = null;
        // A replan asked for during an earlier leg still starts by driving to that leg's
        // stop, which the robot has already reached. Drop it; the next check asks again.
        if (plan != null && replanLeg != legIndex) return;
        // The robot stopped for a block after this replan was asked for, so it may start
        // somewhere the robot no longer is. Use it only if the robot is on its predicted path.
        if (plan != null && pausedSinceAsked && fresh != null && fresh.ok() && fresh.legs.get(0).path != null
                && Geometry.distanceToPolyline(fresh.legs.get(0).predictedPath, follower.pose().x(), follower.pose().y()) > PAUSED_ON_PATH) {
            return;
        }
        if (fresh != null && fresh.ok()) {
            boolean replanned = plan != null;
            // Read before it is cleared below: a replan around something new in the way is a detour.
            boolean detour = replanned && replanForBlock;
            plan = fresh;
            planFlowerStart = pendingFlowerStart;
            legIndex = 0;
            legStarted = false;
            // The first leg of a replan starts where the robot was when it was asked, and a leg joined
            // partway wasn't driven whole: neither says how good the estimate is.
            learnFromLeg = !replanned && !joinedPartway;
            paused = false;
            braking = false;
            pausedSinceAsked = false;
            replanForBlock = false;
            // Progress starts again with the new plan: a detour may first lead away from the stop.
            progressLeg = -1;
            problem = null;
            double allowed = plan.seconds() * AgateFlowCommands.TIMEOUT_SCALE + AgateFlowCommands.TIMEOUT_EXTRA_SECONDS;
            long deadline = now + (long) (allowed * 1e9);
            if (!replanned) {
                drivingSinceNanos = now;
                latestDeadlineNanos = deadline + (long) (AgateFlowCommands.REPLAN_EXTRA_SECONDS * 1e9);
            } else if (detour && detours < AgateFlowCommands.MAX_DETOURS) {
                // A detour around something new in the way gets the time its own plan needs.
                // Counted, so a route that keeps finding new blocks still ends.
                detours++;
                latestDeadlineNanos = Math.max(latestDeadlineNanos, deadline);
            }
            // A replan may move the deadline, but not past the cap, so a route that keeps replanning still ends.
            deadlineNanos = Math.min(deadline, latestDeadlineNanos);
            allowedSeconds = (deadlineNanos - drivingSinceNanos) / 1e9;
            status = Status.DRIVING;
            return;
        }
        problem = fresh == null ? "Planning failed" : fresh.problem;
        if (plan == null) {
            status = Status.NO_PLAN;
            holdHere();
        }
        // Otherwise keep driving the old plan (or keep holding, if paused) and try again at the next check.
    }

    private void startLeg(Leg leg, long now) {
        if (leg.path != null) follower.follow(leg.path);
        else follower.hold(leg.end);
        legStarted = true;
        legStartNanos = now;
        handedOff = false;
    }

    /**
     * A leg is done once the robot is at its stop: within MotionModel.ARRIVED_INCHES
     * and ARRIVED_DEGREES and slower than ARRIVED_SPEED. Foresight finishes a path
     * a little before its end (and sometimes inches short, cutting into a short last
     * piece), then holds the end pose, so this waits for that hold to get there.
     * If it takes much longer than predicted, the leg counts as done anyway (the
     * follower keeps holding the pose), and reachedStop says it didn't get there.
     */
    private boolean legFinished(Leg leg, long now) {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        reachedStop = Geometry.length(pose.x() - leg.end.x(), pose.y() - leg.end.y()) < MotionModel.ARRIVED_INCHES
                && Math.abs(Angles.wrapRadians(pose.heading() - leg.end.heading())) < Math.toRadians(MotionModel.ARRIVED_DEGREES)
                && Geometry.length(velocity.vx, velocity.vy) < MotionModel.ARRIVED_SPEED;
        if (leg.path == null) return reachedStop || secondsSince(legStartNanos) > leg.seconds + SETTLE_EXTRA_SECONDS;
        if (follower.following()) return false;
        if (!handedOff) {
            handedOff = true;
            handoffNanos = now;
        }
        return reachedStop || secondsSince(handoffNanos) > leg.settleSeconds() + SETTLE_EXTRA_SECONDS;
    }

    /**
     * Replans if the path further ahead is blocked by new zones, or the robot is far
     * off its predicted path. It keeps driving meanwhile (watchAhead stops it if the
     * block comes close before the new plan is ready).
     */
    private void checkForReplan(Leg leg, long now) {
        FieldMap.Snapshot snap = owner.map().snapshot();
        Pose pose = follower.pose();
        double blockedAt = snap.version != plan.zonesVersion ? blockedAhead(snap, pose, Double.POSITIVE_INFINITY) : Double.POSITIVE_INFINITY;
        boolean pushed = leg.path != null && follower.following()
                && Geometry.distanceToPolyline(leg.predictedPath, pose.x(), pose.y()) > AgateFlowCommands.REPLAN_OFF_PATH;
        if (blockedAt == Double.POSITIVE_INFINITY && !pushed) return;
        askForReplan(leg, leg.distanceDone(follower), blockedAt != Double.POSITIVE_INFINITY, now);
    }

    /**
     * How far ahead (inches from the robot, along the predicted path) the rest of
     * that path first passes too close to a zone because of the new zones, or
     * +infinity if it doesn't within {@code limit}. A point counts only if the new
     * zones took room from it: plans pass close to things on purpose (a FLOWER
     * pickup spot, the GARDEN lane, a corner), and those haven't changed. Near a
     * stop (each leg's end, and the start of each later leg, the stop before it) the
     * robot is meant to get close, so there a point counts only if a new zone comes
     * closer than ROBOT_RADIUS: the robot would hit it (a partner parked on the stop).
     * Points closer than CHECK_SPACING to the last one looked at are skipped: room
     * changes at most that much between them, and the check leaves SAFETY_MARGIN / 2
     * to spare.
     */
    private double blockedAhead(FieldMap.Snapshot snap, Pose pose, double limit) {
        double need = AgateFlow.ROBOT_RADIUS + AgateFlow.SAFETY_MARGIN / 2;
        double nearStop = AgateFlow.ROBOT_RADIUS + AgateFlow.SAFETY_MARGIN;
        double along = 0, lastX = pose.x(), lastY = pose.y();
        for (int i = legIndex; i < plan.legs.size(); i++) {
            Leg leg = plan.legs.get(i);
            double[] path = leg.predictedPath;
            int from = i == legIndex ? nearestPoint(path, pose.x(), pose.y()) : 0;
            for (int q = from; q + 1 < path.length; q += 2) {
                double x = path[q], y = path[q + 1];
                double step = Geometry.length(x - lastX, y - lastY);
                if (step < CHECK_SPACING && q != from) continue;
                along += step;
                lastX = x;
                lastY = y;
                if (along > limit) return Double.POSITIVE_INFINITY;
                boolean byStop = Geometry.length(x - leg.end.x(), y - leg.end.y()) < nearStop
                        || (i > legIndex && Geometry.length(x - path[0], y - path[1]) < nearStop);
                double least = byStop ? AgateFlow.ROBOT_RADIUS : need;
                double room = snap.clearance(x, y);
                if (room < least && room < plan.zones.clearance(x, y) - BLOCKED_WORSE_BY) return along;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    /**
     * Whether the flower intake may still be coming up {@code done} inches along the
     * current leg: the leg starts at a Route.flowerIntakeDown stop (or the plan started
     * with the intake down), and the robot is still within
     * AgateFlow.FLOWER_INTAKE_RAISE_RUN of that start.
     */
    private boolean flowerComingUp(double done) {
        boolean leftDown = legIndex == 0 ? planFlowerStart : stopOf(plan.legs.get(legIndex - 1)).flowerDown;
        return leftDown && done < AgateFlow.FLOWER_INTAKE_RAISE_RUN;
    }

    /**
     * The waypoints not reached yet ({@code done} inches along this leg): the rest of
     * this leg's, then every later leg's. {@code flowerStart}: it starts with the
     * flower intake still coming up.
     */
    private Route remainingRoute(Leg leg, double done, boolean flowerStart) {
        List<Waypoint> rest = new ArrayList<>();
        for (int i = 0; i < leg.waypoints.size(); i++) {
            boolean last = i == leg.waypoints.size() - 1;
            if (last || leg.waypointAt[i] > done + 1) rest.add(leg.waypoints.get(i));
        }
        for (int i = legIndex + 1; i < plan.legs.size(); i++) rest.addAll(plan.legs.get(i).waypoints);
        return Route.of(rest, flowerStart);
    }

    private void holdHere() {
        follower.hold(follower.pose());
    }

    /** Ivy may end a command twice, or one that never started, so this must be safe either way. */
    private void finish(EndCondition endCondition) {
        boolean wasDriving = status == Status.PLANNING || status == Status.DRIVING;
        if (wasDriving) status = Status.STOPPED;
        if (wasDriving && started && endCondition == EndCondition.INTERRUPTED) holdHere();
        braking = false;
        pending = null;
        started = false;
    }

    /** The leg's stop: its last waypoint. */
    private static Waypoint stopOf(Leg leg) {
        return leg.waypoints.get(leg.waypoints.size() - 1);
    }

    private static double secondsSince(long nanos) {
        return (NOW.getAsLong() - nanos) / 1e9;
    }

    private static int nearestPoint(double[] path, double x, double y) {
        int best = 0;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int q = 0; q + 1 < path.length; q += 2) {
            double d = Geometry.length(path[q] - x, path[q + 1] - y);
            if (d < bestDistance) {
                bestDistance = d;
                best = q;
            }
        }
        return best;
    }
}
