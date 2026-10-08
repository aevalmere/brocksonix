package org.firstinspires.ftc.teamcode.agateflow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.CompoundPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.firstinspires.ftc.teamcode.util.Angles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Turns a Route into Pedro paths from wherever the robot is right now, around
 * every zone in the FieldMap, picking the fastest safe way it can find.
 *
 * For each stretch between two waypoints:
 * <ol>
 * <li><b>Shortest ways around.</b> A visibility graph (VisibilityGraph) gives
 *     straight-line routes around the zones grown by the robot's size. It
 *     searches twice: with extra CORNER_ROOM around zones (so corners can be
 *     wide and fast) and without (to squeeze through gaps), each with a few
 *     turn weights (so it also tries routes with fewer, gentler turns).</li>
 * <li><b>Round the corners.</b> Each corner becomes a CornerCurve (smooth to
 *     the second derivative), as big as the room around it allows. Bigger is
 *     both shorter and faster.</li>
 * <li><b>Drive it in simulation.</b> MotionModel drives each candidate the
 *     way Foresight would and gives the time. It also predicts where the robot
 *     will really go (Foresight cuts inside on curves, more when fast). If that
 *     would get too close to a zone, that part of the path gets a speed limit
 *     (a Pedro Modifier, starting early enough to slow down), and it tries again.</li>
 * <li><b>Pick the fastest</b> safe candidate. If none keeps a forced direction
 *     (a moving start, a through-waypoint), the ones that don't are tried too.
 *     If none is safe, the closest one is still taken when it misses the
 *     SAFETY_MARGIN by only a little (NEAR_MISS_FRACTION) and keeps clear of
 *     ROBOT_RADIUS; Plan.notes says where.</li>
 * </ol>
 * Then each leg (start to stop, through any through-waypoints) is built into
 * one Pedro Path: lines and CornerCurves, heading from the waypoints' Heading,
 * and speed limits as Modifiers. A through-waypoint gets a short straight
 * through it, so the direction and heading there are what was asked, and the
 * robot passes within THROUGH_TOLERANCE (slowing down there if it must). No
 * path ever turns back on itself (Foresight can get stuck on a hairpin), so a
 * through-waypoint where the route turns back becomes a stop, with a note.
 *
 * A Route.pickUp becomes passes over its targets in the fastest order (see Pickup).
 *
 * Everything is in field coordinates. Route poses are written for red and
 * flipped here for the alliance passed in. Pure math: no hardware, safe to call
 * in init or mid-match. The zones' graph is kept between plans and only rebuilt
 * when the zones change.
 */
@Configurable
public final class AgateFlow {
    // TODO(8): Measure the farthest point of the robot from the turret center (wedge tips, intake). A circle this big must cover the whole robot.
    /** Circle around the turret center that covers the whole robot, inches. */
    public static double ROBOT_RADIUS = 10;
    // TODO(8): Copy the measured Field.CENTER_TO_INTAKE_WALL into ROBOT_FRONT and CENTER_TO_SIDE_WALL into ROBOT_HALF_WIDTH; tape the turret center to the chassis back for ROBOT_BACK.
    /**
     * The robot's outline around the point the localizer tracks, flower intake up
     * (how ParkGuard and every drive with it up go), inches, from CAD (ROBOT.md).
     * The Pinpoint's pod offsets are measured from the turret center, which is
     * also the chassis center (wedges left out): 145.4 mm behind the chassis front.
     * Front: to the front of the intake (203 mm). Back: to the chassis back
     * (145.4 mm). Half width: the outer frame rails (298 mm across; the chassis
     * plate is 280 mm, so this is the wider one). ROBOT_RADIUS must still cover
     * all four corners (now 9.9 in.).
     */
    public static double ROBOT_FRONT = 7.99;
    public static double ROBOT_BACK = 5.72;
    public static double ROBOT_HALF_WIDTH = 5.87;
    /**
     * Check the predicted path with the robot's outline (true), so it may pass
     * things closer side-on than front-on, or with the ROBOT_RADIUS circle (false).
     */
    public static boolean USE_OUTLINE = true;
    // TODO(12): Raise it if rubbing a wall slows the robot down or drags its heading round.
    /**
     * How close the robot's outline may come to the four field walls, inches. Any
     * part may touch them (0): the intake, the sides, the back. Other zones keep
     * SAFETY_MARGIN. So the robot can drive along a wall with its center
     * ROBOT_HALF_WIDTH from it side-on, ROBOT_BACK back-on, ROBOT_FRONT front-on.
     */
    public static double WALL_GAP = 0;
    // TODO(12): Lower it if the odometry pods skip or the robot jolts when it reaches a wall; raise it if wall runs are slow.
    /**
     * Fastest the robot may move into a wall (along the wall's normal) where its
     * outline reaches one, in/s. It slides along a wall or eases onto it, never
     * slams into it. AgateFlow puts a speed limit before the contact if needed.
     */
    public static double WALL_CONTACT_SPEED = 10;
    // TODO(12): Raise if the robot brushes zones on planned paths, lower if paths go needlessly wide.
    /** Extra room past ROBOT_RADIUS on top of what AgateFlow predicts, inches. */
    public static double SAFETY_MARGIN = 2;
    // TODO(8): The smallest distance from the turret center to the robot's outside (the back is about 5.7 in.).
    /** Closer than this to a zone the robot's body is in it. Targets closer than this are moved out. */
    public static double CONTACT_RADIUS = 5.5;
    /** Extra room around zone corners in the first search, so corners can be rounded wide and taken fast, inches. */
    public static double CORNER_ROOM = 4;
    /** How many inches of extra path a right-angle turn is worth avoiding (the search also tries 0 and 3x this). */
    public static double TURN_WEIGHT = 12;
    /** Near the start and end, the robot may be close to a zone but must leave it at least this steeply (inches away per inch driven). */
    public static double ESCAPE_SLOPE = 0.5;
    /** Through-waypoints are passed within this, inches. Foresight cuts them at speed, so AgateFlow slows down there if needed. */
    public static double THROUGH_TOLERANCE = 2;
    /** Straight run on each side of a through-waypoint, inches. Longer: straighter through it. Shorter: tighter route. */
    public static double PASS_RUN = 12;
    /** Straight run into a stop whose heading is tangent (so it arrives facing that heading), inches. */
    public static double TANGENT_APPROACH = 6;
    /**
     * When turning right at the start would swing the robot's corners into a zone
     * next to it (it starts against a wall, say), it keeps its start heading for
     * the first stretch of the way, at most this far, inches, then turns.
     */
    public static double MAX_START_HOLD = 24;
    /** Inches over which the robot turns from its start heading to the planned one after such a hold. */
    public static double START_HOLD_BLEND = 8;
    /**
     * When turning right before a waypoint would swing the robot's corners into
     * something next to it (a wall beside a stop, say), it finishes the turn earlier
     * and drives the last part already facing the waypoint's heading, at most this
     * far, inches.
     */
    public static double MAX_END_HOLD = 24;
    // TODO(8): Time how long the flower intake takes to come up, and set this to how far the robot backs away from a FLOWER in that time.
    /**
     * After a Route.flowerIntakeDown stretch the intake is still coming up for a moment:
     * the next stretch is planned with it down (reaching IntakePoses.FLOWER_INTAKE_REACH
     * ahead) for this many inches from its start, so turning away from a FLOWER can't
     * sweep it through the FLOWER.
     */
    public static double FLOWER_INTAKE_RAISE_RUN = 12;
    /** Above this speed (in/s) a new plan starts in the direction the robot is already moving. */
    public static double MOVING_SPEED = 6;
    /** Slowest speed limit AgateFlow will add to get safely around a tight corner, in/s. Below this it gives up on that route. */
    public static double MIN_SPEED_LIMIT = 12;
    /**
     * Slowest speed limit AgateFlow will put on the last part before a stop when the
     * robot is predicted to overshoot the stop into a zone while settling, in/s. A
     * follower whose brake model doesn't match the drivetrain overshoots stops.
     */
    public static double MIN_STOP_APPROACH_SPEED = 6;
    /**
     * If no way keeps the whole SAFETY_MARGIN, still take the closest one when it is
     * short by less than this fraction of the margin and keeps clear of ROBOT_RADIUS
     * itself (it says so in Plan.notes). 0: never.
     */
    public static double NEAR_MISS_FRACTION = 0.25;
    /** Stop trying more candidate routes after this long (at least one is always tried), ms. AgateFlowCommands plans in the background, so the loop doesn't wait. */
    public static double PLAN_TIME_LIMIT_MS = 100;
    /** Move a target that is inside (or touching) a zone to the nearest free spot. False: refuse to plan instead. */
    public static boolean MOVE_TARGETS_OUT_OF_ZONES = true;

    // ---- Picking up (Route.pickUp, see Pickup) ----
    // TODO(8): Drive into a POLLEN at rising speeds; set this to the fastest that still picks it up every time.
    /** Speed limit at each pickup, from PICKUP_RUN before it to PICKUP_EXIT after it, in/s. */
    public static double PICKUP_SPEED = 30;
    /** Half the intake's width, inches: it is as wide as the chassis (280 mm, ROBOT.md). */
    public static double INTAKE_HALF_WIDTH = 5.51;
    // TODO(8): Drive slowly over a POLLEN and measure how far past the intake's front edge it is when the intake grabs it.
    /** A target this far past the intake front counts as picked up, inches. Passes aim 1 in. (MotionModel.ARRIVED_INCHES) deeper. */
    public static double INTAKE_DEPTH = 2;
    /** POLLEN radius, inches (2.8 in. balls, ROBOT.md). */
    public static double BALL_RADIUS = 1.4;
    /** Straight run into each pickup, inches, so the intake meets the targets square. */
    public static double PICKUP_RUN = 12;
    /** The pickup speed limit stays on this far past each pickup, inches (Foresight looks ahead, so it would speed up early). */
    public static double PICKUP_EXIT = 4;
    /**
     * Planning aims to bring each target at least this far inside the intake's sides,
     * inches, to allow for the prediction being off a little. If the predicted path
     * misses (too fast, or off to the side), AgateFlow slows down before that pickup.
     */
    public static double PICKUP_SIDE_MARGIN = 1;
    /** Most groups one pickUp orders (the ordering takes 2^n steps); the nearest are kept. */
    public static int MAX_PICKUP_GROUPS = 12;
    /** How many of the best quick orders are planned in full; the fastest is kept. */
    public static int PICKUP_ORDERS_TRIED = 5;

    private static final double LINE_STEP = 1.0;
    private static final double CORNER_STEP = 0.5;
    /** Closer than this to the target, the robot is already there and only turns. */
    private static final double ARRIVED = 0.5;

    private final FieldMap map;
    private final Follower follower;
    private final VisibilityGraph[] graphs = new VisibilityGraph[4];
    private int nextGraphSlot = 0;

    /**
     * @param follower the robot's follower: its Foresight config gives the brake
     *                 model and enforces speed limits. Null plans with
     *                 DriveModel's fallbacks and no speed limits (for testing).
     */
    public AgateFlow(FieldMap map, Follower follower) {
        this.map = map;
        this.follower = follower;
    }

    public FieldMap map() {
        return map;
    }

    /**
     * Plans the whole route from this pose and velocity.
     *
     * @param velocity     field-frame velocity (follower.velocity())
     * @param alliance     flips the route's red poses for blue
     * @param batteryVolts the hub's battery reading with the drive idle (for the time estimate)
     */
    public Plan plan(Pose start, Velocity velocity, Route route, Alliance alliance, double batteryVolts) {
        return plan(start, velocity, route, alliance, batteryVolts, DriveModel.capture(follower));
    }

    /**
     * Same, with drive settings already read (DriveModel.capture). Use this one off
     * the main thread: capture on the main thread, then plan anywhere. Only one
     * thread at a time may use an AgateFlow.
     */
    public Plan plan(Pose start, Velocity velocity, Route route, Alliance alliance, double batteryVolts, DriveModel model) {
        long began = System.nanoTime();
        FieldMap.Snapshot snap = map.snapshot();
        List<Leg> legs = new ArrayList<>();
        List<String> notes = new ArrayList<>();
        List<Pose> pickedUp = new ArrayList<>(), skipped = new ArrayList<>();
        String problem;
        try {
            problem = planRoute(start, velocity, route, alliance, batteryVolts, model, snap, began, legs, notes, pickedUp, skipped);
        } catch (RuntimeException e) {
            StackTraceElement at = e.getStackTrace().length > 0 ? e.getStackTrace()[0] : null;
            for (StackTraceElement frame : e.getStackTrace()) {
                if (frame.getClassName().startsWith(AgateFlow.class.getPackage().getName())) {
                    at = frame;
                    break;
                }
            }
            problem = "AgateFlow error: " + e + (at == null ? "" : " at " + at.getFileName() + ":" + at.getLineNumber());
        }
        if (problem != null) {
            legs.clear();
            pickedUp.clear();
            skipped.clear();
        }
        return new Plan(problem, legs, notes, (System.nanoTime() - began) / 1e6, snap, pickedUp, skipped);
    }

    // ---- The route: split into legs at stops ----

    private String planRoute(Pose start, Velocity velocity, Route route, Alliance alliance, double volts,
                             DriveModel model, FieldMap.Snapshot snap, long began,
                             List<Leg> legs, List<String> notes, List<Pose> pickedUp, List<Pose> skipped) {
        if (route.isEmpty()) return "The route has no waypoints";
        if (alliance == null) return "No alliance picked, so the route can't be flipped";
        if (bad(start.x()) || bad(start.y()) || bad(start.heading())) return "The robot's pose is NaN";
        if (bad(volts) || volts < 6) volts = 12;

        Job job = new Job();
        job.snap = snap;
        job.model = model;
        job.safe = ROBOT_RADIUS + SAFETY_MARGIN;
        // Search a little wider than the check: even at MIN_SPEED_LIMIT, Foresight's look-ahead cuts
        // inside a tight wrap around a zone corner by about lookAhead^2 / (2 x radius).
        double look = job.model.lookAhead(MIN_SPEED_LIMIT, 0);
        job.tight = job.safe + look * look / (2 * job.safe) + 0.25;
        job.wide = Math.max(job.safe + CORNER_ROOM, job.tight);
        // A third, narrow search: the robot's half width instead of ROBOT_RADIUS, for gaps it can only
        // pass side-on. The outline check on the predicted path (ClearanceRule.slack with a heading)
        // decides if a route found this way is really safe.
        job.narrow = USE_OUTLINE ? job.tight - ROBOT_RADIUS + ROBOT_HALF_WIDTH : job.tight;
        job.volts = volts;
        job.deadline = began + (long) (PLAN_TIME_LIMIT_MS * 1e6);
        job.notes = notes;

        double vx = bad(velocity.vx) ? 0 : velocity.vx, vy = bad(velocity.vy) ? 0 : velocity.vy;
        List<Waypoint.Resolved> targets = new ArrayList<>();
        List<Waypoint> waypoints = route.waypoints();
        boolean anyPickup = false;
        for (int i = 0; i < waypoints.size(); i++) {
            if (waypoints.get(i).pickup != null) {
                // Worked out below, once the waypoints around it are known.
                targets.add(null);
                anyPickup = true;
                continue;
            }
            Waypoint.Resolved target = waypoints.get(i).resolve(alliance);
            if (target == null) return "Waypoint " + (i + 1) + " has no pose (a live pose gave null or NaN, or it is off the field)";
            String problem = moveOutOfZones(target, "Waypoint " + (i + 1), job);
            if (problem != null) return problem;
            if (!Double.isNaN(target.maxSpeed) && job.model.foresight == null) {
                notes.add("Waypoint " + (i + 1) + ": maxSpeed ignored, the follower isn't Foresight");
            }
            targets.add(target);
        }
        if (!anyPickup) return planTargets(job, targets, start, vx, vy, legs);
        return planPickups(job, waypoints, targets, alliance, start, vx, vy, legs, notes, pickedUp, skipped);
    }

    /**
     * Plans the route's targets (every pickUp already turned into passes), leg by leg:
     * a leg runs from a stop (or the start) to the next stop.
     */
    private String planTargets(Job job, List<Waypoint.Resolved> targets, Pose start, double vx, double vy, List<Leg> legs) {
        List<String> notes = job.notes;
        // A through-waypoint where the route turns back can't be driven through at speed: stop there.
        for (int i = 0; i < targets.size() - 1; i++) {
            Waypoint.Resolved w = targets.get(i);
            if (w.stop) continue;
            double px = i == 0 ? start.x() : targets.get(i - 1).x, py = i == 0 ? start.y() : targets.get(i - 1).y;
            Waypoint.Resolved next = targets.get(i + 1);
            if (VisibilityGraph.turnCost(w.x - px, w.y - py, next.x - w.x, next.y - w.y) > 1 - Math.cos(Math.toRadians(120))) {
                w.stop = true;
                notes.add("Waypoint " + (i + 1) + " turns back sharply, so the robot stops there");
            }
        }

        double x = start.x(), y = start.y(), heading = start.heading();
        int first = 0;
        while (first < targets.size()) {
            int last = first;
            while (last < targets.size() - 1 && !targets.get(last).stop) last++;
            job.startX = x;
            job.startY = y;
            job.startHeading = heading;
            job.startVx = vx;
            job.startVy = vy;
            job.startFlowerDown = first > 0 && targets.get(first - 1).flowerDown;
            job.targets = targets.subList(first, last + 1);
            int legCount = legs.size();
            String problem = planLeg(job, legs);
            if (problem == null && legs.size() == legCount) {
                // planLeg made a through-waypoint a stop: plan this leg again, up to it.
                notes.add("A through-waypoint couldn't be driven through smoothly, so the robot stops there");
                continue;
            }
            if (problem != null) return problem;
            Leg leg = legs.get(legs.size() - 1);
            x = leg.end.x();
            y = leg.end.y();
            heading = leg.end.heading();
            vx = 0;
            vy = 0;
            first = last + 1;
        }
        return null;
    }

    // ---- Picking up (Route.pickUp) ----

    /**
     * Plans a route with pickUps: the first pickUp's best orders by quick time in full
     * (see worthTrying), keeping the fastest; any later pickUp takes its best quick
     * order. Then the capture test (Pickup.capturedAt) on the chosen plan.
     */
    private String planPickups(Job job, List<Waypoint> waypoints, List<Waypoint.Resolved> resolved, Alliance alliance,
                               Pose start, double vx, double vy, List<Leg> legs, List<String> notes,
                               List<Pose> pickedUp, List<Pose> skipped) {
        Map<String, PickupOrders> known = new HashMap<>();
        List<double[]> everyTarget = new ArrayList<>();
        List<Integer> mostAllowed = new ArrayList<>();
        List<Leg> bestLegs = null;
        List<String> bestNotes = null;
        double bestSeconds = Double.POSITIVE_INFINITY;
        String firstProblem = null;
        for (int choice = 0; choice < 2 * Math.max(1, PICKUP_ORDERS_TRIED); choice++) {
            // Out of time: stop once one order has worked (if none has, keep trying the next best).
            if (bestLegs != null && System.nanoTime() > job.deadline) break;
            List<String> tryNotes = new ArrayList<>(notes);
            job.notes = tryNotes;
            List<Waypoint.Resolved> targets = expand(job, waypoints, resolved, alliance, start, vx, vy, choice, known,
                    choice == 0 ? everyTarget : null, choice == 0 ? mostAllowed : null, bestSeconds);
            if (targets == null) break;
            if (targets.isEmpty()) {
                firstProblem = "Nothing to pick up, and no other waypoints";
                break;
            }
            List<Leg> tryLegs = new ArrayList<>();
            String problem = planTargets(job, targets, start, vx, vy, tryLegs);
            if (problem != null) {
                if (firstProblem == null) firstProblem = problem;
                continue;
            }
            double seconds = 0;
            for (Leg leg : tryLegs) seconds += leg.seconds;
            if (seconds < bestSeconds) {
                bestSeconds = seconds;
                bestLegs = tryLegs;
                bestNotes = tryNotes;
            }
        }
        job.notes = notes;
        if (bestLegs == null) return firstProblem != null ? firstProblem : "No way to pick up the targets";
        notes.clear();
        notes.addAll(bestNotes);
        legs.addAll(bestLegs);

        // The capture test: which targets the predicted path really drives into the intake, in order.
        List<double[]> got = new ArrayList<>();
        for (double[] target : everyTarget) {
            int when = -1;
            for (int l = 0; l < legs.size() && when < 0; l++) {
                Leg leg = legs.get(l);
                if (leg.predictedHeading.length == 0) continue;
                int at = Pickup.capturedAt(target[0], target[1], leg.predictedPath, leg.predictedHeading, leg.predictedSpeed, 0);
                if (at >= 0) when = l * 100000 + at;
            }
            if (when < 0) skipped.add(new Pose(target[0], target[1], 0));
            else got.add(new double[]{when, target[0], target[1]});
        }
        Collections.sort(got, (a, b) -> Double.compare(a[0], b[0]));
        for (double[] g : got) pickedUp.add(new Pose(g[1], g[2], 0));
        int allowed = 0;
        for (int most : mostAllowed) allowed = most == Integer.MAX_VALUE || allowed == Integer.MAX_VALUE ? Integer.MAX_VALUE : allowed + most;
        if (pickedUp.size() > allowed) {
            notes.add(String.format("The path drives over %d targets, more than most() allows (%d): it passes over ones it didn't pick",
                    pickedUp.size(), allowed));
        }
        return null;
    }

    /**
     * Whether to plan the {@code pick}-th best order in full, given the fastest full
     * plan so far ({@code bestSeconds}, +infinity if none worked): any order while none
     * has worked; up to PICKUP_ORDERS_TRIED orders within 15% of the best quick time;
     * and, up to twice as many, any order that could still win if the plans so far came
     * out unusually slow for their quick time (a pass next to a zone that the quick time
     * can't see, say). Full plans take about 1.5x the quick time.
     */
    private static boolean worthTrying(List<Pickup.Order> orders, int pick, double bestSeconds) {
        if (Double.isInfinite(bestSeconds)) return true;
        double quick = orders.get(pick).seconds;
        if (pick < Math.max(1, PICKUP_ORDERS_TRIED) && quick <= 1.15 * orders.get(0).seconds) return true;
        return 1.5 * quick < bestSeconds;
    }

    /** One pickUp's groups, quick hop times and best orders, from one spot. */
    private static final class PickupOrders {
        List<Pickup.Group> groups;
        List<double[]> targets;
        Pickup.Hops hops;
        List<Pickup.Order> orders;
    }

    /**
     * The route's targets with every pickUp turned into passes: the first pickUp in
     * its {@code choice}-th best order (null if it has no such order), any later one in
     * its best. Fresh copies, since planning changes them. The first time (choice 0),
     * fills everyTarget with every pickUp target and mostAllowed with each most().
     * Also null when the order isn't worth planning in full (see worthTrying).
     */
    private List<Waypoint.Resolved> expand(Job job, List<Waypoint> waypoints, List<Waypoint.Resolved> resolved,
                                           Alliance alliance, Pose start, double vx, double vy, int choice,
                                           Map<String, PickupOrders> known, List<double[]> everyTarget, List<Integer> mostAllowed,
                                           double bestSeconds) {
        List<Waypoint.Resolved> out = new ArrayList<>();
        boolean first = true;
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint w = waypoints.get(i);
            if (w.pickup == null) {
                out.add(resolved.get(i).copy());
                continue;
            }
            Waypoint.Resolved from = out.isEmpty() ? null : out.get(out.size() - 1);
            double fromX = from == null ? start.x() : from.x, fromY = from == null ? start.y() : from.y;
            double fromSpeed = from == null ? Geometry.length(vx, vy) : 0;
            Waypoint.Resolved next = i + 1 < waypoints.size() ? resolved.get(i + 1) : null;
            String key = i + "@" + fromX + "," + fromY;
            PickupOrders p = known.get(key);
            if (p == null) {
                p = pickupOrders(job, w.pickup, w.pickup.resolve(alliance), fromX, fromY, fromSpeed, next);
                known.put(key, p);
            }
            if (everyTarget != null) everyTarget.addAll(p.targets);
            if (mostAllowed != null) mostAllowed.add(w.pickup.most);
            int pick = first ? choice : 0;
            first = false;
            if (pick >= p.orders.size()) {
                if (pick > 0) return null;
                job.notes.add("pickUp " + (i + 1) + ": nothing it can pick up");
                continue;
            }
            if (pick > 0 && !worthTrying(p.orders, pick, bestSeconds)) return null;
            int[] order = p.orders.get(pick).groups;
            boolean routeEnds = i == waypoints.size() - 1;
            int previous = 0;
            for (int n = 0; n < order.length; n++) {
                int g = order[n];
                Waypoint.Resolved pass = pass(job, w, alliance, p, previous, g, routeEnds && n == order.length - 1);
                if (pass != null) out.add(pass);
                previous = g + 1;
            }
        }
        return out;
    }

    /**
     * Groups, quick hop times and the best orders for one pickUp, from (fromX, fromY)
     * on to {@code next} (or stopping at the last group if null). See Pickup.
     */
    private PickupOrders pickupOrders(Job job, Pickup pickup, List<double[]> targets, double fromX, double fromY,
                                      double fromSpeed, Waypoint.Resolved next) {
        PickupOrders p = new PickupOrders();
        p.targets = targets;
        List<Pickup.Group> groups = Pickup.groups(targets, 2 * (INTAKE_HALF_WIDTH - BALL_RADIUS), pickup.inOrder);
        if (groups.size() > MAX_PICKUP_GROUPS) {
            if (!pickup.inOrder) {
                final double ox = fromX, oy = fromY;
                Collections.sort(groups, (a, b) -> Double.compare(Geometry.length(a.x - ox, a.y - oy), Geometry.length(b.x - ox, b.y - oy)));
            }
            job.notes.add(String.format("pickUp: %d groups of targets, only the %s %d are ordered", groups.size(),
                    pickup.inOrder ? "first" : "nearest", MAX_PICKUP_GROUPS));
            groups = new ArrayList<>(groups.subList(0, MAX_PICKUP_GROUPS));
        }
        p.groups = groups;
        int k = groups.size();
        boolean hasNext = next != null;
        double[] px = new double[k + 2], py = new double[k + 2];
        px[0] = fromX;
        py[0] = fromY;
        for (int g = 0; g < k; g++) {
            px[g + 1] = groups.get(g).x;
            py[g + 1] = groups.get(g).y;
        }
        if (hasNext) {
            px[k + 1] = next.x;
            py[k + 1] = next.y;
        }
        Pickup.Hops hops = new Pickup.Hops(k + 2);
        double top = job.model.forwardPerVolt * job.volts * 0.9;
        double accel = Math.min(job.model.traction, top / (2 * job.model.forwardTau));
        double through = Math.min(PICKUP_SPEED, top);
        for (int a = 0; a <= k; a++) {
            for (int b = 1; b <= k + 1; b++) {
                hops.seconds[a][b] = Double.POSITIVE_INFINITY;
                if (a == b || (b == k + 1 && !hasNext)) continue;
                double[] way = hopPath(job, px[a], py[a], px[b], py[b]);
                if (way == null) continue;
                double length = 0;
                for (int q = 0; q + 3 < way.length; q += 2) length += Geometry.length(way[q + 2] - way[q], way[q + 3] - way[q + 1]);
                if (length < 1e-6) continue;
                int e = way.length;
                double inX = way[e - 2] - way[e - 4], inY = way[e - 1] - way[e - 3], outX = way[2] - way[0], outY = way[3] - way[1];
                double in = Math.max(Geometry.length(inX, inY), 1e-9), out = Math.max(Geometry.length(outX, outY), 1e-9);
                hops.inX[a][b] = inX / in;
                hops.inY[a][b] = inY / in;
                hops.outX[a][b] = outX / out;
                hops.outY[a][b] = outY / out;
                hops.length[a][b] = length;
                double from = a == 0 ? fromSpeed : through, to = b == k + 1 && next.stop ? 0 : through;
                hops.seconds[a][b] = Pickup.hopSeconds(length, from, to, top, accel);
            }
        }
        p.hops = hops;
        p.orders = Pickup.orders(groups, hops, hasNext, pickup.most, pickup.inOrder, through, accel,
                job.volts * job.model.turnPerVolt, 2 * Math.max(1, PICKUP_ORDERS_TRIED));
        return p;
    }

    /** Shortest way from a to b around the zones (straight if it can be), or null if there is none. */
    private double[] hopPath(Job job, double ax, double ay, double bx, double by) {
        ClearanceRule rule = new ClearanceRule(job.snap.zones, job.safe, ESCAPE_SLOPE, ax, ay, bx, by);
        if (rule.lineClear(ax, ay, bx, by, 1)) return new double[]{ax, ay, bx, by};
        for (double radius : searchRadii(rule, job)) {
            double[] way = findPath(job, radius, ax, ay, null, 0, bx, by, null, 0, rule, 0);
            if (way != null) return way;
        }
        return null;
    }

    /**
     * The pass for group g of a pickUp, coming from hop point {@code previous} (0: the
     * start): the intake front INTAKE_DEPTH past the group's farthest target along the
     * way in, centered on the group across it. Null if it can't be placed.
     */
    private Waypoint.Resolved pass(Job job, Waypoint pickUp, Alliance alliance, PickupOrders p, int previous, int g, boolean stop) {
        Pickup.Group group = p.groups.get(g);
        double dx = p.hops.inX[previous][g + 1], dy = p.hops.inY[previous][g + 1];
        double sx = -dy, sy = dx;
        double farthest = -Double.MAX_VALUE, left = -Double.MAX_VALUE, right = Double.MAX_VALUE;
        for (int m : group.members) {
            double[] t = p.targets.get(m);
            double ox = t[0] - group.x, oy = t[1] - group.y;
            farthest = Math.max(farthest, ox * dx + oy * dy);
            double side = ox * sx + oy * sy;
            left = Math.max(left, side);
            right = Math.min(right, side);
        }
        // A stop may settle up to ARRIVED_INCHES short (and a pass may yet become a stop), so aim that much deeper.
        double depth = INTAKE_DEPTH + MotionModel.ARRIVED_INCHES;
        double across = (left + right) / 2, along = farthest + depth - ROBOT_FRONT;
        double cx = group.x + across * sx + along * dx, cy = group.y + across * sy + along * dy;
        double[] members = new double[2 * group.count()];
        for (int i = 0; i < group.count(); i++) {
            members[2 * i] = p.targets.get(group.members.get(i))[0];
            members[2 * i + 1] = p.targets.get(group.members.get(i))[1];
        }
        Waypoint waypoint = Waypoint.pass(new Pose(cx, cy, Math.atan2(dy, dx)), stop, pickUp.maxSpeed, pickUp.contact,
                PICKUP_SPEED, members);
        Waypoint.Resolved target = waypoint.resolve(alliance);
        if (target == null) return null;
        if (USE_OUTLINE && !pickUp.contact) {
            // The intake reaching the targets would put the robot into something (targets against
            // the HIVE frame or a wall, say): it can't pick them up, so don't drive there.
            Zone into = nearestToOutline(job.snap, cx, cy, Math.atan2(dy, dx));
            if (into != null && into.outlineDistance(cx, cy, Math.atan2(dy, dx), ROBOT_FRONT, ROBOT_BACK, ROBOT_HALF_WIDTH) < 0) {
                job.notes.add(String.format("A pickup near (%.0f, %.0f) is too close to %s for the intake to reach it, so it is left out",
                        group.x, group.y, into.name));
                return null;
            }
        }
        String problem = moveOutOfZones(target, String.format("The pickup pass near (%.0f, %.0f)", group.x, group.y), job);
        if (problem != null) {
            job.notes.add(String.format("A pickup near (%.0f, %.0f) is inside a zone, so it is left out", group.x, group.y));
            return null;
        }
        if (target.x != cx || target.y != cy) {
            // Moved out of a zone: keep the moved spot, so a replan drives the same pass.
            waypoint = Waypoint.pass(new Pose(target.x, target.y, Math.atan2(dy, dx)), stop, pickUp.maxSpeed, pickUp.contact,
                    PICKUP_SPEED, members);
            target = waypoint.resolve(alliance);
        }
        return target;
    }

    /** What one leg's planning needs. */
    private static final class Job {
        FieldMap.Snapshot snap;
        DriveModel model;
        /** Clearance checked; radius of the tight search; radius of the wide search. */
        double safe, tight, wide, narrow, volts;
        /** The grown-zone radius each candidate route was found with (its corners are rounded within that). */
        final java.util.Map<double[], Double> foundRadius = new java.util.IdentityHashMap<>();
        long deadline;
        List<String> notes;
        double startX, startY, startHeading, startVx, startVy;
        /** The leg starts where a Route.flowerIntakeDown stretch ended (the intake is still coming up). */
        boolean startFlowerDown;
        List<Waypoint.Resolved> targets;
    }

    /**
     * Moves a target out of (or just away from) a zone, then, if it has a heading of
     * its own, until the robot's outline fits there (see fitOutline), with a note.
     * {@code label} names it in notes and problems.
     */
    private String moveOutOfZones(Waypoint.Resolved target, String label, Job job) {
        String problem = moveCenterOut(target, label, job);
        return problem != null ? problem : fitOutline(target, label, job);
    }

    /** Moves a target's center out of (or just away from) a zone, with a note. */
    private String moveCenterOut(Waypoint.Resolved target, String label, Job job) {
        double room = job.snap.clearance(target.x, target.y);
        // A stop just outside the robot's own radius from a zone leaves no room to settle:
        // the robot may drift ClearanceRule.END_SLACK past a stop, but not into that radius.
        // Nudge it out that far (less than END_SLACK).
        double own = target.contact ? CONTACT_RADIUS : ROBOT_RADIUS;
        if (room >= own && room < own + ClearanceRule.END_SLACK) {
            double x = target.x, y = target.y;
            for (int k = 0; k < 10 && job.snap.clearance(x, y) < own + ClearanceRule.END_SLACK; k++) {
                double[] away = escapeDirection(job.snap, x, y);
                double step = own + ClearanceRule.END_SLACK + 0.02 - job.snap.clearance(x, y);
                x += step * away[0];
                y += step * away[1];
            }
            if (job.snap.clearance(x, y) >= own + ClearanceRule.END_SLACK) {
                job.notes.add(String.format("%s moved %.2f in. away from %s, so the robot has room to settle there", label,
                        Geometry.length(x - target.x, y - target.y), job.snap.nearest(target.x, target.y).name));
                target.x = x;
                target.y = y;
            }
            return null;
        }
        if (room >= CONTACT_RADIUS) return null;
        Zone zone = job.snap.nearest(target.x, target.y);
        String name = zone == null ? "a zone" : zone.name;
        if (!MOVE_TARGETS_OUT_OF_ZONES) return label + " is inside or against " + name;
        double x = target.x, y = target.y;
        for (int k = 0; k < 160; k++) {
            double[] away = escapeDirection(job.snap, x, y);
            x += 0.25 * away[0];
            y += 0.25 * away[1];
            if (job.snap.clearance(x, y) >= CONTACT_RADIUS + 0.25) {
                job.notes.add(String.format("%s moved %.1f in. out of %s", label,
                        Geometry.length(x - target.x, y - target.y), name));
                target.x = x;
                target.y = y;
                return null;
            }
        }
        return label + " is inside " + name + " and couldn't be moved out";
    }

    /**
     * The heading the robot has at a target at (x, y) if the route fixes it: facing a
     * point (Heading.facing), else the target's own heading. NaN if it has none.
     */
    private static double fixedHeading(Waypoint.Resolved target, double x, double y) {
        if (target.turnRule.kind == Heading.Kind.FACING) return Math.atan2(target.turnRule.y - y, target.turnRule.x - x);
        return target.heading;
    }

    /**
     * A target with a heading of its own: if the robot's outline there, facing that
     * heading, overlaps a zone or a wall, moves the target out until the outline is
     * ClearanceRule.END_SLACK clear of it (room to settle), with a note. The robot
     * can't be there, so a plan to it would end in something. Not for the flower intake into a FLOWER (Route.flowerIntakeDown
     * reaches in on purpose) or allowContact (the circle rule). A target without a
     * heading is given one that fits when its leg is planned.
     */
    private String fitOutline(Waypoint.Resolved target, String label, Job job) {
        double heading = fixedHeading(target, target.x, target.y);
        if (Double.isNaN(heading) || target.flowerDown || target.contact || !USE_OUTLINE) return null;
        ClearanceRule rule = new ClearanceRule(job.snap.zones, ROBOT_RADIUS, 0, target.x, target.y, target.x, target.y);
        double room = rule.outlineRoom(target.x, target.y, heading);
        if (room >= 0) return null;
        Zone zone = nearestToOutline(job.snap, target.x, target.y, heading);
        String name = zone == null ? "a zone" : zone.name;
        if (!MOVE_TARGETS_OUT_OF_ZONES) return String.format("%s puts the robot %.1f in. into %s", label, -room, name);
        double x = target.x, y = target.y;
        double clear = ClearanceRule.END_SLACK;
        for (int k = 0; k < 40 && room < clear; k++) {
            // Straight out of the zone it is closest to, by how much it is short (and a hair).
            double h = 0.05;
            double gx = rule.outlineRoom(x + h, y, fixedHeading(target, x + h, y)) - rule.outlineRoom(x - h, y, fixedHeading(target, x - h, y));
            double gy = rule.outlineRoom(x, y + h, fixedHeading(target, x, y + h)) - rule.outlineRoom(x, y - h, fixedHeading(target, x, y - h));
            double length = Geometry.length(gx, gy);
            if (length < 1e-9) break;
            x += gx / length * (clear - room + 0.02);
            y += gy / length * (clear - room + 0.02);
            room = rule.outlineRoom(x, y, fixedHeading(target, x, y));
        }
        if (room < clear) return String.format("%s puts the robot %.1f in. into %s and couldn't be moved out", label, -rule.outlineRoom(target.x, target.y, heading), name);
        job.notes.add(String.format("%s moved %.1f in. so the robot fits there facing %.0f degrees (it was into %s)", label,
                Geometry.length(x - target.x, y - target.y), Math.toDegrees(fixedHeading(target, x, y)), name));
        target.x = x;
        target.y = y;
        return null;
    }

    /** The zone (or wall) the robot's outline at (x, y) facing {@code heading} is closest to (or deepest into). */
    private static Zone nearestToOutline(FieldMap.Snapshot snap, double x, double y, double heading) {
        Zone best = null;
        double least = Double.POSITIVE_INFINITY;
        for (Zone zone : snap.zones) {
            double d = zone.outlineDistance(x, y, heading, ROBOT_FRONT, ROBOT_BACK, ROBOT_HALF_WIDTH);
            if (d < least) {
                least = d;
                best = zone;
            }
        }
        return best;
    }

    /** Unit direction in which the room to the nearest zone grows fastest. */
    private static double[] escapeDirection(FieldMap.Snapshot snap, double x, double y) {
        double h = 0.25;
        double gx = snap.clearance(x + h, y) - snap.clearance(x - h, y);
        double gy = snap.clearance(x, y + h) - snap.clearance(x, y - h);
        double length = Geometry.length(gx, gy);
        if (length < 1e-9) {
            gx = Field.SIZE / 2 - x;
            gy = Field.SIZE / 2 - y;
            length = Math.max(Geometry.length(gx, gy), 1e-9);
        }
        return new double[]{gx / length, gy / length};
    }

    // ---- One leg ----

    private String planLeg(Job job, List<Leg> legs) {
        List<Waypoint.Resolved> targets = job.targets;
        int k = targets.size();
        DriveModel model = job.model;

        Waypoint.Resolved stop = targets.get(k - 1);
        if (k == 1 && Geometry.length(stop.x - job.startX, stop.y - job.startY) < ARRIVED) {
            Leg turn = turnInPlace(job, stop);
            if (turn == null) {
                return String.format("Turning in place at (%.0f, %.0f) would swing the robot's corners into %s", stop.x, stop.y,
                        nearestToOutline(job.snap, stop.x, stop.y, job.startHeading).name);
            }
            legs.add(turn);
            return null;
        }

        double[] ax = new double[k], ay = new double[k];
        for (int j = 0; j < k; j++) {
            ax[j] = j == 0 ? job.startX : targets.get(j - 1).x;
            ay[j] = j == 0 ? job.startY : targets.get(j - 1).y;
        }
        ClearanceRule[] rules = new ClearanceRule[k];
        for (int j = 0; j < k; j++) {
            if (Geometry.length(targets.get(j).x - ax[j], targets.get(j).y - ay[j]) < ARRIVED) {
                return "Waypoints " + j + " and " + (j + 1) + " of this leg are at the same spot";
            }
            double radius = targets.get(j).contact ? CONTACT_RADIUS : job.safe;
            // The first stretch starts where the robot is, facing the way it really faces (so turning in place counts).
            // The flower intake down (Route.flowerIntakeDown) reaches further ahead on that stretch.
            // Only the flower intake is meant to reach into something (a FLOWER's opening): no other end may overlap a zone.
            Waypoint.Resolved target = targets.get(j);
            double front = target.flowerDown ? IntakePoses.FLOWER_INTAKE_REACH : ROBOT_FRONT;
            rules[j] = new ClearanceRule(job.snap.zones, radius, ESCAPE_SLOPE, ax[j], ay[j], target.x, target.y,
                    j == 0 ? job.startHeading : Double.NaN, target.flowerDown, front, target.contact);
            // Leaving a flower intake stop or stretch: the intake is still coming up for a while.
            boolean afterFlower = j > 0 ? targets.get(j - 1).flowerDown : job.startFlowerDown;
            if (afterFlower && !target.flowerDown) {
                rules[j] = rules[j].withStartFront(IntakePoses.FLOWER_INTAKE_REACH, FLOWER_INTAKE_RAISE_RUN);
            }
        }

        // Already moving fast: start along the way it's going, for about as far as it would take to stop.
        double speed = Geometry.length(job.startVx, job.startVy);
        double[] moveDir = null;
        double moveRun = 0, startSpeed = 0;
        if (speed > MOVING_SPEED) {
            moveDir = new double[]{job.startVx / speed, job.startVy / speed};
            double angle = Math.atan2(job.startVy, job.startVx) - job.startHeading;
            moveRun = Math.max(3, Math.min(36, model.lookAhead(speed, angle)));
            startSpeed = speed;
        } else if (job.startFlowerDown) {
            // At a flower intake stop the intake is in the FLOWER and still coming up: back straight out first.
            moveDir = new double[]{-Math.cos(job.startHeading), -Math.sin(job.startHeading)};
            moveRun = FLOWER_INTAKE_RAISE_RUN;
        }

        // Direction through each through-waypoint: halfway between the way in and the way out.
        double[][] passDir = new double[k][];
        double[] passRun = new double[k], passRunOut = new double[k];
        if (k > 1) {
            double[][] rough = new double[k][];
            for (int j = 0; j < k; j++) {
                for (double radius : searchRadii(rules[j], job)) {
                    if (rough[j] != null) break;
                    rough[j] = findPath(job, radius, ax[j], ay[j], j == 0 ? moveDir : null, moveRun,
                            targets.get(j).x, targets.get(j).y, null, 0, rules[j], TURN_WEIGHT);
                }
                if (rough[j] == null) return noWay(ax[j], ay[j], targets.get(j));
            }
            for (int j = 0; j < k - 1; j++) {
                Waypoint.Resolved w = targets.get(j);
                double[] in = rough[j], out = rough[j + 1];
                double inX = w.x - in[in.length - 4], inY = w.y - in[in.length - 3];
                double outX = out[2] - w.x, outY = out[3] - w.y;
                double inLength = Geometry.length(inX, inY), outLength = Geometry.length(outX, outY);
                double dx, dy;
                if (!Double.isNaN(w.approachAngle)) {
                    dx = Math.cos(w.approachAngle);
                    dy = Math.sin(w.approachAngle);
                } else {
                    dx = inX / inLength + outX / outLength;
                    dy = inY / inLength + outY / outLength;
                    if (Geometry.length(dx, dy) < 0.2) {
                        dx = outX / outLength;
                        dy = outY / outLength;
                    }
                }
                double length = Geometry.length(dx, dy);
                passDir[j] = new double[]{dx / length, dy / length};
                double run = w.approachLength > 0 ? w.approachLength : PASS_RUN;
                passRun[j] = Math.max(2, Math.min(run, Math.min(0.4 * inLength, 0.4 * outLength)));
                passRunOut[j] = passRun[j];
                if (w.targets != null) {
                    // A pickup: a long straight run in, so the intake meets the targets square; a short one out.
                    passRun[j] = Math.max(2, Math.min(run, 0.7 * inLength));
                    passRunOut[j] = Math.max(2, Math.min(PICKUP_EXIT + 2, 0.4 * outLength));
                }
            }
        }

        // How the last stop is approached.
        double[] stopDir = null;
        double stopRun = 0;
        if (!Double.isNaN(stop.approachAngle)) {
            stopDir = new double[]{Math.cos(stop.approachAngle), Math.sin(stop.approachAngle)};
            stopRun = Math.max(stop.approachLength, 2);
        } else if (!Double.isNaN(stop.heading)
                && (stop.turnRule.kind == Heading.Kind.TANGENT || stop.turnRule.kind == Heading.Kind.REVERSE_TANGENT)) {
            // Facing the way it drives and asked to end at a heading: arrive driving that way.
            // Without a heading (Route.to(x, y)) it may arrive from any direction.
            double travel = stop.heading + (stop.turnRule.kind == Heading.Kind.REVERSE_TANGENT ? Math.PI : 0);
            stopDir = new double[]{Math.cos(travel), Math.sin(travel)};
            stopRun = TANGENT_APPROACH;
        }

        // Each stretch in turn: try the candidates, keep the fastest safe one.
        List<Stretch> chosen = new ArrayList<>();
        Evaluation best = null;
        for (int j = 0; j < k; j++) {
            // Leave a through-waypoint the way the previous stretch really arrived, so there's no kink
            // there even if that stretch couldn't keep the planned direction.
            double[] startDir = j == 0 ? moveDir : chosen.get(j - 1).endDirection();
            double startRun = j == 0 ? moveRun : passRunOut[j - 1];
            double[] endDir = j < k - 1 ? passDir[j] : stopDir;
            double endRun = j < k - 1 ? passRun[j] : stopRun;
            Waypoint.Resolved target = targets.get(j);

            List<double[]> candidates = candidates(job, ax[j], ay[j], startDir, startRun,
                    target.x, target.y, endDir, endRun, rules[j]);
            if (candidates.isEmpty()) return noWay(ax[j], ay[j], target);

            Choice choice = choose(candidates, chosen, j, rules[j], target, job, startSpeed);
            if (j == 0 && moveDir != null && startSpeed > 0) {
                // Moving: braking and turning back can beat carrying on (the simulation includes the
                // momentum and the overshoot), so try routes that don't start the way it's going too.
                List<double[]> turnBack = candidates(job, ax[j], ay[j], null, 0, target.x, target.y, endDir, endRun, rules[j]);
                for (double[] tried : candidates) turnBack.removeIf(polyline -> Arrays.equals(polyline, tried));
                if (!turnBack.isEmpty()) {
                    Choice other = choose(turnBack, chosen, j, rules[j], target, job, startSpeed);
                    if (other.best != null && (choice.best == null || other.best.profile.seconds < choice.best.profile.seconds)) {
                        choice = other;
                    }
                }
            }
            if (choice.best == null && (startDir != null || endDir != null)) {
                // Every route keeping the asked direction is too tight: try the ones that don't.
                List<double[]> loose = candidates(job, ax[j], ay[j], null, 0, target.x, target.y, null, 0, rules[j]);
                for (double[] tried : candidates) loose.removeIf(polyline -> Arrays.equals(polyline, tried));
                if (!loose.isEmpty()) {
                    Choice second = choose(loose, chosen, j, rules[j], target, job, startSpeed);
                    if (second.best != null || choice.closest == null) choice = second;
                    if (second.best != null) {
                        job.notes.add(String.format("Couldn't keep the asked direction near (%.0f, %.0f)", target.x, target.y));
                    }
                }
            }
            if (choice.best == null) acceptNearMiss(choice, job, startSpeed);
            if (choice.best == null && j > 0 && !targets.get(j - 1).stop
                    && job.snap.clearance(targets.get(j - 1).x, targets.get(j - 1).y) >= ROBOT_RADIUS + ClearanceRule.END_SLACK) {
                // No safe way on from the through-waypoint before this stretch at speed: stop there
                // instead and let planRoute redo the leg. Only where there's room to turn in place.
                targets.get(j - 1).stop = true;
                return null;
            }
            if (choice.best == null) return tooTight(target, choice.closest, job);
            if (choice.before != null) chosen.set(j - 1, choice.before);
            chosen.add(choice.stretch);
            best = choice.best;
            if (j > 0) {
                double[] in = chosen.get(j - 1).endDirection(), out = choice.stretch.startDirection();
                if (in[0] * out[0] + in[1] * out[1] < Math.cos(Math.toRadians(20))) {
                    // Still a sharp kink at the through-waypoint: make it a stop and let planRoute redo the leg.
                    targets.get(j - 1).stop = true;
                    return null;
                }
            }
        }
        legs.add(buildLeg(best, job));
        return null;
    }

    /** The best candidate for one stretch, or the closest miss if none was safe. */
    private static final class Choice {
        Evaluation best, closest, quickestNearMiss;
        Stretch stretch, before;
    }

    /**
     * Rounds each candidate's corners and ranks them by a quick time (no speed
     * limits, no predicted path: a lower bound, since speed limits only add time).
     * Then checks them fully, fastest first, while one could still win.
     */
    private Choice choose(List<double[]> candidates, List<Stretch> chosen, int j, ClearanceRule rule,
                          Waypoint.Resolved target, Job job, double startSpeed) {
        int count = candidates.size();
        Stretch[] shapes = new Stretch[count], befores = new Stretch[count];
        final double[] quick = new double[count];
        Integer[] order = new Integer[count];
        for (int c = 0; c < count; c++) {
            order[c] = c;
            quick[c] = Double.POSITIVE_INFINITY;
            Double found = job.foundRadius.get(candidates.get(c));
            double sizing = found != null ? Math.min(found, job.tight) : job.tight;
            if (rule.contact) sizing = rule.radius + 0.25;
            // Backing out of a FLOWER with the flower intake still coming up: straight back until the intake is
            // out of the opening (a rounded corner there would slide it sideways), and no turning for a while.
            boolean leavingFlower = j == 0 && job.startFlowerDown;
            double straight = leavingFlower ? IntakePoses.FLOWER_REACH_INTO_OPENING + 1 : 0;
            shapes[c] = smooth(candidates.get(c), rule.withRadius(sizing), rule, target, j > 0 ? job.targets.get(j - 1) : null, straight);
            if (shapes[c] == null) continue;
            if (leavingFlower) shapes[c].holdStartAtLeast = FLOWER_INTAKE_RAISE_RUN;
            befores[c] = j > 0 ? chosen.get(j - 1).copy() : null;
            if (befores[c] != null) roundJoint(befores[c], shapes[c]);
            quick[c] = quickSeconds(draft(chosen, befores[c], shapes[c]), job, startSpeed);
        }
        Arrays.sort(order, (a, b) -> Double.compare(quick[a], quick[b]));

        Choice choice = new Choice();
        for (int c : order) {
            if (shapes[c] == null) continue;
            // Model times can come out a few % lower along the predicted path than the planned one.
            if (choice.best != null && (quick[c] * 0.97 > choice.best.profile.seconds || System.nanoTime() > job.deadline)) break;
            Evaluation e = evaluate(draft(chosen, befores[c], shapes[c]), job, startSpeed);
            Stretch shape = shapes[c];
            if (!e.safe && Double.isNaN(target.heading) && target.turnRule.kind == Heading.Kind.TURN) {
                // No heading asked for here: try facing other ways (side-on along a wall, say) before
                // giving up on this way. A heading the route asks for is never changed.
                double[] line = candidates.get(c);
                double way = Math.atan2(line[line.length - 1] - line[1], line[line.length - 2] - line[0]);
                for (double facing : headingChoices(way, job.startHeading)) {
                    Stretch turned = shapes[c].copy();
                    turned.chosenHeading = facing;
                    Evaluation tried = evaluate(draft(chosen, befores[c], turned), job, startSpeed);
                    if (tried.safe && (!e.safe || tried.profile.seconds < e.profile.seconds)) {
                        e = tried;
                        shape = turned;
                    }
                }
            }
            e.shape = shape;
            e.before = befores[c];
            if (e.safe && (choice.best == null || e.profile.seconds < choice.best.profile.seconds)) {
                choice.best = e;
                choice.stretch = shape;
                choice.before = befores[c];
            }
            if (!e.safe && (choice.closest == null || e.worst() > choice.closest.worst())) choice.closest = e;
            if (e.nearMiss != null && (choice.quickestNearMiss == null || e.nearMissSeconds < choice.quickestNearMiss.nearMissSeconds)) {
                choice.quickestNearMiss = e;
            }
        }
        return choice;
    }

    /**
     * Headings to try for a waypoint without one, when keeping the start heading
     * doesn't fit: facing the way ({@code way}, radians), side-on to it either way,
     * and backward, the smallest turns first.
     */
    private static double[] headingChoices(double way, double startHeading) {
        double[] choices = {way, way + Math.PI / 2, way - Math.PI / 2, way + Math.PI};
        Double[] order = {choices[0], choices[1], choices[2], choices[3]};
        Arrays.sort(order, (a, b) -> Double.compare(Math.abs(Angles.wrapRadians(a - startHeading)), Math.abs(Angles.wrapRadians(b - startHeading))));
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) out[i] = Angles.wrapRadians(order[i]);
        return out;
    }

    /**
     * No candidate kept the whole SAFETY_MARGIN. Candidates are smoothed right up
     * to the limit, so the closest one often misses by only a tenth of an inch.
     * Takes it if it misses by less than NEAR_MISS_FRACTION of the margin and its
     * predicted path still keeps the rule without the margin (ROBOT_RADIUS, or
     * CONTACT_RADIUS on an allowContact stretch, which has no margin to give).
     * Tries the quickest candidate that got that close first, with the speed limits
     * it had then (slowing down more makes a path a little safer but can make it
     * much slower). Then the closest candidate, with all its speed limits.
     */
    private void acceptNearMiss(Choice choice, Job job, double startSpeed) {
        Evaluation from = choice.quickestNearMiss;
        Evaluation near = from == null ? null : nearMiss(from.nearMiss, job, startSpeed);
        if (near == null && choice.closest != null) {
            from = choice.closest;
            near = nearMiss(from.stretches, job, startSpeed);
        }
        if (near == null) return;
        choice.best = near;
        choice.stretch = from.shape;
        choice.before = from.before;
    }

    /** The draft simulated as a near miss (see acceptNearMiss), or null if it isn't one. */
    private static Evaluation nearMiss(List<Stretch> draft, Job job, double startSpeed) {
        ClearanceRule[] rules = new ClearanceRule[draft.size()], real = new ClearanceRule[draft.size()];
        for (int j = 0; j < draft.size(); j++) {
            rules[j] = draft.get(j).rule;
            real[j] = rules[j].contact ? rules[j] : rules[j].withRadius(rules[j].radius - SAFETY_MARGIN);
        }
        Track track = buildTrack(draft, job.startHeading);
        // The path itself must keep clear of the robot's own outline (no margin given up there).
        if (plannedMiss(track, real) >= 0) return null;
        List<Piece> lastPieces = draft.get(draft.size() - 1).pieces;
        double handoff = handoffDistance(lastPieces.get(lastPieces.size() - 1), job.model);
        MotionModel.Check check = MotionModel.pursue(track, job.model, rules, job.startVx, job.startVy, job.startHeading, job.volts, handoff);
        if (-check.worstSlack > NEAR_MISS_FRACTION * SAFETY_MARGIN) return null;
        MotionModel.Check realCheck = MotionModel.pursue(track, job.model, real, job.startVx, job.startVy, job.startHeading, job.volts, handoff);
        if (realCheck.worstSlack < -1e-6) return null;
        MotionModel.Driven driven = MotionModel.driven(track, check, job.model, startSpeed, job.volts);
        return new Evaluation(draft, track, driven.track, driven.profile, check, true, null, Double.NaN);
    }

    /** The stretches chosen so far (the last one replaced by {@code before} if given) plus the new one, as fresh copies. */
    private static List<Stretch> draft(List<Stretch> chosen, Stretch before, Stretch stretch) {
        List<Stretch> draft = new ArrayList<>();
        int keep = before != null ? chosen.size() - 1 : chosen.size();
        for (int q = 0; q < keep; q++) draft.add(chosen.get(q).copy());
        if (before != null) draft.add(before.copy());
        draft.add(stretch.copy());
        return draft;
    }

    /** Model time with no speed limits, along the planned path. Fast; used to rank candidates. */
    private static double quickSeconds(List<Stretch> draft, Job job, double startSpeed) {
        Track track = buildTrack(draft, job.startHeading);
        List<Piece> lastPieces = draft.get(draft.size() - 1).pieces;
        double handoff = handoffDistance(lastPieces.get(lastPieces.size() - 1), job.model);
        return MotionModel.run(track, job.model, startSpeed, job.volts, handoff).seconds;
    }

    /** Says where the best try got too close, and to what. */
    private static String tooTight(Waypoint.Resolved target, Evaluation closest, Job job) {
        String where = "";
        if (closest != null && closest.check.worstIndex >= 0) {
            int i = closest.check.worstIndex;
            double x = closest.track.x[i], y = closest.track.y[i];
            Zone zone = job.snap.nearest(x, y);
            where = String.format(" (best try: %.1f in. too close to %s near (%.0f, %.0f))",
                    -closest.worst(), zone == null ? "a zone" : zone.name, x, y);
        }
        return String.format("Every way to (%.0f, %.0f) passes too close to a zone, even at %.0f in/s%s",
                target.x, target.y, MIN_SPEED_LIMIT, where);
    }

    private static String noWay(double fromX, double fromY, Waypoint.Resolved to) {
        return String.format("No way from (%.0f, %.0f) to (%.0f, %.0f): zones block it", fromX, fromY, to.x, to.y);
    }

    // ---- Searching ----

    private List<double[]> candidates(Job job, double ax, double ay, double[] startDir, double startRun,
                                      double bx, double by, double[] endDir, double endRun, ClearanceRule rule) {
        List<double[]> found = new ArrayList<>();
        addCandidates(job, found, ax, ay, startDir, startRun, bx, by, endDir, endRun, rule);
        if (!found.isEmpty() || (startDir == null && endDir == null)) return found;
        // A forced direction that can't be met (a waypoint against a wall, say): drop the end's, then the start's, then both.
        if (endDir != null) addCandidates(job, found, ax, ay, startDir, startRun, bx, by, null, 0, rule);
        if (found.isEmpty() && startDir != null) addCandidates(job, found, ax, ay, null, 0, bx, by, endDir, endRun, rule);
        if (found.isEmpty()) addCandidates(job, found, ax, ay, null, 0, bx, by, null, 0, rule);
        if (!found.isEmpty()) job.notes.add(String.format("Couldn't keep the asked direction near (%.0f, %.0f)", bx, by));
        return found;
    }

    /** Polylines for each search radius and turn weight, without repeats. */
    private void addCandidates(Job job, List<double[]> found, double ax, double ay, double[] startDir, double startRun,
                               double bx, double by, double[] endDir, double endRun, ClearanceRule rule) {
        double[] radii = searchRadii(rule, job);
        double[] weights = {TURN_WEIGHT, 0, 3 * TURN_WEIGHT};
        for (double radius : radii) {
            for (double weight : weights) {
                double[] polyline = findPath(job, radius, ax, ay, startDir, startRun, bx, by, endDir, endRun, rule, weight);
                if (polyline != null && !hasHairpin(polyline) && !alreadyHave(found, polyline)) {
                    found.add(polyline);
                    job.foundRadius.put(polyline, radius);
                }
            }
        }
    }

    /**
     * Grown-zone radii to search with, widest first: with CORNER_ROOM, then just
     * enough to follow, then (USE_OUTLINE) with the robot's half width for gaps it
     * can only pass side-on. A contact stretch (Route.allowContact) searches at its
     * own small radius only.
     */
    private static double[] searchRadii(ClearanceRule rule, Job job) {
        if (rule.contact) return new double[]{rule.radius + 0.25};
        List<Double> radii = new ArrayList<>();
        if (job.wide > job.tight) radii.add(job.wide);
        radii.add(job.tight);
        if (job.narrow < job.tight - 0.5) radii.add(job.narrow);
        double[] out = new double[radii.size()];
        for (int i = 0; i < out.length; i++) out[i] = radii.get(i);
        return out;
    }

    /**
     * True if the polyline turns back by more than 150 degrees anywhere. Foresight
     * can get stuck on such a hairpin (its closest point stops moving), so those
     * routes are never used; a through-waypoint that needs one becomes a stop.
     */
    private static boolean hasHairpin(double[] polyline) {
        double[] p = clean(polyline);
        for (int i = 2; i + 1 < p.length - 2; i += 2) {
            double cost = VisibilityGraph.turnCost(p[i] - p[i - 2], p[i + 1] - p[i - 1], p[i + 2] - p[i], p[i + 3] - p[i + 1]);
            if (cost > 1 - Math.cos(Math.toRadians(150))) return true;
        }
        return false;
    }

    private static boolean alreadyHave(List<double[]> found, double[] polyline) {
        for (double[] other : found) if (Arrays.equals(other, polyline)) return true;
        return false;
    }

    private double[] findPath(Job job, double radius, double ax, double ay, double[] startDir, double startRun,
                              double bx, double by, double[] endDir, double endRun, ClearanceRule rule, double weight) {
        VisibilityGraph graph = graph(job.snap, radius, rule.contact);
        VisibilityGraph.Terminal[] starts = terminals(job, radius, rule, ax, ay, startDir, startRun, true);
        VisibilityGraph.Terminal[] ends = terminals(job, radius, rule, bx, by, endDir, endRun, false);
        return graph.search(starts, ends, rule.withRadius(radius), weight);
    }

    private VisibilityGraph graph(FieldMap.Snapshot snap, double radius, boolean contact) {
        for (VisibilityGraph g : graphs) {
            if (g != null && g.version == snap.version && g.radius == radius && g.contact == contact
                    && g.wallGrowth == VisibilityGraph.wallGrowth(radius, contact)) {
                return g;
            }
        }
        VisibilityGraph built = VisibilityGraph.build(snap, radius, contact);
        graphs[nextGraphSlot] = built;
        nextGraphSlot = (nextGraphSlot + 1) % graphs.length;
        return built;
    }

    /**
     * The ways a search may begin or end at point (px, py): straight along a
     * forced direction if there is one, otherwise the point itself, plus a few
     * straight lines out if the point is close to a zone.
     */
    private static VisibilityGraph.Terminal[] terminals(Job job, double radius, ClearanceRule rule, double px, double py,
                                                        double[] dir, double run, boolean isStart) {
        List<VisibilityGraph.Terminal> list = new ArrayList<>();
        double[] here = {px, py};
        if (dir != null) {
            for (double length = run; length >= 2; length *= 0.6) {
                double sign = isStart ? 1 : -1;
                double vx = px + sign * dir[0] * length, vy = py + sign * dir[1] * length;
                if (rule.lineClear(px, py, vx, vy, 1)) {
                    list.add(new VisibilityGraph.Terminal(vx, vy, dir[0], dir[1], length, here));
                    return list.toArray(new VisibilityGraph.Terminal[0]);
                }
            }
            return new VisibilityGraph.Terminal[0];
        }
        list.add(new VisibilityGraph.Terminal(px, py, 0, 0, 0, new double[0]));
        if (job.snap.clearance(px, py) < radius) {
            double[] away = escapeDirection(job.snap, px, py);
            // Every 15 degrees either side of straight out, so in a corner the 45 degree way out is one of them.
            for (double degrees : new double[]{0, 15, -15, 30, -30, 45, -45, 60, -60, 75, -75}) {
                double a = Math.toRadians(degrees);
                double dx = away[0] * Math.cos(a) - away[1] * Math.sin(a);
                double dy = away[0] * Math.sin(a) + away[1] * Math.cos(a);
                for (double e = 2; e <= 48; e += 2) {
                    double ex = px + dx * e, ey = py + dy * e;
                    if (!rule.lineClear(px, py, ex, ey, 1)) break;
                    if (job.snap.clearance(ex, ey) >= radius + 0.25) {
                        list.add(new VisibilityGraph.Terminal(ex, ey, 0, 0, e, here));
                        break;
                    }
                }
            }
        }
        return list.toArray(new VisibilityGraph.Terminal[0]);
    }

    // ---- Rounding the corners ----

    /** A straight line or rounded corner of a stretch. */
    private static final class Piece {
        final Curve curve;
        final boolean corner;
        final double length;
        /** Line ends (for splitting). */
        final double x0, y0, x1, y1;
        /** Speed limit, in/s. +infinity: none. */
        double cap;
        /** The route's own speed limit here (the waypoint's maxSpeed, a pickup's PICKUP_SPEED): cap never goes back above it. */
        final double ownCap;

        Piece(Curve curve, boolean corner, double length, double x0, double y0, double x1, double y1, double cap, double ownCap) {
            this.curve = curve;
            this.corner = corner;
            this.length = length;
            this.x0 = x0;
            this.y0 = y0;
            this.x1 = x1;
            this.y1 = y1;
            this.cap = cap;
            this.ownCap = ownCap;
        }

        static Piece line(double x0, double y0, double x1, double y1, double cap) {
            return line(x0, y0, x1, y1, cap, cap);
        }

        static Piece line(double x0, double y0, double x1, double y1, double cap, double ownCap) {
            Curve line = new Line(Vector2D.cartesian(x0, y0), Vector2D.cartesian(x1, y1));
            return new Piece(line, false, Geometry.length(x1 - x0, y1 - y0), x0, y0, x1, y1, cap, ownCap);
        }

        Piece copy(double newCap) {
            return new Piece(curve, corner, length, x0, y0, x1, y1, newCap, ownCap);
        }

        /** Same piece with a lower own speed limit. */
        Piece ownLimit(double limit) {
            double own = Math.min(ownCap, limit);
            return new Piece(curve, corner, length, x0, y0, x1, y1, Math.min(cap, own), own);
        }

        /** A line: the part from {@code from} to {@code to} inches along it. */
        Piece part(double from, double to, double newCap) {
            double f0 = from / length, f1 = to / length;
            return line(x0 + f0 * (x1 - x0), y0 + f0 * (y1 - y0), x0 + f1 * (x1 - x0), y0 + f1 * (y1 - y0), newCap, ownCap);
        }
    }

    /** The path between two waypoints. */
    private static final class Stretch {
        final List<Piece> pieces;
        final ClearanceRule rule;
        final Waypoint.Resolved target;
        /** The waypoint's speed limit, in/s, +infinity if none. */
        final double userCap;
        /** Filled in by buildTrack: heading at the start, and the signed turn to the waypoint's heading. */
        double startHeading, turn, endHeading;
        /** Keep the start heading for this many inches before following the heading rule (see MAX_START_HOLD). */
        double holdStart = 0;
        /** At least this much of holdStart, whatever else happens: leaving a flower intake stop (see FLOWER_INTAKE_RAISE_RUN). */
        double holdStartAtLeast = 0;
        /** Finish turning to the waypoint's heading this many inches before it (see MAX_END_HOLD). */
        double holdEnd = 0;
        /** For a waypoint without a heading: the one AgateFlow picked to end at (see choose), or NaN to keep the start heading. */
        double chosenHeading = Double.NaN;

        Stretch(List<Piece> pieces, ClearanceRule rule, Waypoint.Resolved target, double userCap) {
            this.pieces = pieces;
            this.rule = rule;
            this.target = target;
            this.userCap = userCap;
        }

        /** Same shape with speed limits back to just the route's own (see Piece.ownCap). */
        Stretch copy() {
            List<Piece> fresh = new ArrayList<>();
            for (Piece piece : pieces) fresh.add(piece.copy(piece.ownCap));
            Stretch copy = new Stretch(fresh, rule, target, userCap);
            copy.chosenHeading = chosenHeading;
            copy.holdStartAtLeast = holdStartAtLeast;
            return copy;
        }

        /** How far the start heading is kept: holdStart, but never less than holdStartAtLeast. */
        double startHold() {
            return Math.max(holdStart, holdStartAtLeast);
        }

        /** Same shape and the same speed limits, as a copy that later changes don't touch. */
        Stretch copyWithLimits() {
            List<Piece> same = new ArrayList<>();
            for (Piece piece : pieces) same.add(piece.copy(piece.cap));
            Stretch copy = new Stretch(same, rule, target, userCap);
            copy.holdStart = holdStart;
            copy.holdStartAtLeast = holdStartAtLeast;
            copy.holdEnd = holdEnd;
            copy.chosenHeading = chosenHeading;
            return copy;
        }

        double length() {
            double total = 0;
            for (Piece piece : pieces) total += piece.length;
            return total;
        }

        /** Unit direction of travel at the start. */
        double[] startDirection() {
            Piece first = pieces.get(0);
            if (first.corner) {
                Vector2D t = first.curve.tangent(0);
                return new double[]{t.x(), t.y()};
            }
            return new double[]{(first.x1 - first.x0) / first.length, (first.y1 - first.y0) / first.length};
        }

        /** Unit direction of travel at the end. */
        double[] endDirection() {
            Piece last = pieces.get(pieces.size() - 1);
            if (last.corner) {
                Vector2D t = last.curve.tangent(1);
                return new double[]{t.x(), t.y()};
            }
            return new double[]{(last.x1 - last.x0) / last.length, (last.y1 - last.y0) / last.length};
        }
    }

    /**
     * Lines and rounded corners along the polyline, each corner as big as the
     * room allows under {@code sizing}. The stretch keeps {@code rule} for the final check.
     * A pickup pass gets its speed limit on the last PICKUP_RUN inches into it, and
     * on the first PICKUP_EXIT inches out of it ({@code from}, the waypoint before, or null).
     * The first {@code straightStart} inches stay a straight line (no corner rounded into them).
     */
    private static Stretch smooth(double[] polyline, ClearanceRule sizing, ClearanceRule rule, Waypoint.Resolved target,
                                  Waypoint.Resolved from, double straightStart) {
        double[] p = clean(polyline);
        int m = p.length / 2;
        if (m < 2) return null;
        double[] ux = new double[m - 1], uy = new double[m - 1], length = new double[m - 1];
        for (int i = 0; i < m - 1; i++) {
            double dx = p[2 * i + 2] - p[2 * i], dy = p[2 * i + 3] - p[2 * i + 1];
            length[i] = Geometry.length(dx, dy);
            ux[i] = dx / length[i];
            uy[i] = dy / length[i];
        }

        // A first line shorter than straightStart (the way out was cut short) still gets a corner after it.
        straightStart = Math.min(straightStart, Math.max(0, length[0] - 0.5));
        // Corner i runs in along line i-1 and out along line i, size[i] inches each way.
        // A line between two corners is shared half and half at first; a corner that
        // can't use its half (something is in the way) leaves the rest to its neighbor.
        double[] size = new double[m];
        for (int pass = 0; pass < 2; pass++) {
            for (int i = 1; i < m - 1; i++) {
                double in, out;
                if (pass == 0) {
                    in = i - 1 == 0 ? length[i - 1] - straightStart : length[i - 1] / 2;
                    out = i + 1 == m - 1 ? length[i] : length[i] / 2;
                } else {
                    in = length[i - 1] - (i - 1 >= 1 ? size[i - 1] : straightStart);
                    out = length[i] - (i + 1 <= m - 2 ? size[i + 1] : 0);
                    if (Math.min(in, out) <= size[i] + 0.01) continue;
                }
                size[i] = biggestCorner(p[2 * i], p[2 * i + 1], ux[i - 1], uy[i - 1], ux[i], uy[i], Math.min(in, out), sizing);
            }
        }

        double cap = Double.isNaN(target.maxSpeed) ? Double.POSITIVE_INFINITY : target.maxSpeed;
        List<Piece> pieces = new ArrayList<>();
        for (int i = 0; i < m - 1; i++) {
            double cutStart = i >= 1 ? size[i] : 0, cutEnd = i + 1 <= m - 2 ? size[i + 1] : 0;
            double sx = p[2 * i] + ux[i] * cutStart, sy = p[2 * i + 1] + uy[i] * cutStart;
            double ex = p[2 * i + 2] - ux[i] * cutEnd, ey = p[2 * i + 3] - uy[i] * cutEnd;
            if (Geometry.length(ex - sx, ey - sy) > 0.01) pieces.add(Piece.line(sx, sy, ex, ey, cap));
            if (i + 1 <= m - 2 && size[i + 1] > 0) {
                CornerCurve corner = CornerCurve.at(p[2 * i + 2], p[2 * i + 3], ux[i], uy[i], ux[i + 1], uy[i + 1], size[i + 1]);
                pieces.add(new Piece(corner, true, corner.length(), 0, 0, 0, 0, cap, cap));
            }
        }
        if (pieces.isEmpty()) return null;
        if (!Double.isNaN(target.pickupSpeed)) limitEnd(pieces, PICKUP_RUN, target.pickupSpeed);
        if (from != null && !Double.isNaN(from.pickupSpeed) && !from.stop) limitStart(pieces, PICKUP_EXIT, from.pickupSpeed);
        return new Stretch(pieces, rule, target, cap);
    }

    /** Gives the last {@code inches} of the pieces this own speed limit (splitting a line; a corner gets it whole). */
    private static void limitEnd(List<Piece> pieces, double inches, double limit) {
        double left = inches;
        for (int q = pieces.size() - 1; q >= 0 && left > 0; q--) {
            Piece piece = pieces.get(q);
            if (piece.corner || piece.length <= left + 0.5) {
                pieces.set(q, piece.ownLimit(limit));
            } else {
                pieces.set(q, piece.part(piece.length - left, piece.length, piece.cap).ownLimit(limit));
                pieces.add(q, piece.part(0, piece.length - left, piece.cap));
            }
            left -= piece.length;
        }
    }

    /** Gives the first {@code inches} of the pieces this own speed limit. */
    private static void limitStart(List<Piece> pieces, double inches, double limit) {
        double left = inches;
        for (int q = 0; q < pieces.size() && left > 0; q++) {
            Piece piece = pieces.get(q);
            if (piece.corner || piece.length <= left + 0.5) {
                pieces.set(q, piece.ownLimit(limit));
            } else {
                pieces.set(q, piece.part(left, piece.length, piece.cap));
                pieces.add(q, piece.part(0, left, piece.cap).ownLimit(limit));
                q++;
            }
            left -= piece.length;
        }
    }

    /**
     * If stretch b leaves a through-waypoint in a different direction than stretch a
     * arrives (a direction couldn't be kept), rounds that joint with a corner, so
     * the path stays smooth. The robot then passes near the waypoint instead of
     * through it. Only when both sides are straight lines there.
     */
    private static void roundJoint(Stretch a, Stretch b) {
        double[] in = a.endDirection(), out = b.startDirection();
        double dot = in[0] * out[0] + in[1] * out[1];
        // Nothing to round; or it turns back, which would make a hairpin (planLeg makes that waypoint a stop).
        if (dot > 0.9995 || dot < Math.cos(Math.toRadians(120))) return;
        int lastIndex = a.pieces.size() - 1;
        Piece last = a.pieces.get(lastIndex), first = b.pieces.get(0);
        if (last.corner || first.corner) return;
        double room = Math.min(last.length - 0.05, first.length - 0.05);
        double size = biggestCorner(first.x0, first.y0, in[0], in[1], out[0], out[1], room, b.rule);
        if (size < 0.1) return;
        a.pieces.set(lastIndex, Piece.line(last.x0, last.y0, last.x1 - in[0] * size, last.y1 - in[1] * size, last.cap, last.ownCap));
        CornerCurve corner = CornerCurve.at(first.x0, first.y0, in[0], in[1], out[0], out[1], size);
        b.pieces.set(0, Piece.line(first.x0 + out[0] * size, first.y0 + out[1] * size, first.x1, first.y1, first.cap, first.ownCap));
        b.pieces.add(0, new Piece(corner, true, corner.length(), 0, 0, 0, 0, first.cap, first.ownCap));
    }

    /** Drops repeated points and points where the line doesn't turn. */
    private static double[] clean(double[] polyline) {
        double[] out = new double[polyline.length];
        int n = 0;
        for (int i = 0; i < polyline.length / 2; i++) {
            double x = polyline[2 * i], y = polyline[2 * i + 1];
            if (n >= 2 && Geometry.length(x - out[n - 2], y - out[n - 1]) < 1e-6) continue;
            if (n >= 4) {
                double ax = out[n - 2] - out[n - 4], ay = out[n - 1] - out[n - 3];
                double bx = x - out[n - 2], by = y - out[n - 1];
                double cross = ax * by - ay * bx, dot = ax * bx + ay * by;
                if (dot > 0 && Math.abs(cross) < 1e-4 * Geometry.length(ax, ay) * Geometry.length(bx, by)) n -= 2;
            }
            out[n++] = x;
            out[n++] = y;
        }
        return Arrays.copyOf(out, n);
    }

    /** Biggest corner size (inches each way, at most {@code room}) whose curve keeps the rule. */
    private static double biggestCorner(double cx, double cy, double inX, double inY, double outX, double outY,
                                        double room, ClearanceRule rule) {
        if (room < 0.1) return 0;
        if (cornerClear(cx, cy, inX, inY, outX, outY, room, rule)) return room;
        double low = 0, high = room;
        for (int i = 0; i < 7; i++) {
            double mid = (low + high) / 2;
            if (cornerClear(cx, cy, inX, inY, outX, outY, mid, rule)) low = mid;
            else high = mid;
        }
        // A sharp corner is worse than a slightly tight one: MotionModel.pursue has the final say.
        return Math.max(low, Math.min(0.5, room));
    }

    private static boolean cornerClear(double cx, double cy, double inX, double inY, double outX, double outY,
                                       double size, ClearanceRule rule) {
        double[] c = CornerCurve.controls(cx, cy, inX, inY, outX, outY, size);
        double[] point = new double[2];
        for (int i = 1; i < 8; i++) {
            CornerCurve.point(c, i / 8.0, point);
            if (rule.slack(point[0], point[1]) < -1e-6) return false;
        }
        return true;
    }

    // ---- Simulating ----

    private static final class Evaluation {
        final List<Stretch> stretches;
        /** The planned path, sampled. */
        final Track track;
        /** Where the robot is predicted to really drive, and its speeds and times there. */
        final Track driven;
        final MotionModel.Profile profile;
        final MotionModel.Check check;
        final boolean safe;
        /** Not safe: the first (fastest) try that came within NEAR_MISS_FRACTION of the margin, with its speed limits then. Or null. */
        final List<Stretch> nearMiss;
        /** Model time of that try, seconds. */
        final double nearMissSeconds;
        /** Set by choose(): the candidate's shape, and the previous stretch rounded to meet it (or null). */
        Stretch shape, before;
        /** Where the planned path itself breaks the rule (see plannedMiss): how far, as slack. +infinity if it doesn't. */
        double plannedSlack = Double.POSITIVE_INFINITY;

        /** The least slack of the planned and the predicted path. */
        double worst() {
            return Math.min(check.worstSlack, plannedSlack);
        }

        Evaluation(List<Stretch> stretches, Track track, Track driven, MotionModel.Profile profile,
                   MotionModel.Check check, boolean safe, List<Stretch> nearMiss, double nearMissSeconds) {
            this.stretches = stretches;
            this.track = track;
            this.driven = driven;
            this.profile = profile;
            this.check = check;
            this.safe = safe;
            this.nearMiss = nearMiss;
            this.nearMissSeconds = nearMissSeconds;
        }
    }

    /**
     * Drives the draft in simulation. Where the predicted path gets too close to
     * a zone, or misses a through-waypoint, adds a speed limit there and tries
     * again. Once it is safe, the time is worked out again along the predicted
     * path itself.
     */
    private Evaluation evaluate(List<Stretch> draft, Job job, double startSpeed) {
        ClearanceRule[] rules = new ClearanceRule[draft.size()];
        for (int j = 0; j < draft.size(); j++) rules[j] = draft.get(j).rule;
        Track track = null;
        MotionModel.Profile profile = null;
        MotionModel.Check check = null;
        boolean tryThrough = true;
        int pickupTries = 3;
        List<Stretch> nearMiss = null;
        double nearMissSeconds = Double.POSITIVE_INFINITY;
        for (int attempt = 0; attempt < 16; attempt++) {
            track = buildTrack(draft, job.startHeading);
            List<Piece> lastPieces = draft.get(draft.size() - 1).pieces;
            double handoff = handoffDistance(lastPieces.get(lastPieces.size() - 1), job.model);
            profile = MotionModel.run(track, job.model, startSpeed, job.volts, handoff);
            check = MotionModel.pursue(track, job.model, rules, job.startVx, job.startVy, job.startHeading, job.volts, handoff);
            int plannedMiss = plannedMiss(track, rules);
            if (plannedMiss >= 0) {
                // The path itself, facing the way it asks, puts the outline into something (a corner
                // next to a wall while it faces the wrong way, say). Slowing down can't fix that;
                // turning later at the start might. Otherwise this way is out.
                if (holdHeading(draft, track, plannedMiss)) continue;
                Evaluation out = new Evaluation(draft, track, track, profile, check, false, null, Double.NaN);
                out.plannedSlack = rules[track.stretch[plannedMiss]].slack(track.x[plannedMiss], track.y[plannedMiss], track.heading[plannedMiss]);
                return out;
            }
            if (nearMiss == null && check.worstSlack < -1e-6 && -check.worstSlack <= NEAR_MISS_FRACTION * SAFETY_MARGIN) {
                // Kept in case nothing turns out safe (see acceptNearMiss).
                nearMiss = new ArrayList<>();
                for (Stretch stretch : draft) nearMiss.add(stretch.copyWithLimits());
                nearMissSeconds = profile.seconds;
            }
            int missed = tryThrough ? missedThrough(draft, track, check) : -1;
            if (check.worstSlack >= -1e-6 && missed >= 0) {
                // Safe, but cuts a through-waypoint by too much: slow down there and try again.
                if (job.model.foresight != null && slowDown(draft, track, profile, missed, missed, 0, job.model)) continue;
                tryThrough = false;
            }
            int missedPickup = pickupTries > 0 && check.worstSlack >= -1e-6 ? missedPickup(draft, check) : -1;
            if (missedPickup >= 0) {
                // Safe, but the predicted path misses a pickup's targets: slow down before it and try again
                // (a few times at most: if slowing down doesn't do it, the capture test reports it skipped).
                pickupTries--;
                if (job.model.foresight != null && slowForPickup(draft, track, profile, missedPickup, job.model)) continue;
                pickupTries = 0;
            }
            if (check.worstSlack >= -1e-6) {
                MotionModel.Driven driven = MotionModel.driven(track, check, job.model, startSpeed, job.volts);
                return new Evaluation(draft, track, driven.track, driven.profile, check, true, null, Double.NaN);
            }
            if (holdHeading(draft, track, check.worstIndex)) continue;
            if (job.model.foresight == null
                    || !slowDown(draft, track, profile, check.worstIndex, check.worstAim, -check.worstSlack, job.model)) {
                break;
            }
        }
        return new Evaluation(draft, track, track, profile, check, false, nearMiss, nearMissSeconds);
    }

    /**
     * The first sample where the planned path itself, with its planned heading, breaks
     * its stretch's rule (the outline's room), or -1. The predicted path is checked
     * by MotionModel.pursue; this makes sure the path Pedro follows is clean too.
     */
    private static int plannedMiss(Track track, ClearanceRule[] rules) {
        for (int i = 0; i < track.count; i++) {
            if (rules[track.stretch[i]].slack(track.x[i], track.y[i], track.heading[i]) < -1e-6) return i;
        }
        return -1;
    }

    /**
     * Too close while turning, near the start (see holdStart) or near the end of a
     * stretch (see holdEnd): tries the hold for the nearer of the two first.
     */
    private static boolean holdHeading(List<Stretch> draft, Track track, int i) {
        if (i < 0) return false;
        int last = i;
        while (last + 1 < track.count && track.stretch[last + 1] == track.stretch[i]) last++;
        boolean nearerEnd = track.stretch[i] > 0 || track.s[last] - track.s[i] < track.s[i];
        return nearerEnd ? holdEnd(draft, track, i) || holdStart(draft, track, i) : holdStart(draft, track, i) || holdEnd(draft, track, i);
    }

    /**
     * Too close right at the start, on the first stretch, while turning: the robot's
     * corners swing into something next to it as it turns. Keep the start heading a
     * bit further along (6 in. past the trouble, up to MAX_START_HOLD) and try again.
     * Returns false if that doesn't apply or the hold is already as long as allowed.
     */
    private static boolean holdStart(List<Stretch> draft, Track track, int i) {
        if (i < 0 || track.stretch[i] != 0 || track.s[i] > MAX_START_HOLD) return false;
        Stretch first = draft.get(0);
        if (first.target.turnRule.kind == Heading.Kind.FACING) return false;
        double turned = Math.abs(Angles.wrapRadians(track.heading[i] - first.startHeading));
        if (turned < Math.toRadians(5) && first.startHold() <= 0) return false;
        double hold = Math.min(MAX_START_HOLD, Math.max(first.startHold() + 6, track.s[i] + 6));
        if (hold <= first.startHold() + 0.5) return false;
        first.holdStart = hold;
        return true;
    }

    /**
     * Too close right before the end of a stretch, while still turning to its
     * waypoint's heading: the robot's corners swing into something next to the
     * waypoint as it turns (a wall beside a stop, say). Finish the turn earlier, 6 in.
     * before the trouble (up to MAX_END_HOLD before the waypoint), so the robot drives
     * the last part already facing the end heading, and try again. Only for a heading
     * that turns (Heading.linear, finishBy, between): tangent and facing headings follow
     * the path. Returns false if that doesn't apply or the hold is already as long as allowed.
     */
    private static boolean holdEnd(List<Stretch> draft, Track track, int i) {
        if (i < 0) return false;
        int j = track.stretch[i];
        Stretch stretch = draft.get(j);
        if (stretch.target.turnRule.kind != Heading.Kind.TURN) return false;
        int last = i;
        while (last + 1 < track.count && track.stretch[last + 1] == j) last++;
        double toEnd = track.s[last] - track.s[i];
        if (toEnd > MAX_END_HOLD) return false;
        double turned = Math.abs(Angles.wrapRadians(track.heading[i] - track.heading[last]));
        if (turned < Math.toRadians(5) && stretch.holdEnd <= 0) return false;
        // Leave the turn at least a few inches.
        double hold = Math.min(Math.min(MAX_END_HOLD, stretch.length() - 3), Math.max(stretch.holdEnd + 6, toEnd + 6));
        if (hold <= stretch.holdEnd + 0.5) return false;
        stretch.holdEnd = hold;
        return true;
    }

    /**
     * The first stretch ending at a pickup pass whose targets the predicted path
     * doesn't take (Pickup.capturedAt, PICKUP_SIDE_MARGIN to spare), or -1.
     */
    private static int missedPickup(List<Stretch> draft, MotionModel.Check check) {
        for (int j = 0; j < draft.size(); j++) {
            double[] targets = draft.get(j).target.targets;
            if (targets == null) continue;
            for (int t = 0; t + 1 < targets.length; t += 2) {
                if (Pickup.capturedAt(targets[t], targets[t + 1], check.path, check.heading, check.speed, PICKUP_SIDE_MARGIN) < 0) return j;
            }
        }
        return -1;
    }

    /**
     * Slows the way into the pickup at the end of stretch {@code j}: 20% under the
     * speed there (never above PICKUP_SPEED, never under MIN_SPEED_LIMIT),
     * starting early enough to get that slow. Slower, the robot cuts less and
     * arrives straighter. Returns false if it can't go any slower.
     */
    private static boolean slowForPickup(List<Stretch> draft, Track track, MotionModel.Profile profile, int j, DriveModel model) {
        int end = 0;
        while (end < track.count - 1 && track.stretch[end + 1] <= j) end++;
        double at = track.s[end];
        int from = track.indexAt(at - PICKUP_RUN);
        double current = 0, entry = 0;
        for (int i = from; i <= end; i++) current = Math.max(current, Math.min(track.cap[i], profile.speed[i]));
        double limit = Math.max(Math.min(current, PICKUP_SPEED) * 0.8, MIN_SPEED_LIMIT);
        for (int i = track.indexAt(at - PICKUP_RUN - 40); i <= end; i++) entry = Math.max(entry, profile.speed[i]);
        double lead = Math.max(0, entry - limit) * model.forwardTau * 2 + 6;
        return limitBetween(draft, at - PICKUP_RUN - lead, at + PICKUP_EXIT, limit);
    }

    /**
     * The last track sample before the first through-waypoint that the predicted
     * path misses by more than THROUGH_TOLERANCE, or -1 if none.
     */
    private static int missedThrough(List<Stretch> draft, Track track, MotionModel.Check check) {
        for (int j = 0; j < draft.size() - 1; j++) {
            Waypoint.Resolved w = draft.get(j).target;
            if (check.closestTo(w.x, w.y) <= THROUGH_TOLERANCE) continue;
            int last = -1;
            for (int i = 0; i < track.count; i++) if (track.stretch[i] == j) last = i;
            return last;
        }
        return -1;
    }

    /** Foresight calls the path done when it reaches t = 1 - parametricTConstraint on the last segment. */
    private static double handoffDistance(Piece last, DriveModel model) {
        if (last.corner) {
            CornerCurve corner = (CornerCurve) last.curve;
            return corner.length() - corner.lengthTo(1 - model.endFraction);
        }
        return last.length * model.endFraction;
    }

    /**
     * Speed limit where the predicted path got too close: from where the robot was
     * ({@code from}) to a little past where it was aiming ({@code to}), 15 to 40%
     * under the speed there (more for a bigger {@code miss}, inches), starting
     * early enough before it to slow down in time. Returns false if nothing there
     * could be slowed any more (all at MIN_SPEED_LIMIT).
     */
    private static boolean slowDown(List<Stretch> draft, Track track, MotionModel.Profile profile, int from, int to,
                                    double miss, DriveModel model) {
        if (from < 0) return false;
        to = Math.max(to, from);
        double current = Math.min(track.cap[from], profile.speed[from]);
        double floor = MIN_SPEED_LIMIT;
        if (from >= track.count - 1) {
            // Overshooting the stop while settling: slow the last part of the way in.
            from = track.indexAt(track.length() - 12);
            current = 0;
            for (int i = from; i < track.count; i++) current = Math.max(current, Math.min(track.cap[i], profile.speed[i]));
            floor = MIN_STOP_APPROACH_SPEED;
        }
        double factor = miss > 2 ? 0.6 : miss > 0.5 ? 0.75 : 0.85;
        double limit = Math.max(current * factor, floor);
        double a = track.s[from], b = track.s[to] + 6;
        // Fastest speed coming in, to know how far back slowing down must start.
        double entry = 0;
        for (int i = track.indexAt(a - 40); i <= from; i++) entry = Math.max(entry, profile.speed[i]);
        double lead = Math.max(0, entry - limit) * model.forwardTau * 2 + 6;
        return limitBetween(draft, a - lead, b, limit);
    }

    /**
     * Puts the speed limit on the path between {@code a} and {@code b} inches along
     * the leg. Corners get it whole; lines are split so only the part inside gets
     * it. Returns true if any limit went down.
     */
    private static boolean limitBetween(List<Stretch> draft, double a, double b, double limit) {
        boolean changed = false;
        double along = 0;
        for (Stretch stretch : draft) {
            List<Piece> pieces = stretch.pieces;
            for (int q = 0; q < pieces.size(); q++) {
                Piece piece = pieces.get(q);
                double start = along, end = along + piece.length;
                along = end;
                if (end <= a || start >= b || piece.cap <= limit + 1e-6) continue;
                changed = true;
                if (piece.corner) {
                    piece.cap = limit;
                    continue;
                }
                double cutIn = Math.max(a, start) - start, cutOut = Math.min(b, end) - start;
                // No slivers: every part at least half an inch (or the whole line).
                if (cutOut - cutIn < 0.5) cutOut = Math.min(piece.length, cutIn + 0.5);
                if (cutOut - cutIn < 0.5) cutIn = Math.max(0, cutOut - 0.5);
                if (cutIn < 0.5) cutIn = 0;
                if (piece.length - cutOut < 0.5) cutOut = piece.length;
                List<Piece> parts = new ArrayList<>();
                if (cutIn > 0) parts.add(piece.part(0, cutIn, piece.cap));
                parts.add(piece.part(cutIn, cutOut, limit));
                if (cutOut < piece.length) parts.add(piece.part(cutOut, piece.length, piece.cap));
                pieces.remove(q);
                pieces.addAll(q, parts);
                q += parts.size() - 1;
            }
        }
        return changed;
    }

    /** Samples the draft about every inch, with headings from each stretch's Heading. */
    private static Track buildTrack(List<Stretch> draft, double startHeading) {
        int count = 1;
        for (Stretch stretch : draft) for (Piece piece : stretch.pieces) count += samples(piece);
        Track track = new Track(count);

        int i = 0;
        double along = 0, heading = startHeading;
        for (int j = 0; j < draft.size(); j++) {
            Stretch stretch = draft.get(j);
            double length = stretch.length(), done = 0;
            stretch.startHeading = heading;
            // No heading of its own (Route.to(x, y)): keep the one it has, or turn to the one AgateFlow picked.
            double goal = Double.isNaN(stretch.target.heading) ? stretch.chosenHeading : stretch.target.heading;
            stretch.turn = Double.isNaN(goal) ? 0 : Angles.wrapRadians(goal - heading);
            Heading rule = stretch.target.turnRule;
            for (int p = 0; p < stretch.pieces.size(); p++) {
                Piece piece = stretch.pieces.get(p);
                int n = samples(piece);
                for (int q = i == 0 ? 0 : 1; q <= n; q++) {
                    double t = (double) q / n, arc;
                    if (piece.corner) {
                        CornerCurve c = (CornerCurve) piece.curve;
                        track.x[i] = c.x(t);
                        track.y[i] = c.y(t);
                        track.travel[i] = Math.atan2(c.dy(t), c.dx(t));
                        track.bend[i] = c.bend(t);
                        arc = c.lengthTo(t);
                    } else {
                        track.x[i] = piece.x0 + t * (piece.x1 - piece.x0);
                        track.y[i] = piece.y0 + t * (piece.y1 - piece.y0);
                        track.travel[i] = Math.atan2(piece.y1 - piece.y0, piece.x1 - piece.x0);
                        track.bend[i] = 0;
                        arc = t * piece.length;
                    }
                    track.s[i] = along + arc;
                    track.plannedS[i] = track.s[i];
                    // A turn that must be done early (holdEnd) runs over the stretch less its last holdEnd inches.
                    double turnLength = length - stretch.holdEnd;
                    double fraction = turnLength > 0 ? Math.min(1, (done + arc) / turnLength) : 1;
                    double planned = rule.at(fraction, stretch.startHeading, stretch.turn, track.travel[i], track.x[i], track.y[i]);
                    track.heading[i] = held(stretch, done + arc, planned);
                    track.cap[i] = piece.cap;
                    track.stretch[i] = j;
                    track.piece[i] = p;
                    i++;
                }
                along += piece.length;
                done += piece.length;
            }
            heading = track.heading[i - 1];
            stretch.endHeading = heading;
        }

        // Unwrap so turning rates don't see 2π jumps, then rate of turn per inch.
        for (int q = 1; q < count; q++) {
            track.heading[q] = track.heading[q - 1] + Angles.wrapRadians(track.heading[q] - track.heading[q - 1]);
        }
        for (int q = 0; q < count; q++) {
            int a = Math.max(q - 1, 0), b = Math.min(q + 1, count - 1);
            double ds = track.s[b] - track.s[a];
            track.turnRate[q] = ds > 1e-9 ? (track.heading[b] - track.heading[a]) / ds : 0;
        }
        return track;
    }

    /**
     * The heading at this far along a stretch: its rule's ({@code planned}), except
     * during a start hold (see MAX_START_HOLD), where it keeps the stretch's start
     * heading and then turns to the rule's over START_HOLD_BLEND inches.
     */
    private static double held(Stretch stretch, double along, double planned) {
        double hold = stretch.startHold();
        if (hold <= 0 || along >= hold + START_HOLD_BLEND) return planned;
        if (along <= hold) return stretch.startHeading;
        double f = (along - hold) / START_HOLD_BLEND;
        return stretch.startHeading + f * Angles.wrapRadians(planned - stretch.startHeading);
    }

    private static int samples(Piece piece) {
        if (piece.corner) return Math.max(6, (int) Math.ceil(piece.length / CORNER_STEP));
        return Math.max(1, (int) Math.ceil(piece.length / LINE_STEP));
    }

    // ---- Building the Pedro path ----

    private Leg buildLeg(Evaluation e, Job job) {
        List<Path> stretchPaths = new ArrayList<>();
        List<Curve> segments = new ArrayList<>();
        List<Double> starts = new ArrayList<>();
        List<Waypoint> waypoints = new ArrayList<>();
        double[] waypointAt = new double[e.stretches.size()];
        double along = 0;
        int limited = 0;
        for (int j = 0; j < e.stretches.size(); j++) {
            Stretch stretch = e.stretches.get(j);
            List<Path> piecePaths = new ArrayList<>();
            for (Piece piece : stretch.pieces) {
                List<Modifier> modifiers = Collections.emptyList();
                if (piece.cap != Double.POSITIVE_INFINITY && job.model.foresight != null) {
                    double cap = Math.min(piece.cap, job.model.speedLimit);
                    modifiers = Collections.singletonList(job.model.foresight.config.maxVelocityConstraint.at(cap));
                    if (piece.cap < piece.ownCap) limited++;
                }
                piecePaths.add(new AtomicPath(piece.curve, null, modifiers));
                segments.add(piece.curve);
                starts.add(along);
                along += piece.length;
            }
            stretchPaths.add(new CompoundPath(interpolator(stretch), Collections.<Modifier>emptyList(), piecePaths));
            waypoints.add(stretch.target.source);
            waypointAt[j] = along;
        }
        Path path = stretchPaths.size() == 1 ? stretchPaths.get(0)
                : new CompoundPath(null, Collections.<Modifier>emptyList(), stretchPaths);

        for (Stretch stretch : e.stretches) {
            // Say when the heading isn't simply what the route asked for.
            if (stretch.holdStart > stretch.holdStartAtLeast) {
                job.notes.add(String.format("Keeps its heading for the first %.0f in. toward (%.0f, %.0f) before turning, so its corners clear what's next to it",
                        stretch.holdStart, stretch.target.x, stretch.target.y));
            } else if (stretch.holdStartAtLeast > 0) {
                job.notes.add(String.format("Backs away with its heading kept for the first %.0f in., while the flower intake comes up",
                        stretch.holdStartAtLeast));
            }
            if (stretch.holdEnd > 0) {
                job.notes.add(String.format("Turns to its heading %.0f in. before (%.0f, %.0f), so its corners clear what's next to it",
                        stretch.holdEnd, stretch.target.x, stretch.target.y));
            }
            if (!Double.isNaN(stretch.chosenHeading)) {
                job.notes.add(String.format("No heading asked for at (%.0f, %.0f): it faces %.0f degrees there, so it fits",
                        stretch.target.x, stretch.target.y, Math.toDegrees(stretch.chosenHeading)));
            }
        }
        Waypoint.Resolved stop = e.stretches.get(e.stretches.size() - 1).target;
        double endHeading = e.stretches.get(e.stretches.size() - 1).endHeading;
        double[] segmentStart = new double[starts.size()];
        for (int i = 0; i < segmentStart.length; i++) segmentStart[i] = starts.get(i);
        if (e.check.worstSlack < -1e-6) {
            // A near miss (acceptNearMiss): say where.
            int i = Math.max(e.check.worstIndex, 0);
            Zone zone = job.snap.nearest(e.track.x[i], e.track.y[i]);
            job.notes.add(String.format("Near (%.0f, %.0f) the robot passes %.1f in. closer to %s than SAFETY_MARGIN asks"
                            + " (no way kept all of it; it still clears ROBOT_RADIUS)",
                    e.track.x[i], e.track.y[i], -e.check.worstSlack, zone == null ? "a zone" : zone.name));
        }
        return new Leg(path, new Pose(stop.x, stop.y, endHeading), e.profile.seconds, e.track.length(),
                e.profile.topSpeed, e.profile.lowestVolts, e.profile.peakAmps, e.check.worstSlack, limited,
                e.driven, e.profile, segments.toArray(new Curve[0]), segmentStart, waypoints, waypointAt, e.check.path, e.check.heading,
                e.check.speed);
    }

    /** The same heading rule buildTrack used, as a Pedro Interpolator over the stretch. */
    private static Interpolator interpolator(Stretch stretch) {
        final Interpolator planned = ruleInterpolator(stretch);
        if (stretch.startHold() <= 0) return planned;
        final Stretch held = stretch;
        final double length = stretch.length();
        return (curve, t) -> held(held, curve.pathCompletion(t) * length, planned.interpolate(curve, t));
    }

    /** The waypoint's Heading as a Pedro Interpolator over the stretch, without any start hold. */
    private static Interpolator ruleInterpolator(Stretch stretch) {
        final Heading rule = stretch.target.turnRule;
        final double start = stretch.startHeading, turn = stretch.turn;
        // The turn ends holdEnd inches early (see buildTrack): stretch the completion to match.
        final double length = stretch.length(), turnLength = length - stretch.holdEnd;
        final double scale = turnLength > 0 ? length / turnLength : Double.POSITIVE_INFINITY;
        switch (rule.kind) {
            case TANGENT:
                return Interpolator.tangent;
            case REVERSE_TANGENT:
                return Interpolator.tangent.reverse();
            case FACING:
                return Interpolator.facingPoint(Vector2D.cartesian(rule.x, rule.y));
            case TURN:
            default:
                return (curve, t) -> rule.at(Math.min(1, curve.pathCompletion(t) * scale), start, turn, 0, 0, 0);
        }
    }

    /**
     * Already at the spot: hold it and turn to the waypoint's heading (Foresight turns
     * the short way). Null if the robot's corners would swing into something on the way
     * round: past 0 (touching), or, if the robot already overlaps something at the
     * start (pressed against a wall, say), more than END_SLACK past that overlap.
     */
    private static Leg turnInPlace(Job job, Waypoint.Resolved stop) {
        DriveModel model = job.model;
        double goal = stop.heading;
        if (stop.turnRule.kind == Heading.Kind.FACING) goal = Math.atan2(stop.turnRule.y - stop.y, stop.turnRule.x - stop.x);
        else if (Double.isNaN(goal)) goal = job.startHeading;
        double signed = Angles.wrapRadians(goal - job.startHeading);
        if (USE_OUTLINE && !stop.contact) {
            double front = stop.flowerDown ? IntakePoses.FLOWER_INTAKE_REACH : ROBOT_FRONT;
            int steps = Math.max(1, (int) Math.ceil(Math.abs(signed) / Math.toRadians(3)));
            for (Zone zone : job.snap.zones) {
                double atStart = zone.outlineDistance(job.startX, job.startY, job.startHeading, front, ROBOT_BACK, ROBOT_HALF_WIDTH);
                double floor = Math.min(0, atStart - ClearanceRule.END_SLACK);
                for (int q = 1; q <= steps; q++) {
                    double h = job.startHeading + signed * q / steps;
                    if (zone.outlineDistance(job.startX, job.startY, h, front, ROBOT_BACK, ROBOT_HALF_WIDTH) < floor) return null;
                }
            }
        }
        double turn = Math.abs(signed);
        double seconds = turn / (job.volts * model.turnPerVolt) + (turn > 0.01 ? model.turnTau : 0) + model.settleSeconds;
        return new Leg(null, new Pose(stop.x, stop.y, goal), seconds, 0, 0, job.volts, 0,
                Double.POSITIVE_INFINITY, 0, null, null, new Curve[0], new double[0],
                Collections.singletonList(stop.source), new double[]{0}, new double[0], new double[0], new double[0]);
    }

    private static boolean bad(double value) {
        return Double.isNaN(value) || Double.isInfinite(value);
    }
}
