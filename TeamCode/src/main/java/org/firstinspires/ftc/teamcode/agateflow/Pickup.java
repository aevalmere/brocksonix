package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

/**
 * Route.pickUp: things on the floor (POLLEN) for the robot to drive over intake
 * first, in the order that takes the least time. Made by Route; worked out by
 * AgateFlow when it plans.
 *
 * How the order is picked:
 * <ol>
 * <li>Targets close enough together for the intake to take in one pass become one
 *     group (every two within 2 x (INTAKE_HALF_WIDTH - BALL_RADIUS)).</li>
 * <li>A quick time for every hop (start to group, group to group, group to the next
 *     waypoint): the shortest way around the zones, driven at full speed between
 *     groups, at PICKUP_SPEED through each group, slower where it turns there, and
 *     stopping where it turns back.</li>
 * <li>Every order is tried with the quick times when there are few enough (up to
 *     BRUTE_FORCE_ORDERS, which is up to 7 groups all taken); otherwise Held-Karp
 *     (dynamic programming over every subset, up to MAX_PICKUP_GROUPS groups) finds
 *     near-best ones. It keeps only the quickest way into each group for each set
 *     of groups, but the time lost turning at a group depends on the group before
 *     it too, so it can miss the very best order by a few percent. Every order would
 *     take far too long by then, and the full plans of the best few decide anyway.
 *     With most(n), only orders taking up to n targets count, and the ones taking
 *     the most win.</li>
 * <li>AgateFlow plans the best of those in full (the same simulation as any
 *     route) and keeps the fastest: up to PICKUP_ORDERS_TRIED within 15% of the
 *     best quick time, and more if the full plans come out much slower than their
 *     quick times say (something the quick times can't see, like a pickup next to
 *     a zone), while PLAN_TIME_LIMIT_MS allows.</li>
 * </ol>
 * Each group becomes a pass: a through-waypoint (a stop if the route turns back
 * there, or at the end of the route) where the intake front is INTAKE_DEPTH past the
 * group, driven into straight for PICKUP_RUN inches, facing forward, at no more
 * than PICKUP_SPEED from PICKUP_RUN before it to PICKUP_EXIT after it.
 *
 * After planning, every target (picked or not) is run past the predicted path:
 * it counts as picked up if it goes at least INTAKE_DEPTH into the intake mouth
 * (within INTAKE_HALF_WIDTH of the robot's center line) at no more than
 * PICKUP_SPEED + 3 in/s. Plan.pickedUp() and Plan.skipped() say which. A replan
 * (DriveRoute) keeps the order and drops passes already driven.
 */
final class Pickup {
    /** Targets written for red (flipped for blue), or null. */
    final List<Pose> redTargets;
    /** Targets asked for when the drive starts, in field coordinates, or null. */
    final Supplier<List<Pose>> liveTargets;
    /** Pick up at most this many (Route.most). */
    int most = Integer.MAX_VALUE;
    /** Keep the order given (Route.inOrder). */
    boolean inOrder = false;

    Pickup(List<Pose> redTargets, Supplier<List<Pose>> liveTargets) {
        this.redTargets = redTargets;
        this.liveTargets = liveTargets;
    }

    /** For live targets: a copy holding the list the supplier gives right now (see Waypoint.askNow). */
    Pickup askNow() {
        if (liveTargets == null) return this;
        List<Pose> asked;
        try {
            asked = liveTargets.get();
        } catch (RuntimeException e) {
            asked = null;
        }
        final List<Pose> now = asked == null ? Collections.<Pose>emptyList() : new ArrayList<>(asked);
        Pickup copy = new Pickup(null, () -> now);
        copy.most = most;
        copy.inOrder = inOrder;
        return copy;
    }

    /**
     * The targets in field coordinates, as {x, y} pairs, in the order given. Null,
     * NaN and off-field targets are left out (a camera that lost one, say).
     */
    List<double[]> resolve(Alliance alliance) {
        List<Pose> given = liveTargets != null ? liveTargets.get() : redTargets;
        List<double[]> out = new ArrayList<>();
        if (given == null) return out;
        for (Pose pose : given) {
            if (pose == null) continue;
            Pose field = liveTargets == null && alliance == Alliance.BLUE ? alliance.fromRed(pose) : pose;
            double x = field.x(), y = field.y();
            if (Double.isNaN(x) || Double.isNaN(y) || x < 0 || y < 0 || x > Field.SIZE || y > Field.SIZE) continue;
            out.add(new double[]{x, y});
        }
        return out;
    }

    // ---- Groups ----

    /** Targets the intake takes in one pass: their indexes in the resolved list. */
    static final class Group {
        final List<Integer> members = new ArrayList<>();
        double x, y;

        int count() {
            return members.size();
        }
    }

    /**
     * Groups targets that one pass can take: a target joins a group only if it is
     * within {@code reach} of every member. With {@code inOrder}, only targets next
     * to each other in the list are grouped, so the order stays.
     */
    static List<Group> groups(List<double[]> targets, double reach, boolean inOrder) {
        List<Group> groups = new ArrayList<>();
        for (int i = 0; i < targets.size(); i++) {
            double[] t = targets.get(i);
            Group joined = null;
            int from = inOrder ? Math.max(0, groups.size() - 1) : 0;
            for (int g = from; g < groups.size() && joined == null; g++) {
                boolean fits = true;
                for (int m : groups.get(g).members) {
                    double[] o = targets.get(m);
                    if (Geometry.length(t[0] - o[0], t[1] - o[1]) > reach) fits = false;
                }
                if (fits) joined = groups.get(g);
            }
            if (joined == null) {
                joined = new Group();
                groups.add(joined);
            }
            joined.members.add(i);
        }
        for (Group group : groups) {
            double sx = 0, sy = 0;
            for (int m : group.members) {
                sx += targets.get(m)[0];
                sy += targets.get(m)[1];
            }
            group.x = sx / group.count();
            group.y = sy / group.count();
        }
        return groups;
    }

    // ---- The order ----

    /**
     * The quick cost of every hop. Index 0 is the start, 1..k the groups, k + 1 the
     * next waypoint (if any). seconds[a][b] is +infinity where there is no way.
     * inX/inY: the direction the hop arrives at b; outX/outY: the direction it leaves a.
     */
    static final class Hops {
        final double[][] seconds, length, inX, inY, outX, outY;

        Hops(int points) {
            seconds = new double[points][points];
            length = new double[points][points];
            inX = new double[points][points];
            inY = new double[points][points];
            outX = new double[points][points];
            outY = new double[points][points];
        }
    }

    /**
     * Quick time to drive a hop of this length, starting and ending at these
     * speeds: full acceleration up toward top speed and braking to the end speed
     * (a trapezoid), in/s and in/s^2.
     */
    static double hopSeconds(double length, double from, double to, double top, double accel) {
        from = Math.min(from, top);
        to = Math.min(to, top);
        // Distance to speed up from 'from' to v and brake from v to 'to': (2v^2 - from^2 - to^2) / (2 accel).
        double peak = Math.sqrt(Math.max((2 * accel * length + from * from + to * to) / 2, 0));
        if (peak <= top) return Math.max((2 * peak - from - to) / accel, 0);
        double ramps = (2 * top * top - from * from - to * to) / (2 * accel);
        return (2 * top - from - to) / accel + (length - ramps) / top;
    }

    /**
     * Extra time for turning by {@code turn} radians at a group while passing at
     * {@code speed}: slowing to the speed the turn allows and back up. Turning back
     * (past 120 degrees) means stopping: braking, turning in place, speeding up.
     */
    static double turnSeconds(double turn, double speed, double accel, double turnRate) {
        if (turn < Math.toRadians(30)) return 0;
        if (turn > Math.toRadians(120)) return 2 * speed / accel + turn / turnRate + 0.2;
        double slower = speed * Math.cos(turn);
        return 2 * (speed - slower) / accel;
    }

    /** An order to pick the groups up in: group numbers (0-based) in driving order, and its quick time. */
    static final class Order {
        final int[] groups;
        final double seconds;

        Order(int[] groups, double seconds) {
            this.groups = groups;
            this.seconds = seconds;
        }
    }

    /**
     * The best orders for the groups by quick time, best first, at most {@code howMany}.
     * Only orders taking at most {@code most} targets count, and of those, the ones
     * taking the most.
     *
     * @param hops    from Hops: index 0 the start, 1..k the groups, k + 1 the next waypoint;
     *                each hop's seconds already include speeding up and slowing down for its ends
     * @param hasNext whether a waypoint follows (else the last group is a stop)
     * @param speed   speed through a group (PICKUP_SPEED), in/s
     */
    static List<Order> orders(List<Group> groups, Hops hops, boolean hasNext, int most, boolean inOrder,
                              double speed, double accel, double turnRate, int howMany) {
        int k = groups.size();
        List<Order> result = new ArrayList<>();
        if (k == 0) return result;
        if (inOrder) {
            List<Integer> taken = new ArrayList<>();
            int count = 0;
            for (int g = 0; g < k; g++) {
                if (count + groups.get(g).count() > most) break;
                count += groups.get(g).count();
                taken.add(g);
            }
            int[] order = new int[taken.size()];
            for (int i = 0; i < order.length; i++) order[i] = taken.get(i);
            if (order.length > 0) result.add(new Order(order, 0));
            return result;
        }
        if (orderCount(k, groups, most) <= BRUTE_FORCE_ORDERS) {
            return everyOrder(groups, hops, hasNext, most, speed, accel, turnRate, howMany);
        }
        return heldKarp(groups, hops, hasNext, most, speed, accel, turnRate, howMany);
    }

    /**
     * Held-Karp: the quickest time into each group for each set of groups taken, then
     * the best finishes. Near-best with turn costs (see the class comment).
     */
    static List<Order> heldKarp(List<Group> groups, Hops hops, boolean hasNext, int most,
                                double speed, double accel, double turnRate, int howMany) {
        int k = groups.size();
        List<Order> result = new ArrayList<>();
        int subsets = 1 << k;
        double[][] best = new double[subsets][k];
        int[][] before = new int[subsets][k];
        int[] count = new int[subsets];
        for (int s = 1; s < subsets; s++) {
            int low = Integer.numberOfTrailingZeros(s);
            count[s] = count[s & (s - 1)] + groups.get(low).count();
        }
        for (double[] row : best) java.util.Arrays.fill(row, Double.POSITIVE_INFINITY);
        for (int g = 0; g < k; g++) {
            if (groups.get(g).count() > most) continue;
            best[1 << g][g] = hops.seconds[0][g + 1];
            before[1 << g][g] = -1;
        }
        for (int s = 1; s < subsets; s++) {
            for (int g = 0; g < k; g++) {
                double here = best[s][g];
                if (Double.isInfinite(here)) continue;
                for (int m = 0; m < k; m++) {
                    if ((s & (1 << m)) != 0 || count[s] + groups.get(m).count() > most) continue;
                    double hop = hops.seconds[g + 1][m + 1];
                    if (Double.isInfinite(hop)) continue;
                    double t = here + hop + turnSeconds(turnAt(hops, before[s][g], g, m + 1), speed, accel, turnRate);
                    int next = s | (1 << m);
                    if (t < best[next][m]) {
                        best[next][m] = t;
                        before[next][m] = g;
                    }
                }
            }
        }
        // The finish: on to the next waypoint, or stop at the last group. Only the orders taking the most count.
        int taken = 0;
        for (int s = 1; s < subsets; s++) {
            for (int g = 0; g < k; g++) if (!Double.isInfinite(best[s][g])) taken = Math.max(taken, count[s]);
        }
        List<double[]> finals = new ArrayList<>();
        for (int s = 1; s < subsets; s++) {
            if (count[s] != taken) continue;
            for (int g = 0; g < k; g++) {
                if (Double.isInfinite(best[s][g])) continue;
                double t = best[s][g];
                if (hasNext) {
                    double hop = hops.seconds[g + 1][k + 1];
                    if (Double.isInfinite(hop)) continue;
                    t += hop + turnSeconds(turnAt(hops, before[s][g], g, k + 1), speed, accel, turnRate);
                } else {
                    t += speed / accel;
                }
                finals.add(new double[]{t, s, g});
            }
        }
        Collections.sort(finals, (a, b) -> Double.compare(a[0], b[0]));
        for (int f = 0; f < finals.size() && result.size() < howMany; f++) {
            double seconds = finals.get(f)[0];
            int s = (int) finals.get(f)[1], g = (int) finals.get(f)[2];
            int[] order = new int[Integer.bitCount(s)];
            for (int i = order.length - 1; i >= 0; i--) {
                order[i] = g;
                int previous = before[s][g];
                s &= ~(1 << g);
                g = previous;
            }
            result.add(new Order(order, seconds));
        }
        return result;
    }

    /** Below this many orders, every one is scored (exact turns); above, Held-Karp. */
    static final int BRUTE_FORCE_ORDERS = 20000;

    /** How many orders (sequences of groups taking at most {@code most} targets) there are, up to a bit past BRUTE_FORCE_ORDERS. */
    private static long orderCount(int k, List<Group> groups, int most) {
        long total = 0, ways = 1;
        int smallest = Integer.MAX_VALUE;
        for (Group g : groups) smallest = Math.min(smallest, g.count());
        for (int length = 1; length <= k && length * smallest <= most; length++) {
            ways *= k - length + 1;
            total += ways;
            if (total > BRUTE_FORCE_ORDERS) break;
        }
        return total;
    }

    /** Every order scored with the quick times; the best {@code howMany} of those taking the most targets. */
    static List<Order> everyOrder(List<Group> groups, Hops hops, boolean hasNext, int most,
                                          double speed, double accel, double turnRate, int howMany) {
        int k = groups.size();
        List<Order> all = new ArrayList<>();
        int[] taken = new int[1];
        extend(groups, hops, hasNext, most, speed, accel, turnRate, new int[k], 0, new boolean[k], 0, 0, all, taken);
        List<Order> result = new ArrayList<>();
        Collections.sort(all, (a, b) -> Double.compare(a.seconds, b.seconds));
        for (Order order : all) {
            int count = 0;
            for (int g : order.groups) count += groups.get(g).count();
            if (count == taken[0] && result.size() < howMany) result.add(order);
        }
        return result;
    }

    /** Depth-first: adds every finished order (no group fits any more, or a waypoint follows) with its quick time. */
    private static void extend(List<Group> groups, Hops hops, boolean hasNext, int most, double speed, double accel, double turnRate,
                               int[] order, int length, boolean[] used, int count, double seconds, List<Order> out, int[] taken) {
        int k = groups.size();
        boolean grew = false;
        for (int m = 0; m < k; m++) {
            if (used[m] || count + groups.get(m).count() > most) continue;
            int from = length == 0 ? 0 : order[length - 1] + 1;
            double hop = hops.seconds[from][m + 1];
            if (Double.isInfinite(hop)) continue;
            double turn = length == 0 ? 0 : turnSeconds(turnAt(hops, length >= 2 ? order[length - 2] : -1, order[length - 1], m + 1), speed, accel, turnRate);
            used[m] = true;
            order[length] = m;
            grew = true;
            extend(groups, hops, hasNext, most, speed, accel, turnRate, order, length + 1, used, count + groups.get(m).count(),
                    seconds + hop + turn, out, taken);
            used[m] = false;
        }
        if (grew || length == 0) return;
        double t = seconds;
        int g = order[length - 1];
        if (hasNext) {
            double hop = hops.seconds[g + 1][k + 1];
            if (Double.isInfinite(hop)) return;
            t += hop + turnSeconds(turnAt(hops, length >= 2 ? order[length - 2] : -1, g, k + 1), speed, accel, turnRate);
        } else {
            t += speed / accel;
        }
        taken[0] = Math.max(taken[0], count);
        out.add(new Order(java.util.Arrays.copyOf(order, length), t));
    }

    /** Turn (radians) at group g, coming from group {@code from} (-1: the start) and going on to point {@code to}. */
    private static double turnAt(Hops hops, int from, int g, int to) {
        int a = from + 1;
        double ix = hops.inX[a][g + 1], iy = hops.inY[a][g + 1];
        double ox = hops.outX[g + 1][to], oy = hops.outY[g + 1][to];
        double dot = Math.max(-1, Math.min(1, ix * ox + iy * oy));
        return Math.acos(dot);
    }

    // ---- The capture test ----

    /**
     * Where the predicted path carries the target at (tx, ty) into the intake: the
     * first predicted point where it is at least INTAKE_DEPTH behind the intake front
     * (and no more than 1.5 in. further), within INTAKE_HALF_WIDTH less {@code margin}
     * of the center line, with the robot no faster than PICKUP_SPEED + 3 in/s. -1 if never.
     */
    static int capturedAt(double tx, double ty, double[] path, double[] heading, double[] speed, double margin) {
        double front = AgateFlow.ROBOT_FRONT, depth = AgateFlow.INTAKE_DEPTH, half = AgateFlow.INTAKE_HALF_WIDTH - margin;
        for (int i = 0; i < heading.length; i++) {
            double c = Math.cos(heading[i]), s = Math.sin(heading[i]);
            double dx = tx - path[2 * i], dy = ty - path[2 * i + 1];
            double ahead = dx * c + dy * s, side = -dx * s + dy * c;
            if (ahead <= front - depth && ahead >= front - depth - 1.5 && Math.abs(side) <= half
                    && (speed == null || speed[i] <= AgateFlow.PICKUP_SPEED + 3)) {
                return i;
            }
        }
        return -1;
    }
}
