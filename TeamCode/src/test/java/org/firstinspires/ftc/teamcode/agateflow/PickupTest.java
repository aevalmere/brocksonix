package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

/** Route.pickUp: grouping, ordering, the capture test, and picking up for real on TestRobot. */
public class PickupTest {
    private double timeLimit;

    @Before
    public void noTimeLimit() {
        timeLimit = AgateFlow.PLAN_TIME_LIMIT_MS;
        AgateFlow.PLAN_TIME_LIMIT_MS = 1e9;
    }

    @After
    public void putBack() {
        AgateFlow.PLAN_TIME_LIMIT_MS = timeLimit;
    }

    @Test
    public void closeTargetsGroupInOnePass() {
        List<double[]> targets = Arrays.asList(new double[]{10, 10}, new double[]{14, 10}, new double[]{40, 40}, new double[]{12, 13});
        List<Pickup.Group> groups = Pickup.groups(targets, 8, false);
        assertEquals(2, groups.size());
        assertEquals(3, groups.get(0).count());
        // In order, only neighbors in the list group.
        assertEquals(3, Pickup.groups(targets, 8, true).size());
    }

    @Test
    public void hopTimeIsATrapezoid() {
        // From a stop to a stop, short enough not to reach top speed: 2 * sqrt(length / accel).
        assertEquals(2 * Math.sqrt(10 / 100.0), Pickup.hopSeconds(10, 0, 0, 1000, 100), 1e-9);
        // Long: speeds up, cruises, slows down.
        double t = Pickup.hopSeconds(200, 0, 0, 50, 100);
        assertEquals(2 * 50 / 100.0 + (200 - 25) / 50.0, t, 1e-9);
        assertTrue(Pickup.hopSeconds(100, 30, 30, 60, 100) < Pickup.hopSeconds(100, 0, 0, 60, 100));
    }

    /** Hops between random points with straight-line lengths and directions. */
    private static Pickup.Hops straightHops(double[][] p, double top, double accel, double through) {
        int n = p.length;
        Pickup.Hops hops = new Pickup.Hops(n);
        for (int a = 0; a < n; a++) {
            for (int b = 1; b < n; b++) {
                if (a == b) {
                    hops.seconds[a][b] = Double.POSITIVE_INFINITY;
                    continue;
                }
                double dx = p[b][0] - p[a][0], dy = p[b][1] - p[a][1], l = Math.hypot(dx, dy);
                hops.length[a][b] = l;
                hops.inX[a][b] = dx / l;
                hops.inY[a][b] = dy / l;
                hops.outX[a][b] = dx / l;
                hops.outY[a][b] = dy / l;
                hops.seconds[a][b] = Pickup.hopSeconds(l, a == 0 ? 0 : through, b == n - 1 ? 0 : through, top, accel);
            }
        }
        return hops;
    }

    @Test
    public void ordersAreSortedAndTakeTheMost() {
        Random rnd = new Random(7);
        for (int trial = 0; trial < 20; trial++) {
            int k = 3 + rnd.nextInt(4);
            double[][] p = new double[k + 2][];
            for (int i = 0; i < p.length; i++) p[i] = new double[]{rnd.nextDouble() * 140, rnd.nextDouble() * 140};
            List<Pickup.Group> groups = new ArrayList<>();
            for (int g = 0; g < k; g++) {
                Pickup.Group group = new Pickup.Group();
                group.members.add(g);
                group.x = p[g + 1][0];
                group.y = p[g + 1][1];
                groups.add(group);
            }
            Pickup.Hops hops = straightHops(p, 60, 90, 30);
            int most = 1 + rnd.nextInt(k);
            List<Pickup.Order> orders = Pickup.orders(groups, hops, true, most, false, 30, 90, 6, 10);
            assertFalse(orders.isEmpty());
            for (int i = 0; i < orders.size(); i++) {
                assertEquals("takes exactly most() targets", Math.min(most, k), orders.get(i).groups.length);
                if (i > 0) assertTrue("sorted", orders.get(i).seconds >= orders.get(i - 1).seconds);
            }
        }
    }

    @Test
    public void heldKarpIsCloseToEveryOrder() {
        // 8 groups, all taken (40320 orders): Held-Karp against scoring every order, same hops and turns.
        // With turn costs it is near-best, not always the best (see Pickup's class comment).
        Random rnd = new Random(11);
        double worst = 0, total = 0;
        int exact = 0, trials = 30;
        for (int trial = 0; trial < trials; trial++) {
            int k = 8;
            double[][] p = new double[k + 2][];
            for (int i = 0; i < p.length; i++) p[i] = new double[]{rnd.nextDouble() * 140, rnd.nextDouble() * 140};
            List<Pickup.Group> groups = new ArrayList<>();
            for (int g = 0; g < k; g++) {
                Pickup.Group group = new Pickup.Group();
                group.members.add(g);
                group.x = p[g + 1][0];
                group.y = p[g + 1][1];
                groups.add(group);
            }
            Pickup.Hops hops = straightHops(p, 60, 90, 30);
            double best = Pickup.everyOrder(groups, hops, true, k, 30, 90, 6, 1).get(0).seconds;
            List<Pickup.Order> heldKarp = Pickup.heldKarp(groups, hops, true, k, 30, 90, 6, 10);
            assertEquals("takes every group", k, heldKarp.get(0).groups.length);
            for (int i = 1; i < heldKarp.size(); i++) assertTrue("sorted", heldKarp.get(i).seconds >= heldKarp.get(i - 1).seconds);
            double gap = heldKarp.get(0).seconds / best - 1;
            assertTrue("never better than the best order", gap > -1e-9);
            worst = Math.max(worst, gap);
            total += gap;
            if (gap < 1e-9) exact++;
        }
        assertTrue(String.format("worst %.1f%% slower", 100 * worst), worst < 0.10);
        assertTrue(String.format("mean %.2f%% slower", 100 * total / trials), total / trials < 0.02);
        assertTrue("the best order most of the time (" + exact + " of " + trials + ")", exact >= trials / 2);
    }

    @Test
    public void captureNeedsTheTargetInTheIntakeMouth() {
        // The robot at the origin facing +x, a target just inside the intake (INTAKE_DEPTH + 0.5 in.).
        double inside = AgateFlow.ROBOT_FRONT - AgateFlow.INTAKE_DEPTH - 0.5;
        double[] path = {0, 0}, heading = {0}, slow = {10}, fast = {AgateFlow.PICKUP_SPEED + 10};
        assertEquals(0, Pickup.capturedAt(inside, 0, path, heading, slow, 0));
        assertEquals(-1, Pickup.capturedAt(inside, 0, path, heading, fast, 0));
        assertEquals(-1, Pickup.capturedAt(inside, AgateFlow.INTAKE_HALF_WIDTH + 0.1, path, heading, slow, 0));
        assertEquals(-1, Pickup.capturedAt(AgateFlow.ROBOT_FRONT + 1, 0, path, heading, slow, 0));
    }

    @Test
    public void picksUpEveryTargetAndSaysSo() {
        Pose start = new Pose(20, 20, 0);
        Plan plan = AgateFlowTest.planSafely(start, new Route().pickUp(new Pose(50, 30, 0), new Pose(80, 40, 0), new Pose(60, 110, 0)), Alliance.RED);
        assertEquals(3, plan.pickedUp().size());
        assertTrue(plan.skipped().isEmpty());
    }

    @Test
    public void mostTakesTheQuickestAndSkipsTheRest() {
        Plan plan = AgateFlowTest.planSafely(new Pose(20, 20, 0),
                new Route().pickUp(new Pose(50, 30, 0), new Pose(80, 40, 0), new Pose(60, 110, 0)).most(2).to(36, 48), Alliance.RED);
        assertEquals(2, plan.pickedUp().size());
        assertEquals(1, plan.skipped().size());
        assertTrue("the far one is skipped", plan.skipped().get(0).y() > 100);
    }

    @Test
    public void inOrderKeepsTheOrder() {
        Plan plan = AgateFlowTest.planSafely(new Pose(20, 20, 0),
                new Route().pickUp(new Pose(60, 110, 0), new Pose(50, 30, 0), new Pose(80, 40, 0)).inOrder(), Alliance.RED);
        assertEquals(3, plan.pickedUp().size());
        assertTrue(plan.pickedUp().get(0).y() > 100);
    }

    @Test
    public void liveTargetsAreFieldCoordinatesAndBadOnesAreLeftOut() {
        final List<Pose> seen = new ArrayList<>();
        seen.add(new Pose(100, 30, 0));
        seen.add(null);
        seen.add(new Pose(Double.NaN, 40, 0));
        Pose start = Alliance.BLUE.fromRed(new Pose(20, 20, 0));
        Plan plan = AgateFlowTest.planSafely(start, new Route().pickUpLive(() -> seen).askLivePosesNow(), Alliance.BLUE);
        assertEquals(1, plan.pickedUp().size());
        assertEquals(100, plan.pickedUp().get(0).x(), 1e-9);
    }

    @Test
    public void reallyPicksThemUpOnTheRobot() {
        Pose start = new Pose(20, 20, 0);
        double[][] targets = {{50, 30}, {80, 40}, {60, 110}};
        Route route = new Route().pickUp(new Pose(50, 30, 0), new Pose(80, 40, 0), new Pose(60, 110, 0)).to(36, 48, 180);
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Plan plan = robot.plan(agateFlow, start, route, Alliance.RED);
        assertTrue(plan.toString(), plan.ok());
        // Drive it, and watch each target: it must go into the intake mouth at no more than PICKUP_SPEED + 3,
        // and never touch the robot's sides (outside the intake) before that.
        robot.place(start);
        boolean[] got = new boolean[targets.length];
        for (Leg leg : plan.legs) {
            if (leg.path != null) robot.follower.follow(leg.path);
            double t = 0, handoff = Double.NaN;
            while (t < 20) {
                robot.step();
                t += TestRobot.STEP;
                Pose p = robot.pose();
                double c = Math.cos(p.heading()), s = Math.sin(p.heading());
                for (int i = 0; i < targets.length; i++) {
                    double dx = targets[i][0] - p.x(), dy = targets[i][1] - p.y();
                    double ahead = dx * c + dy * s, side = -dx * s + dy * c;
                    if (got[i]) continue;
                    boolean alongside = ahead >= -AgateFlow.ROBOT_BACK - AgateFlow.BALL_RADIUS && ahead <= AgateFlow.ROBOT_FRONT
                            && Math.abs(side) > AgateFlow.INTAKE_HALF_WIDTH && Math.abs(side) < AgateFlow.ROBOT_HALF_WIDTH + AgateFlow.BALL_RADIUS;
                    assertFalse(String.format("target %d hit the robot's side at (%.1f, %.1f)", i, p.x(), p.y()), alongside);
                    if (ahead <= AgateFlow.ROBOT_FRONT - AgateFlow.INTAKE_DEPTH && ahead >= AgateFlow.ROBOT_FRONT - AgateFlow.INTAKE_DEPTH - 2
                            && Math.abs(side) <= AgateFlow.INTAKE_HALF_WIDTH) {
                        got[i] = true;
                        assertTrue(String.format("target %d taken at %.1f in/s", i, robot.speed()), robot.speed() <= AgateFlow.PICKUP_SPEED + 3);
                    }
                }
                if (!robot.follower.following() && Double.isNaN(handoff)) handoff = t;
                if (!Double.isNaN(handoff) && t - handoff > leg.settleSeconds() + 0.5) break;
            }
        }
        for (int i = 0; i < targets.length; i++) assertTrue("picked up target " + i, got[i]);
    }
}
