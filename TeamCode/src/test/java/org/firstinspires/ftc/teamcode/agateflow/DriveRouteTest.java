package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Flower;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.firstinspires.ftc.teamcode.robot.RobotState;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Future;

/**
 * DriveRoute on the simulated robot, run like an auto on a virtual clock: replanning
 * around a zone that shows up ahead (and the time a detour gets), a zone that lands
 * on a stop, the deadline and its cap, overtime for a robot still getting there, a
 * replan that comes back too late, and a replan just after a flowerIntakeDown stop.
 */
public class DriveRouteTest {
    /** Up the red side of the field, clear of the red wall's FLOWER and the HIVE frame. */
    private static final Pose START = new Pose(24, 16, Math.PI / 2);
    private static final Pose END = new Pose(24, 124, Math.PI / 2);
    /** A partner robot that stops in our way, about halfway. */
    private static final Zone PARTNER = Zone.circle("partner", 24, 75, 6);

    private TestRobot.Saved saved;
    private TestRobot robot;
    private AgateFlowCommands agateFlow;

    @Before
    public void setUp() {
        saved = TestRobot.save();
        RobotState.alliance = Alliance.RED;
        EtaCalibration.LEARN = false;
        robot = new TestRobot();
        robot.useVirtualClock();
        robot.place(START);
        agateFlow = new AgateFlowCommands(robot.follower, () -> TestRobot.VOLTS, new Object());
    }

    @After
    public void putBack() {
        saved.restore();
    }

    /** What one drive did, watched loop by loop. */
    private final class Watch {
        final DriveRoute drive;
        final List<Plan> plans = new ArrayList<>();
        /** Virtual time the first plan was taken, and the drive ended, seconds. */
        double firstPlanAt = Double.NaN, endedAt = Double.NaN;

        Watch(DriveRoute drive) {
            this.drive = drive;
        }

        /** Runs the drive like an auto until it ends (at most 30 s), calling {@code each} before every loop. */
        Watch run(Runnable each) {
            Scheduler.schedule(drive);
            for (int i = 0; i < 3000 && Scheduler.isScheduled(drive); i++) {
                if (each != null) each.run();
                robot.loop();
                see();
            }
            return this;
        }

        /** After a loop: the drive's work happened at the clock before the 10 ms step. */
        void see() {
            Plan plan = drive.plan();
            double then = robot.seconds() - TestRobot.STEP;
            if (plan != null && (plans.isEmpty() || plans.get(plans.size() - 1) != plan)) plans.add(plan);
            if (plan != null && Double.isNaN(firstPlanAt)) firstPlanAt = then;
            if (!Scheduler.isScheduled(drive) && Double.isNaN(endedAt)) endedAt = then;
        }

        /** Seconds the first plan allowed (TIMEOUT_SCALE x estimate + TIMEOUT_EXTRA_SECONDS). */
        double firstAllowed() {
            return plans.get(0).seconds() * AgateFlowCommands.TIMEOUT_SCALE + AgateFlowCommands.TIMEOUT_EXTRA_SECONDS;
        }
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    private void showPartner() {
        agateFlow.map().setZones("vision", Collections.singletonList(PARTNER));
    }

    /** Short deadlines with no slack: a drive that stops for a block can't make the first one. */
    private static void tightDeadlines() {
        AgateFlowCommands.TIMEOUT_SCALE = 1.1;
        AgateFlowCommands.TIMEOUT_EXTRA_SECONDS = 0.3;
        AgateFlowCommands.REPLAN_EXTRA_SECONDS = 0;
        AgateFlowCommands.MAX_OVERTIME_SECONDS = 0;
    }

    /** Drives START to END and shows PARTNER in the way once the robot is going fast. */
    private Watch driveIntoPartner(final double[] slowestBeforeReplan, final double[] closest) {
        final Watch watch = new Watch(agateFlow.driveTo(new Route().to(END)));
        final boolean[] shown = {false};
        return watch.run(() -> {
            Pose pose = robot.pose();
            if (!shown[0] && pose.y() > 40) {
                showPartner();
                shown[0] = true;
            }
            if (!shown[0]) return;
            closest[0] = Math.min(closest[0], PARTNER.distance(pose.x(), pose.y()));
            if (watch.plans.size() == 1) slowestBeforeReplan[0] = Math.min(slowestBeforeReplan[0], robot.speed());
        });
    }

    @Test
    public void aNewZoneAheadMakesItBrakeHoldAndDetourWithTheDetoursOwnTime() {
        tightDeadlines();
        double[] slowest = {Double.POSITIVE_INFINITY}, closest = {Double.POSITIVE_INFINITY};
        Watch watch = driveIntoPartner(slowest, closest);
        DriveRoute drive = watch.drive;
        assertEquals(drive.problem(), DriveRoute.Status.ARRIVED, drive.status());
        assertTrue("arrived " + distance(robot.pose(), END) + " in. from the stop", distance(robot.pose(), END) < 1.5);
        // It stopped for the block (nearly still) before the new plan took over, and drove round it.
        assertTrue("slowest before the replan " + slowest[0] + " in/s", slowest[0] < 3);
        assertTrue("plans " + watch.plans.size(), watch.plans.size() >= 2);
        assertTrue("came " + closest[0] + " in. from the partner", closest[0] >= AgateFlow.ROBOT_RADIUS);
        // The detour got its own time: the drive took longer than the first plan allowed, and still arrived.
        assertEquals(1, drive.detours());
        double took = watch.endedAt - watch.firstPlanAt;
        assertTrue(String.format("took %.2f s, the first plan allowed %.2f s", took, watch.firstAllowed()), took > watch.firstAllowed());
    }

    @Test
    public void withoutDetourTimeTheSameDriveTimesOut() {
        tightDeadlines();
        AgateFlowCommands.MAX_DETOURS = 0;
        double[] slowest = {Double.POSITIVE_INFINITY}, closest = {Double.POSITIVE_INFINITY};
        Watch watch = driveIntoPartner(slowest, closest);
        assertEquals(DriveRoute.Status.TIMED_OUT, watch.drive.status());
        assertEquals(0, watch.drive.detours());
        // At the first deadline (the cap, with REPLAN_EXTRA_SECONDS 0), give or take a loop.
        assertEquals(watch.firstAllowed(), watch.endedAt - watch.firstPlanAt, 0.025);
    }

    @Test
    public void aPartnerParkedOnTheStopMovesTheStop() {
        // Just past our stop, where the robot's front would end up inside it. The points the robot drives
        // through more than ROBOT_RADIUS + SAFETY_MARGIN before the stop all keep their room, so only
        // the check near the stop can see it.
        final Zone parked = Zone.circle("partner", END.x(), END.y() + 7, 5);
        final Watch watch = new Watch(agateFlow.driveTo(new Route().to(END)));
        final boolean[] shown = {false};
        final double[] room = {Double.POSITIVE_INFINITY};
        watch.run(() -> {
            Pose pose = robot.pose();
            // The robot is going fast, about 30 in. from the stop.
            if (!shown[0] && pose.y() > END.y() - 30) {
                agateFlow.map().setZones("vision", Collections.singletonList(parked));
                shown[0] = true;
            }
            room[0] = Math.min(room[0], parked.outlineDistance(pose.x(), pose.y(), pose.heading(),
                    AgateFlow.ROBOT_FRONT, AgateFlow.ROBOT_BACK, AgateFlow.ROBOT_HALF_WIDTH));
        });
        DriveRoute drive = watch.drive;
        assertEquals(drive.problem(), DriveRoute.Status.ARRIVED, drive.status());
        assertTrue("plans " + watch.plans.size(), watch.plans.size() >= 2);
        assertTrue(drive.plan().notes.toString(), drive.plan().notes.toString().contains("moved"));
        assertTrue("the robot's outline came " + room[0] + " in. from the partner", room[0] >= 0);
        assertTrue("still at the old stop", distance(robot.pose(), END) > 2);
    }

    @Test
    public void replansForPushesEndAtTheCap() {
        // Pushed sideways and back every 0.6 s, so it never gets there: each push replans, and the
        // deadline can move at most REPLAN_EXTRA_SECONDS past the first one.
        AgateFlowCommands.REPLAN_EXTRA_SECONDS = 1.5;
        AgateFlowCommands.MAX_OVERTIME_SECONDS = 0;
        final Watch watch = new Watch(agateFlow.driveTo(new Route().to(END)));
        final int[] pushes = {0};
        watch.run(() -> {
            if (Double.isNaN(watch.firstPlanAt)) return;
            double since = robot.seconds() - watch.firstPlanAt;
            if (since > 0.6 * (pushes[0] + 1)) {
                pushes[0]++;
                robot.place(new Pose(pushes[0] % 2 == 1 ? 38 : 24, 30, Math.PI / 2));
            }
        });
        DriveRoute drive = watch.drive;
        assertEquals(DriveRoute.Status.TIMED_OUT, drive.status());
        assertTrue("plans " + watch.plans.size(), watch.plans.size() >= 3);
        assertEquals(0, drive.detours());
        double cap = watch.firstAllowed() + AgateFlowCommands.REPLAN_EXTRA_SECONDS;
        assertEquals(cap, watch.endedAt - watch.firstPlanAt, 0.025);
    }

    @Test
    public void aRobotStillGettingThereGetsOvertime() {
        // The deadline comes halfway through the drive.
        AgateFlowCommands.TIMEOUT_SCALE = 0.5;
        AgateFlowCommands.TIMEOUT_EXTRA_SECONDS = 0;
        AgateFlowCommands.MAX_OVERTIME_SECONDS = 3;
        Watch watch = new Watch(agateFlow.driveTo(new Route().to(END))).run(null);
        assertEquals(watch.drive.problem(), DriveRoute.Status.ARRIVED, watch.drive.status());
        double took = watch.endedAt - watch.firstPlanAt;
        assertTrue(String.format("took %.2f s, allowed %.2f s", took, watch.firstAllowed()),
                took > watch.firstAllowed() + 0.3 && took < watch.firstAllowed() + AgateFlowCommands.MAX_OVERTIME_SECONDS);
    }

    @Test
    public void aStuckRobotGetsNoOvertime() {
        AgateFlowCommands.TIMEOUT_SCALE = 0.5;
        AgateFlowCommands.TIMEOUT_EXTRA_SECONDS = 0;
        AgateFlowCommands.MAX_OVERTIME_SECONDS = 3;
        final Watch watch = new Watch(agateFlow.driveTo(new Route().to(END)));
        final Pose[] stuck = {null};
        watch.run(() -> {
            if (Double.isNaN(watch.firstPlanAt)) return;
            // Stuck from the deadline on (held where it is, like against another robot).
            if (stuck[0] == null && robot.seconds() - watch.firstPlanAt > watch.firstAllowed()) stuck[0] = robot.pose();
            if (stuck[0] != null) robot.place(stuck[0]);
        });
        assertEquals(DriveRoute.Status.TIMED_OUT, watch.drive.status());
        // It gives up once it has made no progress for half a second, not after MAX_OVERTIME_SECONDS.
        double over = watch.endedAt - watch.firstPlanAt - watch.firstAllowed();
        assertTrue("timed out " + over + " s past the deadline", over < 0.6);
    }

    @Test
    public void aReplanThatComesBackAfterTheNextLegStartedIsDropped() throws Exception {
        final Pose stop = new Pose(24, 60, Math.PI / 2);
        DriveRoute drive = agateFlow.driveTo(new Route().to(stop).to(END));
        Scheduler.schedule(drive);
        for (int i = 0; i < 300 && (drive.plan() == null || robot.pose().y() < 30); i++) robot.loop();
        Plan first = drive.plan();
        assertEquals(0, drive.legIndex());

        // Hold up the planning thread, then push the robot off its path: the replan waits in line.
        final CountDownLatch release = new CountDownLatch(1);
        Future<Object> blocker = AgateFlowCommands.WORKER.submit(() -> {
            release.await();
            return null;
        });
        try {
            robot.place(new Pose(36, robot.pose().y(), Math.PI / 2));
            for (int i = 0; i < 100 && !drive.planPending(); i++) {
                Scheduler.execute();
                robot.step();
            }
            assertTrue("asked for a replan", drive.planPending());
            assertEquals(0, drive.legIndex());
            // The old plan still drives the robot to the first stop and on to the next leg.
            for (int i = 0; i < 1000 && drive.legIndex() == 0; i++) {
                Scheduler.execute();
                robot.step();
            }
            assertEquals(1, drive.legIndex());
        } finally {
            release.countDown();
        }
        blocker.get();
        TestRobot.waitForPlans();
        // The replan starts with the first stop, which the robot has passed: it is dropped.
        robot.loop();
        assertSame(first, drive.plan());
        assertEquals(1, drive.legIndex());
        assertFalse(drive.planPending());
        for (int i = 0; i < 3000 && Scheduler.isScheduled(drive); i++) robot.loop();
        assertEquals(drive.problem(), DriveRoute.Status.ARRIVED, drive.status());
        assertTrue(distance(robot.pose(), END) < 1.5);
    }

    @Test
    public void aRouteThatStartsWithTheFlowerIntakeDownBacksOut() {
        // What a replan just after a flowerIntakeDown stop (or ParkGuard at a FLOWER) asks for.
        Pose flower = IntakePoses.flowerPose(Flower.ALLIANCE_WALL);
        Route shoot = new Route().to(36, 48, 180);
        Plan down = robot.plan(robot.agateFlow(), flower, shoot.startingFlowerDown(true), Alliance.RED);
        Plan up = robot.plan(robot.agateFlow(), flower, shoot, Alliance.RED);
        assertTrue(down.toString(), down.ok());
        assertTrue(down.notes.toString(), down.notes.toString().contains("Backs away with its heading kept"));
        assertFalse(up.notes.toString(), up.notes.toString().contains("Backs away with its heading kept"));
        // The flag is on a copy, and survives asking for live poses (as every plan does).
        assertFalse(shoot.startFlowerDown);
        assertNotSame(shoot, shoot.startingFlowerDown(true));
        assertSame(shoot, shoot.startingFlowerDown(false));
        assertTrue(shoot.startingFlowerDown(true).askLivePosesNow().startFlowerDown);
    }
}
