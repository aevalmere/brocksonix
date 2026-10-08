package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.FieldZones;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Random fields (the real field plus up to 3 random boxes and circles, a random
 * start, 1 to 3 waypoints with random heading rules, sometimes a moving start),
 * the same every run (fixed seed). Every plan must keep the true rule (TrueRule) on
 * the planned and the predicted path. Some are driven on TestRobot: no collision,
 * they finish, and the predicted time is close.
 */
public class SafetyFuzzTest {
    private static final int SCENES = 40, DRIVEN = 12;
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
    public void randomFieldsKeepTheTrueRule() {
        Random rnd = new Random(2026);
        int planned = 0, driven = 0;
        List<String> failures = new ArrayList<>();
        for (int id = 0; id < SCENES; id++) {
            Alliance alliance = rnd.nextBoolean() ? Alliance.RED : Alliance.BLUE;
            List<Zone> extra = new ArrayList<>();
            int count = rnd.nextInt(4);
            for (int k = 0; k < count; k++) {
                double cx = 15 + rnd.nextDouble() * (Field.SIZE - 30), cy = 15 + rnd.nextDouble() * (Field.SIZE - 30);
                if (rnd.nextInt(3) == 0) extra.add(Zone.circle("circle " + k, cx, cy, 3 + rnd.nextDouble() * 6));
                else extra.add(Zone.box("box " + k, new Pose(cx, cy, rnd.nextDouble() * Math.PI), 8 + rnd.nextDouble() * 16, 8 + rnd.nextDouble() * 16));
            }
            FieldMap map = new FieldMap();
            map.setZones("field", FieldZones.zones());
            map.setZones("extra", extra);
            FieldMap.Snapshot snap = map.snapshot();
            Pose start = free(rnd, snap, 11, null);
            Route route = new Route();
            int waypoints = 1 + rnd.nextInt(3);
            Pose last = start;
            for (int j = 0; j < waypoints; j++) {
                Pose p = free(rnd, snap, 12, last);
                if (j < waypoints - 1 && rnd.nextBoolean()) route.through(p);
                else route.to(p);
                int h = rnd.nextInt(4);
                if (h == 1) route.heading(Heading.tangent());
                if (h == 2) route.heading(Heading.finishBy(rnd.nextDouble()));
                if (rnd.nextInt(5) == 0) route.maxSpeed(15 + rnd.nextDouble() * 30);
                last = p;
            }
            double vx = 0, vy = 0;
            if (id % 3 == 1) {
                double speed = rnd.nextDouble() * 40, dir = rnd.nextDouble() * 2 * Math.PI;
                vx = speed * Math.cos(dir);
                vy = speed * Math.sin(dir);
            }
            // Built for red, spun for blue.
            Pose fieldStart = alliance.fromRed(start);
            if (alliance == Alliance.BLUE) {
                vx = -vx;
                vy = -vy;
            }
            List<Zone> fieldExtra = new ArrayList<>();
            for (Zone z : extra) fieldExtra.add(z.forAlliance(alliance));

            TestRobot robot = new TestRobot();
            FieldMap fieldMap = new FieldMap();
            fieldMap.setZones("field", FieldZones.zones());
            fieldMap.setZones("extra", fieldExtra);
            AgateFlow agateFlow = new AgateFlow(fieldMap, robot.follower);
            Plan plan = agateFlow.plan(fieldStart, new Velocity(vx, vy, 0), route, alliance, TestRobot.VOLTS, DriveModel.capture(robot.follower));
            if (!plan.ok()) continue;
            planned++;
            List<Zone> zones = fieldMap.snapshot().zones;
            double[] points = PlanChecks.points(fieldStart, plan, alliance);
            double plannedRoom = PlanChecks.plannedRoom(plan, zones, AgateFlow.ROBOT_RADIUS, points, fieldStart.heading());
            double predictedRoom = PlanChecks.predictedRoom(plan, zones, AgateFlow.ROBOT_RADIUS, points, fieldStart.heading());
            if (plannedRoom < -0.01 || predictedRoom < -0.01) {
                failures.add(String.format("scene %d: planned room %.2f, predicted room %.2f", id, plannedRoom, predictedRoom));
            }
            if (vx == 0 && vy == 0 && driven < DRIVEN) {
                driven++;
                TestRobot.Drive d = robot.drive(fieldStart, plan, zones, AgateFlow.ROBOT_RADIUS, points);
                double error = (d.seconds - plan.rawSeconds()) / plan.rawSeconds();
                if (!d.finished || d.worstRoom < -0.5 || Math.abs(error) > 0.15) {
                    failures.add(String.format("scene %d driven: finished %s, worst room %.2f, time %.2f s vs %.2f predicted",
                            id, d.finished, d.worstRoom, d.seconds, plan.rawSeconds()));
                }
            }
        }
        assertTrue("most scenes plan (" + planned + " of " + SCENES + ")", planned >= SCENES * 0.8);
        assertTrue(failures.toString(), failures.isEmpty());
    }

    /** A random spot at least {@code room} from every zone (and 10 in. from {@code awayFrom}), written for red. */
    private static Pose free(Random rnd, FieldMap.Snapshot snap, double room, Pose awayFrom) {
        while (true) {
            double x = 3 + rnd.nextDouble() * (Field.SIZE - 6), y = 3 + rnd.nextDouble() * (Field.SIZE - 6);
            if (snap.clearance(x, y) < room) continue;
            if (awayFrom != null && Math.hypot(x - awayFrom.x(), y - awayFrom.y()) < 10) continue;
            return new Pose(x, y, (rnd.nextDouble() * 2 - 1) * Math.PI);
        }
    }
}
