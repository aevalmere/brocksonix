package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.Flower;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

/** Planning on the real field: routes like an auto's, safety, determinism, and the problems it reports. */
public class AgateFlowTest {
    private double timeLimit;
    private boolean moveTargets;

    @Before
    public void noTimeLimit() {
        // Every candidate is tried, so plans don't depend on how fast the computer is.
        timeLimit = AgateFlow.PLAN_TIME_LIMIT_MS;
        moveTargets = AgateFlow.MOVE_TARGETS_OUT_OF_ZONES;
        AgateFlow.PLAN_TIME_LIMIT_MS = 1e9;
    }

    @After
    public void putBack() {
        AgateFlow.PLAN_TIME_LIMIT_MS = timeLimit;
        AgateFlow.MOVE_TARGETS_OUT_OF_ZONES = moveTargets;
    }

    /** Plans, checks it planned and that neither the planned nor the predicted path breaks the true rule. */
    static Plan planSafely(Pose start, Route route, Alliance alliance) {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Plan plan = robot.plan(agateFlow, start, route, alliance);
        assertTrue("planned: " + plan, plan.ok());
        List<Zone> zones = agateFlow.map().snapshot().zones;
        double[] points = PlanChecks.points(start, plan, alliance);
        assertTrue("planned path keeps the true rule", PlanChecks.plannedRoom(plan, zones, AgateFlow.ROBOT_RADIUS, points, start.heading()) >= -0.01);
        assertTrue("predicted path keeps the true rule", PlanChecks.predictedRoom(plan, zones, AgateFlow.ROBOT_RADIUS, points, start.heading()) >= -0.01);
        return plan;
    }

    @Test
    public void autoRoutesPlanSafelyForBothAlliances() {
        Pose shoot = new Pose(36, 48, Math.PI);
        Route[] routes = {
                new Route().to(shoot),
                new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL))
                        .approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH).to(shoot),
                new Route().through(40, 100, 90).heading(Heading.tangent()).to(100, 120, 0).heading(Heading.finishBy(0.5)),
                new Route().to(120, 120).heading(Heading.tangent()),
        };
        for (Alliance alliance : Alliance.values()) {
            for (Route route : routes) planSafely(alliance.fromRed(new Pose(20, 24, 0)), route, alliance);
        }
    }

    @Test
    public void samePlanEveryTime() {
        Route route = new Route().through(40, 100, 90).heading(Heading.tangent()).to(100, 120, 0);
        Plan a = planSafely(new Pose(20, 20, 0), route, Alliance.RED), b = planSafely(new Pose(20, 20, 0), route, Alliance.RED);
        assertEquals(a.length(), b.length(), 1e-9);
        assertEquals(a.rawSeconds(), b.rawSeconds(), 1e-9);
    }

    @Test
    public void blueIsRedSpunAround() {
        Route route = new Route().to(100, 110, 0);
        Plan red = planSafely(new Pose(20, 20, 0), route, Alliance.RED);
        Plan blue = planSafely(Alliance.BLUE.fromRed(new Pose(20, 20, 0)), route, Alliance.BLUE);
        assertEquals(red.rawSeconds(), blue.rawSeconds(), 0.02 * red.rawSeconds());
        Pose end = blue.legs.get(blue.legs.size() - 1).end;
        assertEquals(Field.SIZE - 100, end.x(), 1e-6);
    }

    @Test
    public void startingAgainstAWallTurnsOnceClear() {
        // Facing along the audience wall, close enough that turning in place would swing a corner into it.
        Pose start = new Pose(36, AgateFlow.ROBOT_HALF_WIDTH + 1.5, 0);
        Plan plan = planSafely(start, new Route().to(70, 60).heading(Heading.tangent()), Alliance.RED);
        assertEquals(1, plan.legs.size());
    }

    @Test
    public void throughWaypointThatTurnsBackBecomesAStop() {
        Plan plan = planSafely(new Pose(20, 60, 0), new Route().through(80, 60, 0).to(30, 64, 180), Alliance.RED);
        assertEquals(2, plan.legs.size());
        assertTrue(plan.notes.toString(), plan.notes.toString().contains("turns back sharply"));
    }

    @Test
    public void targetInsideAZoneIsMovedOutOrRefused() {
        Pose inside = new Pose(48.4, 70.75, 0); // inside the red HIVE frame end
        Plan moved = planSafely(new Pose(20, 20, 0), new Route().to(inside), Alliance.RED);
        assertTrue(moved.notes.toString(), moved.notes.toString().contains("moved"));
        AgateFlow.MOVE_TARGETS_OUT_OF_ZONES = false;
        TestRobot robot = new TestRobot();
        Plan refused = robot.plan(robot.agateFlow(), new Pose(20, 20, 0), new Route().to(inside), Alliance.RED);
        assertFalse(refused.ok());
        assertTrue(refused.problem, refused.problem.contains("inside"));
    }

    @Test
    public void badInputsAreProblemsNotCrashes() {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        assertTrue(robot.plan(agateFlow, new Pose(Double.NaN, 20, 0), new Route().to(40, 40), Alliance.RED).problem.contains("NaN"));
        assertTrue(robot.plan(agateFlow, new Pose(20, 20, 0), new Route(), Alliance.RED).problem.contains("no waypoints"));
        assertTrue(robot.plan(agateFlow, new Pose(20, 20, 0), new Route().to(40, 40), null).problem.contains("alliance"));
        Plan noPose = robot.plan(agateFlow, new Pose(20, 20, 0), new Route().toLive(() -> null), Alliance.RED);
        assertTrue(noPose.problem, noPose.problem.contains("has no pose"));
    }

    @Test
    public void movingStartPlansFromTheMomentum() {
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Plan plan = agateFlow.plan(new Pose(30, 30, 0), new Velocity(40, 0, 0), new Route().to(30, 100, 90), Alliance.RED,
                TestRobot.VOLTS, DriveModel.capture(robot.follower));
        assertTrue(plan.toString(), plan.ok());
    }
}
