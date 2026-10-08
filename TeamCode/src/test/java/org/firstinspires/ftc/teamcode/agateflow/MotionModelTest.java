package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * The predicted time against TestRobot (Pedro's Foresight on DriveModel's physics),
 * driven: straight lines in every direction, turning on the way, and corners
 * driven facing forward. In an empty field so only the motion counts.
 */
public class MotionModelTest {
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

    /** Predicted and driven seconds for a route from start in an empty field (walls only). */
    private static double[] predictedAndDriven(Pose start, Route route) {
        TestRobot robot = new TestRobot();
        FieldMap walls = new FieldMap();
        walls.setZones("none", new ArrayList<Zone>());
        AgateFlow agateFlow = new AgateFlow(walls, robot.follower);
        Plan plan = robot.plan(agateFlow, start, route, Alliance.RED);
        assertTrue(plan.toString(), plan.ok());
        List<Zone> zones = walls.snapshot().zones;
        TestRobot.Drive d = robot.drive(start, plan, zones, AgateFlow.ROBOT_RADIUS, PlanChecks.points(start, plan, Alliance.RED));
        assertTrue(d.finished);
        return new double[]{plan.rawSeconds(), d.seconds};
    }

    private static void assertClose(String what, double[] times, double fraction) {
        assertEquals(what + String.format(" (predicted %.2f s, driven %.2f s)", times[0], times[1]), times[1], times[0], fraction * times[1]);
    }

    @Test
    public void straightLinesAnyWay() {
        for (double angle : new double[]{0, 45, 90, 180}) {
            for (double length : new double[]{24, 48, 96}) {
                double travel = Math.toRadians(90 + angle);
                Pose start = new Pose(72 - Math.cos(travel) * length / 2, 72 - Math.sin(travel) * length / 2, Math.PI / 2);
                Route route = new Route().to(start.x() + Math.cos(travel) * length, start.y() + Math.sin(travel) * length, 90);
                assertClose(angle + " degrees, " + length + " in.", predictedAndDriven(start, route), 0.04);
            }
        }
    }

    @Test
    public void turningToFaceTheWayItDrives() {
        for (double turn : new double[]{22, 90, 170}) {
            for (double length : new double[]{24, 96}) {
                Pose start = new Pose(72, 72 - length / 2, Math.toRadians(90 - turn));
                Route route = new Route().to(72, 72 + length / 2).heading(Heading.tangent());
                assertClose("turn " + turn + " degrees, " + length + " in.", predictedAndDriven(start, route), 0.04);
            }
        }
    }

    @Test
    public void cornersFacingForward() {
        for (double turn : new double[]{30, 60, 90, 120}) {
            double t = Math.toRadians(turn);
            Pose start = new Pose(24, 40, 0);
            Route route = new Route().through(72, 40).heading(Heading.tangent()).to(72 + Math.cos(t) * 48, 40 + Math.sin(t) * 48).heading(Heading.tangent());
            assertClose("corner " + turn + " degrees", predictedAndDriven(start, route), 0.05);
        }
    }
}
