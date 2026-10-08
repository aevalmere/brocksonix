package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Flower;
import org.firstinspires.ftc.teamcode.field.IntakePoses;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

/**
 * The robot's outline at the end of a stretch: it may start inside something (it is
 * where it is), but never end inside a zone or a wall, or swing into one on the way
 * in, unless the flower intake is meant to reach into a FLOWER. A stop whose outline
 * doesn't fit is moved out, a pickup pass that doesn't fit is left out, a turn in
 * place that would swing into a wall is refused, and a turn that would swing into a
 * wall right before a stop is finished earlier (holdEnd).
 */
public class OutlineFitTest {
    private double timeLimit, endHold;

    @Before
    public void noTimeLimit() {
        timeLimit = AgateFlow.PLAN_TIME_LIMIT_MS;
        endHold = AgateFlow.MAX_END_HOLD;
        AgateFlow.PLAN_TIME_LIMIT_MS = 1e9;
    }

    @After
    public void putBack() {
        AgateFlow.PLAN_TIME_LIMIT_MS = timeLimit;
        AgateFlow.MAX_END_HOLD = endHold;
    }

    private static boolean noted(Plan plan, String part) {
        for (String note : plan.notes) if (note.contains(part)) return true;
        return false;
    }

    /** Least exact room between the predicted outline and any zone or wall, more than an inch from the start. */
    private static double predictedOutlineRoom(Plan plan, List<Zone> zones, Pose start) {
        double least = Double.POSITIVE_INFINITY;
        for (Leg leg : plan.legs) {
            for (int q = 0; q < leg.predictedHeading.length; q++) {
                double x = leg.predictedPath[2 * q], y = leg.predictedPath[2 * q + 1];
                if (Math.hypot(x - start.x(), y - start.y()) < 1) continue;
                for (Zone zone : zones) least = Math.min(least, TrueRule.outlineDistance(zone, x, y, leg.predictedHeading[q]));
            }
        }
        return least;
    }

    @Test
    public void turnsEarlyRightBeforeAStopAgainstAWall() {
        // Facing the red wall, then a late turn to side-on against it: turning in the last 7 in.
        // would swing the front corners into the wall.
        Pose start = new Pose(40, 100, Math.PI);
        Route route = new Route().to(6.2, 100, 90).heading(Heading.between(0.8, 1));

        AgateFlow.MAX_END_HOLD = 0;
        TestRobot without = new TestRobot();
        assertFalse("without holdEnd there is no way in", without.plan(without.agateFlow(), start, route, Alliance.RED).ok());

        AgateFlow.MAX_END_HOLD = endHold;
        Plan plan = AgateFlowTest.planSafely(start, route, Alliance.RED);
        assertTrue(plan.notes.toString(), noted(plan, "Turns to its heading"));
        // Driven (Pedro's Foresight, with the plan's own heading interpolation): the outline
        // never goes into the wall, judged strictly (no relaxing near the stop).
        TestRobot robot = new TestRobot();
        AgateFlow agateFlow = robot.agateFlow();
        Plan again = robot.plan(agateFlow, start, route, Alliance.RED);
        TestRobot.Drive drive = robot.drive(start, again, agateFlow.map().snapshot().zones, AgateFlow.ROBOT_RADIUS,
                new double[]{start.x(), start.y()});
        assertTrue(drive.finished);
        assertTrue("driven outline room " + drive.worstRoom, drive.worstRoom >= -0.05);
    }

    @Test
    public void aStopTheRobotDoesNotFitIsMovedOut() {
        // Front-on to the red wall with its center 7 in. away: the intake (7.99 in.) would be in the wall.
        Pose start = new Pose(40, 100, 0);
        Plan plan = AgateFlowTest.planSafely(start, new Route().to(7, 100, 180), Alliance.RED);
        assertTrue(plan.notes.toString(), noted(plan, "so the robot fits there facing 180"));
        Leg last = plan.legs.get(plan.legs.size() - 1);
        // Moved out until the intake is END_SLACK clear of the wall (room to settle).
        assertEquals(AgateFlow.ROBOT_FRONT + ClearanceRule.END_SLACK, last.end.x(), 0.1);

        // The same next to the HIVE frame (a zone, not a wall).
        Plan frame = AgateFlowTest.planSafely(new Pose(20, 40, 0), new Route().to(39, 70, 0), Alliance.RED);
        assertTrue(frame.notes.toString(), noted(frame, "into HIVE frame"));
        TestRobot robot = new TestRobot();
        assertTrue(predictedOutlineRoom(frame, robot.agateFlow().map().snapshot().zones, new Pose(20, 40, 0)) >= -0.01);
    }

    @Test
    public void pickUpTargetsAgainstTheHiveFrameAreLeftOut() {
        // Balls 1.5 in. from the frame: the intake can't reach them without the robot hitting it.
        Pose start = new Pose(20, 40, 0);
        Route route = new Route().pickUp(new Pose(44.5, 62, 0), new Pose(44.5, 78, 0)).to(20, 110, 90);
        Plan plan = AgateFlowTest.planSafely(start, route, Alliance.RED);
        assertEquals(2, plan.skipped().size());
        assertTrue(plan.notes.toString(), noted(plan, "too close to HIVE frame"));
        TestRobot robot = new TestRobot();
        assertTrue(predictedOutlineRoom(plan, robot.agateFlow().map().snapshot().zones, start) >= -0.01);
    }

    @Test
    public void aTurnInPlaceThatWouldHitTheWallIsRefused() {
        // Side-on against the red wall, asked to face away: its back corners would swing into the wall.
        TestRobot robot = new TestRobot();
        Plan plan = robot.plan(robot.agateFlow(), new Pose(7, 100, Math.PI / 2), new Route().to(7, 100, 0), Alliance.RED);
        assertFalse(plan.ok());
        assertTrue(plan.problem, plan.problem.contains("Turning in place"));
        // Turning the other way round the same spot is fine (side-on to side-on, through facing away from the wall).
        Plan fine = robot.plan(robot.agateFlow(), new Pose(12, 100, Math.PI / 2), new Route().to(12, 100, 0), Alliance.RED);
        assertTrue(fine.toString(), fine.ok());
    }

    @Test
    public void leavingAFlowerBacksOutWhileTheIntakeComesUp() {
        // From the alliance-wall FLOWER (intake down, 2.4 in. into its opening) to spots all around:
        // the intake is still down for FLOWER_INTAKE_RAISE_RUN, so the robot backs straight out without
        // turning until the intake is out of the opening, and never sweeps it deeper into the FLOWER.
        Pose flower = IntakePoses.flowerPose(Flower.ALLIANCE_WALL);
        Pose start = new Pose(30, 40, 0);
        double[][] exits = {{40, 20, 0}, {30, 60, 90}, {70, 70, 0}, {60, 40, 0}, {9, 22, -90}, {9, 72, 90}};
        TestRobot robot = new TestRobot();
        List<Zone> zones = robot.agateFlow().map().snapshot().zones;
        for (double[] exit : exits) {
            Route route = new Route().to(flower).approach(IntakePoses.flowerTravelDegrees(Flower.ALLIANCE_WALL), IntakePoses.FLOWER_APPROACH)
                    .flowerIntakeDown().to(exit[0], exit[1], exit[2]);
            Plan plan = AgateFlowTest.planSafely(start, route, Alliance.RED);
            assertTrue(plan.notes.toString(), noted(plan, "Backs away with its heading kept"));
            Leg leave = plan.legs.get(plan.legs.size() - 1);
            double sx = leave.predictedPath[0], sy = leave.predictedPath[1], h0 = leave.predictedHeading[0];
            double backX = -Math.cos(h0), backY = -Math.sin(h0);
            for (int q = 0; q < leave.predictedHeading.length; q++) {
                double x = leave.predictedPath[2 * q], y = leave.predictedPath[2 * q + 1], dx = x - sx, dy = y - sy;
                double gone = Math.hypot(dx, dy);
                if (gone > AgateFlow.FLOWER_INTAKE_RAISE_RUN) break;
                if (gone <= IntakePoses.FLOWER_REACH_INTO_OPENING + 1) {
                    // Still in (or just out of) the opening: straight back, not turning.
                    String at = String.format(" leaving for (%.0f, %.0f) at (%.1f, %.1f)", exit[0], exit[1], x, y);
                    assertTrue("sideways" + at, Math.abs(-dx * backY + dy * backX) < 1);
                    assertTrue("turning" + at, Math.abs(org.firstinspires.ftc.teamcode.util.Angles.wrapRadians(leave.predictedHeading[q] - h0)) < Math.toRadians(1));
                }
                for (Zone zone : zones) {
                    if (zone.wall) continue;
                    double room = zone.outlineDistance(x, y, leave.predictedHeading[q], IntakePoses.FLOWER_INTAKE_REACH,
                            AgateFlow.ROBOT_BACK, AgateFlow.ROBOT_HALF_WIDTH);
                    assertTrue(String.format("intake %.2f into %s at (%.1f, %.1f)", -room, zone.name, x, y),
                            room >= -IntakePoses.FLOWER_REACH_INTO_OPENING - 0.05);
                }
            }
        }
    }

    @Test
    public void theFlowerIntakeStillReachesIntoTheFlower() {
        // Designed overlap: the flower intake reaches 2.4 in. into the FLOWER's opening.
        Plan plan = AgateFlowTest.planSafely(new Pose(36, 48, Math.PI),
                new Route().to(IntakePoses.flowerPose(Flower.ALLIANCE_WALL)).approach(180, 12).flowerIntakeDown(), Alliance.RED);
        Leg last = plan.legs.get(plan.legs.size() - 1);
        Pose flower = IntakePoses.flowerPose(Flower.ALLIANCE_WALL);
        assertEquals(flower.x(), last.end.x(), 1e-6);
        assertEquals(flower.y(), last.end.y(), 1e-6);
    }
}
