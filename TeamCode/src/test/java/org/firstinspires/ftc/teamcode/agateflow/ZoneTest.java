package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

/** Zone geometry: distances, the shapes, and the outline's exact distance. */
public class ZoneTest {
    private static final double EPS = 1e-9;

    @Test
    public void rectangleDistanceIsNegativeInside() {
        Zone box = Zone.rectangle("box", 0, 0, 10, 10);
        assertEquals(-4, box.distance(5, 4), EPS);
        assertEquals(3, box.distance(13, 5), EPS);
        assertEquals(5, box.distance(13, 14), EPS);
    }

    @Test
    public void distanceAtLeastMatchesDistanceWhenItMatters() {
        Zone box = Zone.rectangle("box", 0, 0, 10, 10);
        assertEquals(box.distance(5, 4), box.distanceAtLeast(5, 4, -2), EPS);
        assertEquals(box.distance(30, 4), box.distanceAtLeast(30, 4, 100), EPS);
        assertTrue(box.distanceAtLeast(60, 60, 5) >= 5);
    }

    @Test
    public void lineIsABarOfTheGivenThickness() {
        Zone bar = Zone.line("bar", 10, 10, 40, 40, 4);
        assertTrue(bar.distance(25, 25) < 0);
        assertEquals(1, bar.distance(25 + 3 / Math.sqrt(2), 25 - 3 / Math.sqrt(2)), 1e-6);
        assertTrue(bar.distance(45, 45) > 0);
    }

    @Test
    public void fieldMapLeavesOutNullAndNaNZones() {
        FieldMap map = new FieldMap();
        map.setZones("vision", java.util.Arrays.asList(null, Zone.box("lost", new Pose(Double.NaN, 100, 0), 18, 18),
                Zone.box("partner", new Pose(70, 100, 0), 18, 18), Zone.circle("far", Double.POSITIVE_INFINITY, 5, 3)));
        // The 4 walls and the partner robot only.
        assertEquals(5, map.snapshot().zones.size());
        double room = map.snapshot().clearance(70, 70);
        assertTrue("clearance is a number", !Double.isNaN(room));
        assertEquals(21, room, 1e-9);
        map.setZones("vision", null);
        assertEquals(4, map.snapshot().zones.size());
    }

    @Test(expected = IllegalArgumentException.class)
    public void lineWithBothEndsTheSameIsRefused() {
        Zone.line("dot", 5, 5, 5, 5, 2);
    }

    @Test
    public void circleIsCoveredWhole() {
        Zone circle = Zone.circle("ball", 50, 50, 4);
        for (int i = 0; i < 64; i++) {
            double a = i * Math.PI / 32;
            assertTrue("a point on the circle is inside the zone", circle.distance(50 + 4 * Math.cos(a), 50 + 4 * Math.sin(a)) <= 1e-9);
        }
    }

    @Test
    public void boxIsCenteredAndTurned() {
        Zone box = Zone.box("robot", new Pose(20, 20, Math.PI / 4), 10, 10);
        assertTrue(box.distance(20, 20) < 0);
        assertEquals(-5, box.distance(20, 20), 1e-6);
    }

    @Test
    public void outlineDistanceIsExactSideOnAndFrontOn() {
        // A wall along x = 0; the robot 20 in. away.
        Zone wall = Zone.rectangle("wall", -10, -100, 0, 100);
        double frontOn = wall.outlineDistance(20, 0, Math.PI, AgateFlow.ROBOT_FRONT, AgateFlow.ROBOT_BACK, AgateFlow.ROBOT_HALF_WIDTH);
        double sideOn = wall.outlineDistance(20, 0, Math.PI / 2, AgateFlow.ROBOT_FRONT, AgateFlow.ROBOT_BACK, AgateFlow.ROBOT_HALF_WIDTH);
        double backOn = wall.outlineDistance(20, 0, 0, AgateFlow.ROBOT_FRONT, AgateFlow.ROBOT_BACK, AgateFlow.ROBOT_HALF_WIDTH);
        assertEquals(20 - AgateFlow.ROBOT_FRONT, frontOn, 1e-9);
        assertEquals(20 - AgateFlow.ROBOT_HALF_WIDTH, sideOn, 1e-9);
        assertEquals(20 - AgateFlow.ROBOT_BACK, backOn, 1e-9);
        // The tests' own judge agrees.
        assertEquals(frontOn, TrueRule.outlineDistance(wall, 20, 0, Math.PI), 1e-9);
    }

    @Test
    public void outlineDistanceIsNegativeWhenOverlapping() {
        Zone wall = Zone.rectangle("wall", -10, -100, 0, 100);
        double d = wall.outlineDistance(5, 0, Math.PI, AgateFlow.ROBOT_FRONT, AgateFlow.ROBOT_BACK, AgateFlow.ROBOT_HALF_WIDTH);
        assertEquals(5 - AgateFlow.ROBOT_FRONT, d, 1e-9);
    }
}
