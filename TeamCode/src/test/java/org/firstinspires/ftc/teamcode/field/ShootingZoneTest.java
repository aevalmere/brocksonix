package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.agateflow.AgateFlow;
import org.firstinspires.ftc.teamcode.util.SeparatingAxis;
import org.junit.Test;

/**
 * ShootingZone: inside, far away, an outline over the edge with its center
 * outside, the outline's corners facing +y, and for every alliance and cell, a
 * zone on that cell's half around that alliance's HIVE, mirrored (not spun)
 * between them. The poses come from the zone's own corners, so retuning them in
 * Panels or the code doesn't break these.
 */
public class ShootingZoneTest {
    private static final double EPS = 1e-9;

    /** The middle of the zone: the average of its corners. */
    private static Pose middle(double[][] zone) {
        double x = 0, y = 0;
        for (int i = 0; i < zone[0].length; i++) {
            x += zone[0][i];
            y += zone[1][i];
        }
        return new Pose(x / zone[0].length, y / zone[0].length, 0);
    }

    /** True if the point is inside the zone, tested as a tiny square around it. */
    private static boolean pointInside(double[][] zone, double x, double y) {
        double d = 0.001;
        return SeparatingAxis.overlaps(zone[0], zone[1],
                new double[] {x - d, x + d, x + d, x - d}, new double[] {y - d, y - d, y + d, y + d});
    }

    @Test
    public void wellInsideIsIn() {
        for (Alliance alliance : Alliance.values()) {
            for (Cell cell : Cell.values()) {
                Pose middle = middle(ShootingZone.corners(alliance, cell));
                assertTrue(alliance + " " + cell, ShootingZone.overlaps(alliance, cell, middle));
            }
        }
    }

    @Test
    public void farAwayIsOut() {
        // The other half of the field, and the other alliance's side of our half.
        assertFalse(ShootingZone.overlaps(Alliance.RED, Cell.FAR, new Pose(Field.RED_HIVE_X, 20, 0)));
        assertFalse(ShootingZone.overlaps(Alliance.RED, Cell.FAR, new Pose(Field.SIZE - 15, 120, 0)));
        // The audience cell's zone, with the robot in the far cell's.
        Pose inFarZone = middle(ShootingZone.corners(Alliance.RED, Cell.FAR));
        assertFalse(ShootingZone.overlaps(Alliance.RED, Cell.AUDIENCE, inFarZone));
    }

    @Test
    public void anOutlineOverTheEdgeIsInWithItsCenterOutside() {
        double[][] zone = ShootingZone.corners(Alliance.RED, Cell.FAR);
        Pose middle = middle(zone);
        // The middle of the first edge (the short side, near the cell), and the way out of the zone from there.
        double ax = zone[0][0], ay = zone[1][0], bx = zone[0][1], by = zone[1][1];
        double edgeX = (ax + bx) / 2, edgeY = (ay + by) / 2;
        double length = Math.hypot(bx - ax, by - ay);
        double outX = (by - ay) / length, outY = -(bx - ax) / length;
        if (outX * (edgeX - middle.x()) + outY * (edgeY - middle.y()) < 0) {
            outX = -outX;
            outY = -outY;
        }
        // Driving along the edge, so the robot's side reaches ROBOT_HALF_WIDTH toward it.
        double heading = Math.atan2(by - ay, bx - ax);
        double half = AgateFlow.ROBOT_HALF_WIDTH;

        // Center half a side-width outside the edge: the side still reaches into the zone.
        double x = edgeX + outX * half / 2, y = edgeY + outY * half / 2;
        assertFalse("center outside", pointInside(zone, x, y));
        assertTrue("side over the edge", ShootingZone.overlaps(Alliance.RED, Cell.FAR, new Pose(x, y, heading)));

        // Twice the side-width outside: nothing reaches in.
        x = edgeX + outX * half * 2;
        y = edgeY + outY * half * 2;
        assertFalse("clear of the edge", ShootingZone.overlaps(Alliance.RED, Cell.FAR, new Pose(x, y, heading)));
    }

    @Test
    public void outlineTurnsWithTheHeading() {
        // Facing +y, the robot's left is -x: the front-left corner is up and to the left of
        // the turret center, the back-right one down and to the right. Heading 0 and the
        // mirrored zones can't catch a wrong rotation sign or front and back swapped; this can.
        double x = 40, y = 60;
        double[][] outline = ShootingZone.robotOutline(new Pose(x, y, Math.PI / 2));
        assertTrue("front-left", hasCorner(outline,
                x - AgateFlow.ROBOT_HALF_WIDTH, y + AgateFlow.ROBOT_FRONT));
        assertTrue("back-right", hasCorner(outline,
                x + AgateFlow.ROBOT_HALF_WIDTH, y - AgateFlow.ROBOT_BACK));
    }

    private static boolean hasCorner(double[][] outline, double x, double y) {
        for (int i = 0; i < outline[0].length; i++) {
            if (Math.abs(outline[0][i] - x) < EPS && Math.abs(outline[1][i] - y) < EPS) return true;
        }
        return false;
    }

    @Test
    public void eachZoneIsOnItsCellsHalfAroundItsAlliancesHive() {
        double center = Field.SIZE / 2;
        for (Alliance alliance : Alliance.values()) {
            for (Cell cell : Cell.values()) {
                double[][] zone = ShootingZone.corners(alliance, cell);
                double hiveX = Field.cellTarget(alliance, cell).x();
                double lowestX = Double.POSITIVE_INFINITY, highestX = Double.NEGATIVE_INFINITY;
                for (int i = 0; i < zone[0].length; i++) {
                    String what = alliance + " " + cell + " corner " + i;
                    if (cell == Cell.FAR) assertTrue(what, zone[1][i] > center);
                    else assertTrue(what, zone[1][i] < center);
                    lowestX = Math.min(lowestX, zone[0][i]);
                    highestX = Math.max(highestX, zone[0][i]);
                }
                assertTrue(alliance + " " + cell + " around the HIVE", lowestX < hiveX && hiveX < highestX);
            }
        }
    }

    @Test
    public void zonesAreMirroredNotSpun() {
        double[][] redFar = ShootingZone.corners(Alliance.RED, Cell.FAR);
        double[][] redAudience = ShootingZone.corners(Alliance.RED, Cell.AUDIENCE);
        double[][] blueFar = ShootingZone.corners(Alliance.BLUE, Cell.FAR);
        for (int i = 0; i < redFar[0].length; i++) {
            assertEquals(redFar[0][i], redAudience[0][i], EPS);
            assertEquals(Field.SIZE - redFar[1][i], redAudience[1][i], EPS);
            assertEquals(Field.SIZE - redFar[0][i], blueFar[0][i], EPS);
            assertEquals(redFar[1][i], blueFar[1][i], EPS);
        }
        // A robot mirrored across x = SIZE / 2 (heading too) is in or out the same for blue as for red.
        Pose red = new Pose(middle(redFar).x() + 12, middle(redFar).y() - 14, 0.4);
        Pose blue = new Pose(Field.SIZE - red.x(), red.y(), Math.PI - red.heading());
        assertEquals(ShootingZone.overlaps(Alliance.RED, Cell.FAR, red), ShootingZone.overlaps(Alliance.BLUE, Cell.FAR, blue));
    }
}
