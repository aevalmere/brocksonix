package org.firstinspires.ftc.teamcode.agateflow;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;
import org.junit.Test;

/** Route and Waypoint: flipping for blue, live poses, bad poses, headingless waypoints. */
public class RouteTest {
    @Test
    public void redPoseIsFlippedForBlue() {
        Waypoint w = new Route().to(30, 40, 90).approach(0, 10).waypoints().get(0);
        Waypoint.Resolved red = w.resolve(Alliance.RED), blue = w.resolve(Alliance.BLUE);
        assertEquals(30, red.x, 1e-9);
        assertEquals(Field.SIZE - 30, blue.x, 1e-9);
        assertEquals(Field.SIZE - 40, blue.y, 1e-9);
        assertEquals(Math.PI, blue.approachAngle, 1e-9);
    }

    @Test
    public void livePoseIsNotFlipped() {
        Waypoint w = new Route().toLive(() -> new Pose(30, 40, 0)).waypoints().get(0);
        assertEquals(30, w.resolve(Alliance.BLUE).x, 1e-9);
    }

    @Test
    public void nanOrInfiniteLivePoseCountsAsNoPose() {
        double[][] bad = {{40, 60, Double.NaN}, {Double.NaN, 60, 0}, {40, Double.NaN, 0}, {40, 60, Double.POSITIVE_INFINITY}};
        for (double[] b : bad) {
            final Pose live = new Pose(b[0], b[1], b[2]);
            assertNull(new Route().toLive(() -> live).waypoints().get(0).resolve(Alliance.RED));
        }
        assertNull(new Route().toLive(() -> null).waypoints().get(0).resolve(Alliance.RED));
        assertNull(new Route().toLive(() -> new Pose(-5, 40, 0)).waypoints().get(0).resolve(Alliance.RED));
    }

    @Test
    public void throwingSupplierCountsAsNoPose() {
        Route route = new Route().toLive(() -> {
            throw new IllegalStateException("camera unplugged");
        });
        assertNull(route.askLivePosesNow().waypoints().get(0).resolve(Alliance.RED));
    }

    @Test
    public void headinglessWaypointHasNaNHeading() {
        Waypoint.Resolved r = new Route().to(30, 40).waypoints().get(0).resolve(Alliance.RED);
        assertTrue(Double.isNaN(r.heading));
        assertTrue(r.stop);
        assertTrue(!new Route().through(30, 40).waypoints().get(0).resolve(Alliance.RED).stop);
    }

    @Test(expected = IllegalStateException.class)
    public void mostNeedsAPickUp() {
        new Route().to(30, 40).most(2);
    }

    @Test(expected = IllegalStateException.class)
    public void headingNeedsAWaypoint() {
        new Route().heading(Heading.tangent());
    }

    @Test
    public void pickUpTargetsFlipForBlueAndDropBadOnes() {
        Route route = new Route().pickUp(new Pose(30, 40, 0), new Pose(Double.NaN, 1, 0));
        Pickup pickup = route.waypoints().get(0).pickup;
        assertEquals(1, pickup.resolve(Alliance.RED).size());
        assertEquals(Field.SIZE - 30, pickup.resolve(Alliance.BLUE).get(0)[0], 1e-9);
    }
}
