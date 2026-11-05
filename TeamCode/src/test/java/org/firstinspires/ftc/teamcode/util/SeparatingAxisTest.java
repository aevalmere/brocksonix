package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * SeparatingAxis: overlapping, apart, touching, one inside the other, either
 * winding order, and a diamond off a trapezoid's corner, where the bounding
 * boxes overlap but a slanted edge separates the shapes.
 */
public class SeparatingAxisTest {

    /** Corners {x0, y0, x1, y1, ...} as the x array. */
    private static double[] xs(double... corners) {
        double[] xs = new double[corners.length / 2];
        for (int i = 0; i < xs.length; i++) xs[i] = corners[2 * i];
        return xs;
    }

    private static double[] ys(double... corners) {
        double[] ys = new double[corners.length / 2];
        for (int i = 0; i < ys.length; i++) ys[i] = corners[2 * i + 1];
        return ys;
    }

    /** Same corners, the other way round. */
    private static double[] reversed(double[] values) {
        double[] out = new double[values.length];
        for (int i = 0; i < values.length; i++) out[i] = values[values.length - 1 - i];
        return out;
    }

    private static boolean overlaps(double[] a, double[] b) {
        return SeparatingAxis.overlaps(xs(a), ys(a), xs(b), ys(b));
    }

    /** The answer must not depend on the order of the shapes or on either one's winding. */
    private static boolean overlapsEveryWay(double[] a, double[] b) {
        boolean answer = overlaps(a, b);
        double[] ax = xs(a), ay = ys(a), bx = xs(b), by = ys(b);
        assertEquals("shapes swapped", answer, SeparatingAxis.overlaps(bx, by, ax, ay));
        assertEquals("first reversed", answer, SeparatingAxis.overlaps(reversed(ax), reversed(ay), bx, by));
        assertEquals("second reversed", answer, SeparatingAxis.overlaps(ax, ay, reversed(bx), reversed(by)));
        assertEquals("both reversed, swapped", answer, SeparatingAxis.overlaps(reversed(bx), reversed(by), reversed(ax), reversed(ay)));
        return answer;
    }

    private static final double[] SQUARE = {0, 0, 4, 0, 4, 4, 0, 4};

    @Test
    public void overlappingSquaresOverlap() {
        assertTrue(overlapsEveryWay(SQUARE, new double[] {2, 2, 6, 2, 6, 6, 2, 6}));
    }

    @Test
    public void separatedSquaresDoNot() {
        assertFalse(overlapsEveryWay(SQUARE, new double[] {5, 0, 9, 0, 9, 4, 5, 4}));
        assertFalse(overlapsEveryWay(SQUARE, new double[] {0, 4.01, 4, 4.01, 4, 8, 0, 8}));
    }

    @Test
    public void touchingCountsAsOverlapping() {
        // Sharing an edge.
        assertTrue(overlapsEveryWay(SQUARE, new double[] {4, 0, 8, 0, 8, 4, 4, 4}));
        // Sharing only a corner.
        assertTrue(overlapsEveryWay(SQUARE, new double[] {4, 4, 8, 4, 8, 8, 4, 8}));
    }

    @Test
    public void oneInsideTheOtherOverlaps() {
        assertTrue(overlapsEveryWay(SQUARE, new double[] {1, 1, 2, 1, 2, 2, 1, 2}));
    }

    @Test
    public void aDiamondOffATrapezoidsCornerIsApartThoughTheBoxesOverlap() {
        // Trapezoid: bottom 0 to 10, top 3 to 7, 6 tall. Its right edge runs from (10, 0) to (7, 6).
        double[] trapezoid = {0, 0, 10, 0, 7, 6, 3, 6};
        // A square turned 45°, centered at (10, 6): x 8 to 12, y 4 to 8. Its box overlaps the
        // trapezoid's (x 0 to 10, y 0 to 6), but all of it is outside the slanted right edge.
        double[] diamond = {12, 6, 10, 8, 8, 6, 10, 4};
        assertFalse(overlapsEveryWay(trapezoid, diamond));
        // Moved in toward the trapezoid, its left corner crosses that edge.
        double[] movedIn = {10.5, 5, 8.5, 7, 6.5, 5, 8.5, 3};
        assertTrue(overlapsEveryWay(trapezoid, movedIn));
    }

    @Test
    public void aTriangleWorksToo() {
        double[] triangle = {0, 0, 4, 0, 0, 4};
        // Just past the slanted side x + y = 4.
        assertFalse(overlapsEveryWay(triangle, new double[] {2.1, 2.1, 4, 2.1, 4, 4, 2.1, 4}));
        assertTrue(overlapsEveryWay(triangle, new double[] {1.9, 1.9, 4, 1.9, 4, 4, 1.9, 4}));
    }
}
