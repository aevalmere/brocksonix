package org.firstinspires.ftc.teamcode.util;

/**
 * Do two convex shapes overlap? The separating axis theorem: two convex shapes
 * are apart exactly when a straight line fits between them, and if one does,
 * one parallel to an edge of one of the shapes does too. So for every edge of
 * both shapes, both shapes are projected onto the direction straight across
 * that edge, where each one covers an interval. If any pair of intervals
 * doesn't meet, the shapes are apart. If every pair meets, they overlap.
 * Touching counts as overlapping.
 *
 * A shape is its corners in order around it, clockwise or counter-clockwise
 * (either works, so a mirrored shape needs no reordering): xs[i], ys[i] is
 * corner i. Convex shapes only: a shape with a dent can give a wrong answer.
 * A NaN corner counts as overlapping, so check for NaN first.
 */
public final class SeparatingAxis {
    private SeparatingAxis() {}

    /** True if the two convex shapes overlap or touch. Each needs at least 3 corners. */
    public static boolean overlaps(double[] ax, double[] ay, double[] bx, double[] by) {
        // Every edge of the first shape, then every edge of the second.
        for (int shape = 0; shape < 2; shape++) {
            double[] xs = shape == 0 ? ax : bx;
            double[] ys = shape == 0 ? ay : by;
            for (int i = 0; i < xs.length; i++) {
                int next = (i + 1) % xs.length;
                // Straight across the edge. Pointing in or out doesn't matter: both shapes use the same one.
                double nx = ys[next] - ys[i];
                double ny = xs[i] - xs[next];
                // Two corners in the same place: no edge there.
                if (nx == 0 && ny == 0) continue;
                // Strictly less: intervals that only touch still meet.
                if (highest(ax, ay, nx, ny) < lowest(bx, by, nx, ny)) return false;
                if (highest(bx, by, nx, ny) < lowest(ax, ay, nx, ny)) return false;
            }
        }
        return true;
    }

    /** Where the shape's interval starts along (nx, ny). */
    private static double lowest(double[] xs, double[] ys, double nx, double ny) {
        double lowest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) lowest = Math.min(lowest, xs[i] * nx + ys[i] * ny);
        return lowest;
    }

    /** Where the shape's interval ends along (nx, ny). */
    private static double highest(double[] xs, double[] ys, double nx, double ny) {
        double highest = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) highest = Math.max(highest, xs[i] * nx + ys[i] * ny);
        return highest;
    }
}
