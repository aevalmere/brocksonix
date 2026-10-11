package org.firstinspires.ftc.teamcode.agateflow;

/** Small math helpers for AgateFlow. */
final class Geometry {
    private Geometry() {}

    /**
     * Length of (x, y). Same as Math.hypot, but several times faster: Math.hypot
     * guards against numbers too big to square, which never happen on a field,
     * and it was most of AgateFlow's planning time.
     */
    static double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * The angle wrapped to -pi..pi, like Angles.wrapRadians but quicker
     * (Math.IEEEremainder is slow, and MotionModel wraps every 10 ms step).
     */
    static double wrap(double radians) {
        return radians - 2 * Math.PI * Math.floor((radians + Math.PI) / (2 * Math.PI));
    }

    /**
     * Distance from (x, y) to the nearest point of the line through {@code path}'s
     * points (x, y pairs, like a Leg's predicted path). +infinity if it has fewer
     * than two points.
     */
    static double distanceToPolyline(double[] path, double x, double y) {
        double best = Double.POSITIVE_INFINITY;
        for (int q = 0; q + 3 < path.length; q += 2) {
            double ax = path[q], ay = path[q + 1], dx = path[q + 2] - ax, dy = path[q + 3] - ay;
            double l2 = dx * dx + dy * dy;
            double f = l2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / l2));
            best = Math.min(best, length(x - ax - f * dx, y - ay - f * dy));
        }
        return best;
    }
}
