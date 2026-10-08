package org.firstinspires.ftc.teamcode.agateflow;

import java.util.List;

/**
 * The tests' own judge of a collision, written separately from ClearanceRule: the
 * robot's real outline (AgateFlow.ROBOT_FRONT, ROBOT_BACK, ROBOT_HALF_WIDTH) must
 * not overlap any zone, with no safety margin. Near a route point the outline
 * already overlaps (a waypoint against a wall, say), it may overlap that much
 * plus 0.5 in., less ESCAPE_SLOPE per inch away from the point (the robot has to
 * leave). For a contact route (radius under ROBOT_RADIUS) it is the center's
 * distance to the zones against that radius, relaxed the same way.
 */
final class TrueRule {
    private TrueRule() {}

    /**
     * Spare room at (x, y) facing h, inches; negative is a collision.
     *
     * @param points       route points (start first) as x, y pairs
     * @param startHeading the robot's heading at the start (for the relaxing there)
     */
    static double room(List<Zone> zones, double radius, double[] points, double startHeading, double x, double y, double h) {
        boolean outline = AgateFlow.USE_OUTLINE && radius >= AgateFlow.ROBOT_RADIUS;
        double least = Double.POSITIVE_INFINITY;
        for (Zone zone : zones) {
            double here = outline ? outlineDistance(zone, x, y, h) : zone.distance(x, y) - radius;
            if (here > 30) {
                least = Math.min(least, here);
                continue;
            }
            double need = 0;
            for (int p = 0; p + 1 < points.length; p += 2) {
                double at = outline ? outlineDistance(zone, points[p], points[p + 1], p == 0 ? startHeading : h)
                        : zone.distance(points[p], points[p + 1]) - radius;
                if (at < 0) need = Math.min(need, at - 0.5 + AgateFlow.ESCAPE_SLOPE * Math.hypot(x - points[p], y - points[p + 1]));
            }
            least = Math.min(least, here - need);
        }
        return least;
    }

    /** Exact distance between the robot's outline at (x, y, h) and a zone; negative is how deep they overlap. */
    static double outlineDistance(Zone zone, double x, double y, double h) {
        double c = Math.cos(h), s = Math.sin(h);
        double f = AgateFlow.ROBOT_FRONT, b = AgateFlow.ROBOT_BACK, w = AgateFlow.ROBOT_HALF_WIDTH;
        double[][] corners = {{f, w}, {-b, w}, {-b, -w}, {f, -w}};
        double[] rx = new double[4], ry = new double[4];
        for (int i = 0; i < 4; i++) {
            rx[i] = x + corners[i][0] * c - corners[i][1] * s;
            ry[i] = y + corners[i][0] * s + corners[i][1] * c;
        }
        double gap = Math.max(separation(rx, ry, zone.xs, zone.ys), separation(zone.xs, zone.ys, rx, ry));
        if (gap <= 0) return gap;
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4; i++) best = Math.min(best, zone.distance(rx[i], ry[i]));
        for (int i = 0; i < zone.xs.length; i++) best = Math.min(best, pointToPolygon(zone.xs[i], zone.ys[i], rx, ry));
        return best;
    }

    /** Largest gap between two convex shapes along the normals of a's sides (negative: overlap depth). */
    private static double separation(double[] ax, double[] ay, double[] bx, double[] by) {
        double best = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < ax.length; i++) {
            int j = (i + 1) % ax.length;
            double ex = ax[j] - ax[i], ey = ay[j] - ay[i], l = Math.hypot(ex, ey);
            if (l < 1e-12) continue;
            double nx = ey / l, ny = -ex / l;
            double aMin = Double.POSITIVE_INFINITY, aMax = Double.NEGATIVE_INFINITY, bMin = aMin, bMax = aMax;
            for (int k = 0; k < ax.length; k++) {
                double p = ax[k] * nx + ay[k] * ny;
                aMin = Math.min(aMin, p);
                aMax = Math.max(aMax, p);
            }
            for (int k = 0; k < bx.length; k++) {
                double p = bx[k] * nx + by[k] * ny;
                bMin = Math.min(bMin, p);
                bMax = Math.max(bMax, p);
            }
            best = Math.max(best, Math.max(bMin - aMax, aMin - bMax));
        }
        return best;
    }

    private static double pointToPolygon(double px, double py, double[] xs, double[] ys) {
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) {
            int j = (i + 1) % xs.length;
            double ex = xs[j] - xs[i], ey = ys[j] - ys[i];
            double t = Math.max(0, Math.min(1, ((px - xs[i]) * ex + (py - ys[i]) * ey) / (ex * ex + ey * ey)));
            best = Math.min(best, Math.hypot(px - xs[i] - t * ex, py - ys[i] - t * ey));
        }
        return best;
    }
}
