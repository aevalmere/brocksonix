package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;

import java.util.Arrays;

/**
 * A keep-out area: the real outline of something on the field (a HIVE, another
 * robot, a game element vision found). AgateFlow adds the robot's radius and
 * a safety margin itself, so don't add the robot's size here.
 *
 * Zones are always convex. polygon() takes the convex hull of its points, so a
 * concave shape grows (safe) instead of shrinking. For an L shape, add two zones.
 * Points are stored counter-clockwise.
 */
public final class Zone {
    public final String name;
    final double[] xs;
    final double[] ys;
    final double minX, minY, maxX, maxY;
    /** One of the four field walls (FieldMap makes them). The robot may touch these (see AgateFlow.WALL_GAP). */
    final boolean wall;

    private Zone(String name, double[] xs, double[] ys) {
        this(name, xs, ys, false);
    }

    private Zone(String name, double[] xs, double[] ys, boolean wall) {
        this.wall = wall;
        if (xs.length < 3) throw new IllegalArgumentException("Zone " + name + " needs at least 3 corners that aren't in a line");
        this.name = name;
        this.xs = xs;
        this.ys = ys;
        double loX = Double.POSITIVE_INFINITY, loY = Double.POSITIVE_INFINITY;
        double hiX = Double.NEGATIVE_INFINITY, hiY = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) {
            loX = Math.min(loX, xs[i]);
            hiX = Math.max(hiX, xs[i]);
            loY = Math.min(loY, ys[i]);
            hiY = Math.max(hiY, ys[i]);
        }
        minX = loX;
        minY = loY;
        maxX = hiX;
        maxY = hiY;
    }

    /** A field wall: a rectangle the robot may touch (see AgateFlow.WALL_GAP). Only FieldMap makes these. */
    static Zone wall(String name, double x1, double y1, double x2, double y2) {
        Zone box = rectangle(name, x1, y1, x2, y2);
        return new Zone(box.name, box.xs, box.ys, true);
    }

    /** Axis-aligned rectangle between two opposite corners, in inches. */
    public static Zone rectangle(String name, double x1, double y1, double x2, double y2) {
        return polygon(name, x1, y1, x2, y1, x2, y2, x1, y2);
    }

    /** A rectangle centered on a pose and turned with its heading, like another robot. Length is along the heading. */
    public static Zone box(String name, Pose center, double length, double width) {
        double c = Math.cos(center.heading()), s = Math.sin(center.heading());
        double hl = length / 2, hw = width / 2;
        double[] xy = new double[8];
        double[][] corners = {{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}};
        for (int i = 0; i < 4; i++) {
            xy[2 * i] = center.x() + corners[i][0] * c - corners[i][1] * s;
            xy[2 * i + 1] = center.y() + corners[i][0] * s + corners[i][1] * c;
        }
        return polygon(name, xy);
    }

    /**
     * A straight bar from (x1, y1) to (x2, y2), {@code thickness} inches across, like
     * a rail or a long thin part of the field. Its ends are square.
     */
    public static Zone line(String name, double x1, double y1, double x2, double y2, double thickness) {
        double dx = x2 - x1, dy = y2 - y1, length = Geometry.length(dx, dy);
        if (length < 1e-9) throw new IllegalArgumentException("Zone " + name + ": the two ends are the same point");
        double nx = -dy / length * thickness / 2, ny = dx / length * thickness / 2;
        return polygon(name, x1 + nx, y1 + ny, x2 + nx, y2 + ny, x2 - nx, y2 - ny, x1 - nx, y1 - ny);
    }

    /** A circle, stored as a 16-sided polygon drawn just outside it, so the circle is fully covered. */
    public static Zone circle(String name, double centerX, double centerY, double radius) {
        int sides = 16;
        double outer = radius / Math.cos(Math.PI / sides);
        double[] xy = new double[2 * sides];
        for (int i = 0; i < sides; i++) {
            double angle = 2 * Math.PI * i / sides;
            xy[2 * i] = centerX + outer * Math.cos(angle);
            xy[2 * i + 1] = centerY + outer * Math.sin(angle);
        }
        return polygon(name, xy);
    }

    /** Any convex shape from x, y pairs: polygon("name", x0, y0, x1, y1, ...). The convex hull is used. */
    public static Zone polygon(String name, double... xy) {
        if (xy.length % 2 != 0) throw new IllegalArgumentException("Zone " + name + ": points must be x, y pairs");
        int n = xy.length / 2;
        Integer[] order = new Integer[n];
        for (int i = 0; i < n; i++) order[i] = i;
        // Monotone chain convex hull: sort by x then y, build the lower and upper halves.
        Arrays.sort(order, (a, b) -> xy[2 * a] != xy[2 * b]
                ? Double.compare(xy[2 * a], xy[2 * b])
                : Double.compare(xy[2 * a + 1], xy[2 * b + 1]));
        int[] hull = new int[2 * n];
        int k = 0;
        for (int j = 0; j < n; j++) {
            while (k >= 2 && cross(xy, hull[k - 2], hull[k - 1], order[j]) <= 1e-12) k--;
            hull[k++] = order[j];
        }
        int lowerSize = k + 1;
        for (int j = n - 2; j >= 0; j--) {
            while (k >= lowerSize && cross(xy, hull[k - 2], hull[k - 1], order[j]) <= 1e-12) k--;
            hull[k++] = order[j];
        }
        k--; // the last point repeats the first
        double[] xs = new double[k], ys = new double[k];
        for (int i = 0; i < k; i++) {
            xs[i] = xy[2 * hull[i]];
            ys[i] = xy[2 * hull[i] + 1];
        }
        return new Zone(name, xs, ys);
    }

    private static double cross(double[] xy, int o, int a, int b) {
        return (xy[2 * a] - xy[2 * o]) * (xy[2 * b + 1] - xy[2 * o + 1])
                - (xy[2 * a + 1] - xy[2 * o + 1]) * (xy[2 * b] - xy[2 * o]);
    }

    /** True if every corner is a real number (no NaN or infinity). FieldMap leaves out zones that aren't. */
    boolean finite() {
        for (int i = 0; i < xs.length; i++) {
            if (Double.isNaN(xs[i]) || Double.isInfinite(xs[i]) || Double.isNaN(ys[i]) || Double.isInfinite(ys[i])) return false;
        }
        return true;
    }

    /** For a zone written for red: the same zone on this alliance's side (blue is red spun 180° about the center). */
    public Zone forAlliance(Alliance alliance) {
        if (alliance != Alliance.BLUE) return this;
        double[] xy = new double[2 * xs.length];
        for (int i = 0; i < xs.length; i++) {
            xy[2 * i] = Field.SIZE - xs[i];
            xy[2 * i + 1] = Field.SIZE - ys[i];
        }
        return polygon(name, xy);
    }

    public int corners() {
        return xs.length;
    }

    public double cornerX(int i) {
        return xs[i];
    }

    public double cornerY(int i) {
        return ys[i];
    }

    public boolean contains(double x, double y) {
        return distance(x, y) < 0;
    }

    /** Distance from the point to the zone's edge in inches. Negative inside. */
    public double distance(double x, double y) {
        boolean inside = true;
        double nearest = Double.POSITIVE_INFINITY;
        int n = xs.length;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double ex = xs[j] - xs[i], ey = ys[j] - ys[i];
            double px = x - xs[i], py = y - ys[i];
            // Counter-clockwise, so a point right of any edge is outside.
            if (ex * py - ey * px < 0) inside = false;
            double t = (px * ex + py * ey) / (ex * ex + ey * ey);
            t = Math.max(0, Math.min(1, t));
            double dx = px - t * ex, dy = py - t * ey;
            nearest = Math.min(nearest, dx * dx + dy * dy);
        }
        nearest = Math.sqrt(nearest);
        return inside ? -nearest : nearest;
    }

    /**
     * Same as distance(), and also the way from (x, y) to the nearest point of the
     * zone's edge, as a unit vector in {@code toward} (x, y). Used to see which side
     * of the robot faces the zone.
     */
    double distanceToward(double x, double y, double[] toward) {
        boolean inside = true;
        double nearest = Double.POSITIVE_INFINITY, nx = 0, ny = 0;
        int n = xs.length;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double ex = xs[j] - xs[i], ey = ys[j] - ys[i];
            double px = x - xs[i], py = y - ys[i];
            if (ex * py - ey * px < 0) inside = false;
            double t = (px * ex + py * ey) / (ex * ex + ey * ey);
            t = Math.max(0, Math.min(1, t));
            double dx = px - t * ex, dy = py - t * ey;
            double d2 = dx * dx + dy * dy;
            if (d2 < nearest) {
                nearest = d2;
                nx = -dx;
                ny = -dy;
            }
        }
        nearest = Math.sqrt(nearest);
        toward[0] = nearest > 1e-9 ? nx / nearest : 0;
        toward[1] = nearest > 1e-9 ? ny / nearest : 0;
        return inside ? -nearest : nearest;
    }

    /**
     * Exact room between this zone and a rectangle centered at (x, y) facing
     * {@code heading}, reaching {@code front} ahead, {@code back} behind and
     * {@code halfWidth} to each side. Negative: how deep they overlap. Slower
     * than distance() (it compares every side of both shapes).
     */
    double outlineDistance(double x, double y, double heading, double front, double back, double halfWidth) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double[] rx = new double[4], ry = new double[4];
        double[] along = {front, -back, -back, front}, across = {halfWidth, halfWidth, -halfWidth, -halfWidth};
        for (int i = 0; i < 4; i++) {
            rx[i] = x + along[i] * c - across[i] * s;
            ry[i] = y + along[i] * s + across[i] * c;
        }
        // Separating axis test: the largest gap along any side's outward normal, from either shape.
        double gap = Math.max(separation(rx, ry, xs, ys), separation(xs, ys, rx, ry));
        if (gap <= 0) return gap;
        double best = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4; i++) best = Math.min(best, distance(rx[i], ry[i]));
        for (int i = 0; i < xs.length; i++) {
            for (int k = 0; k < 4; k++) {
                int m = (k + 1) % 4;
                double ex = rx[m] - rx[k], ey = ry[m] - ry[k];
                double t = Math.max(0, Math.min(1, ((xs[i] - rx[k]) * ex + (ys[i] - ry[k]) * ey) / (ex * ex + ey * ey)));
                best = Math.min(best, Geometry.length(xs[i] - rx[k] - t * ex, ys[i] - ry[k] - t * ey));
            }
        }
        return best;
    }

    /** Largest gap between two convex shapes along the normals of a's sides (negative when they overlap). */
    private static double separation(double[] ax, double[] ay, double[] bx, double[] by) {
        double best = Double.NEGATIVE_INFINITY;
        int n = ax.length;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double ex = ax[j] - ax[i], ey = ay[j] - ay[i], l = Geometry.length(ex, ey);
            if (l < 1e-12) continue;
            double nx = ey / l, ny = -ex / l;
            double aMin = Double.POSITIVE_INFINITY, aMax = Double.NEGATIVE_INFINITY, bMin = aMin, bMax = aMax;
            for (int k = 0; k < ax.length; k++) {
                double v = ax[k] * nx + ay[k] * ny;
                aMin = Math.min(aMin, v);
                aMax = Math.max(aMax, v);
            }
            for (int k = 0; k < bx.length; k++) {
                double v = bx[k] * nx + by[k] * ny;
                bMin = Math.min(bMin, v);
                bMax = Math.max(bMax, v);
            }
            best = Math.max(best, Math.max(bMin - aMax, aMin - bMax));
        }
        return best;
    }

    /**
     * Same as distance(), but stops early when the bounding box alone shows the
     * point is at least {@code enough} away. Then it returns a value of at least
     * {@code enough} that may not be exact. Used by the many clearance checks.
     * With {@code enough} at 0 or below there is no shortcut (the point may be
     * inside, where only distance() gives the right negative number).
     */
    double distanceAtLeast(double x, double y, double enough) {
        double dx = Math.max(Math.max(minX - x, x - maxX), 0);
        double dy = Math.max(Math.max(minY - y, y - maxY), 0);
        if (enough > 0 && (dx >= enough || dy >= enough || dx * dx + dy * dy >= enough * enough)) {
            return Geometry.length(dx, dy);
        }
        return distance(x, y);
    }

    /**
     * The zone grown by {@code radius}, as a convex polygon that fully covers the
     * true grown shape (straight sides pushed out, corners rounded). Each rounded
     * corner is split into pieces of at most {@code maxArcStep} radians, and the
     * polygon's sides touch the arc from outside. Returns {xs, ys}.
     */
    double[][] grown(double radius, double maxArcStep) {
        int n = xs.length;
        if (radius <= 0) return new double[][]{xs.clone(), ys.clone()};
        double[] outX = new double[n * 8], outY = new double[n * 8];
        int count = 0;
        for (int i = 0; i < n; i++) {
            int prev = (i + n - 1) % n, next = (i + 1) % n;
            // Outward normal of a counter-clockwise edge (dx, dy) is (dy, -dx).
            double inAngle = Math.atan2(-(xs[i] - xs[prev]), ys[i] - ys[prev]);
            double outAngle = Math.atan2(-(xs[next] - xs[i]), ys[next] - ys[i]);
            double turn = outAngle - inAngle;
            while (turn < 0) turn += 2 * Math.PI;
            while (turn >= 2 * Math.PI) turn -= 2 * Math.PI;
            int steps = Math.max(1, (int) Math.ceil(turn / maxArcStep - 1e-9));
            double step = turn / steps;
            double reach = radius / Math.cos(step / 2);
            for (int j = 0; j < steps; j++) {
                if (count == outX.length) {
                    outX = Arrays.copyOf(outX, count * 2);
                    outY = Arrays.copyOf(outY, count * 2);
                }
                double angle = inAngle + (j + 0.5) * step;
                outX[count] = xs[i] + reach * Math.cos(angle);
                outY[count] = ys[i] + reach * Math.sin(angle);
                count++;
            }
        }
        return new double[][]{Arrays.copyOf(outX, count), Arrays.copyOf(outY, count)};
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Zone)) return false;
        Zone zone = (Zone) other;
        return name.equals(zone.name) && wall == zone.wall && Arrays.equals(xs, zone.xs) && Arrays.equals(ys, zone.ys);
    }

    @Override
    public int hashCode() {
        return 31 * (31 * name.hashCode() + Arrays.hashCode(xs)) + Arrays.hashCode(ys);
    }

    @Override
    public String toString() {
        return "Zone " + name + " (" + xs.length + " corners)";
    }
}
