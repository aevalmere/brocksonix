package org.firstinspires.ftc.teamcode.agateflow;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Finds the shortest way around the zones, as a list of straight lines.
 *
 * Every zone is grown by the robot's radius (plus margin), a field wall by less
 * (the robot may touch the walls: ClearanceRule.circleNeed). The robot's center
 * then only has to stay outside the grown shapes, and the shortest way around
 * a set of convex shapes always runs along straight lines that touch their
 * corners. So the graph's points are the grown corners, and two points are
 * joined if the line between them crosses nothing and just grazes both shapes
 * (a "tangent" line; any other line can't be part of a shortest path).
 *
 * The graph only depends on the zones and the radius, so AgateFlow keeps it
 * and only adds the start and end points for each new search.
 *
 * The search is A* with a cost for turning: each corner costs
 * turnWeight × (1 - cos(turn angle)) inches extra, so with a turn weight the
 * search prefers fewer and gentler turns, which are faster to drive.
 */
final class VisibilityGraph {
    /** Rounded corners of a grown zone are split into steps of at most this. */
    private static final double ARC_STEP = Math.toRadians(30);

    final long version;
    final double radius;
    /** How much the walls were grown (less than the radius, see ClearanceRule), to know when the tunables change. */
    final double wallGrowth;
    /** Grown for a Route.allowContact stretch (walls like any zone). */
    final boolean contact;

    private final List<Zone> zones;
    private final double[][] polyX, polyY;
    private final double[] boxMinX, boxMinY, boxMaxX, boxMaxY;

    private final int nodeCount;
    private final double[] nodeX, nodeY;
    private final int[] nodePoly;
    /** The corners before and after each node on its shape, for the tangent test. */
    private final double[] prevX, prevY, nextX, nextY;

    /** Node i's neighbors are adjNode[adjStart[i] .. adjStart[i + 1] - 1]. */
    private final int[] adjStart, adjNode;
    private final double[] adjLength;

    private VisibilityGraph(FieldMap.Snapshot snapshot, double radius, boolean contact) {
        this.version = snapshot.version;
        this.radius = radius;
        this.contact = contact;
        this.zones = snapshot.zones;
        this.wallGrowth = wallGrowth(radius, contact);

        int polys = zones.size();
        polyX = new double[polys][];
        polyY = new double[polys][];
        boxMinX = new double[polys];
        boxMinY = new double[polys];
        boxMaxX = new double[polys];
        boxMaxY = new double[polys];
        for (int p = 0; p < polys; p++) {
            double[][] grown = zones.get(p).grown(ClearanceRule.circleNeed(zones.get(p), radius, contact), ARC_STEP);
            polyX[p] = grown[0];
            polyY[p] = grown[1];
            boxMinX[p] = min(grown[0]);
            boxMaxX[p] = max(grown[0]);
            boxMinY[p] = min(grown[1]);
            boxMaxY[p] = max(grown[1]);
        }

        // Nodes: grown corners that aren't inside another grown zone.
        List<double[]> nodes = new ArrayList<>();
        for (int p = 0; p < polys; p++) {
            int n = polyX[p].length;
            for (int i = 0; i < n; i++) {
                double x = polyX[p][i], y = polyY[p][i];
                boolean covered = false;
                for (int q = 0; q < polys && !covered; q++) {
                    if (q != p && strictlyInside(q, x, y)) covered = true;
                }
                if (covered) continue;
                int prev = (i + n - 1) % n, next = (i + 1) % n;
                nodes.add(new double[]{x, y, p, polyX[p][prev], polyY[p][prev], polyX[p][next], polyY[p][next], i});
            }
        }
        nodeCount = nodes.size();
        nodeX = new double[nodeCount];
        nodeY = new double[nodeCount];
        nodePoly = new int[nodeCount];
        prevX = new double[nodeCount];
        prevY = new double[nodeCount];
        nextX = new double[nodeCount];
        nextY = new double[nodeCount];
        int[] cornerIndex = new int[nodeCount];
        for (int i = 0; i < nodeCount; i++) {
            double[] node = nodes.get(i);
            nodeX[i] = node[0];
            nodeY[i] = node[1];
            nodePoly[i] = (int) node[2];
            prevX[i] = node[3];
            prevY[i] = node[4];
            nextX[i] = node[5];
            nextY[i] = node[6];
            cornerIndex[i] = (int) node[7];
        }

        // Edges: tangent at both ends and crossing no grown zone.
        List<List<Integer>> neighbors = new ArrayList<>();
        for (int i = 0; i < nodeCount; i++) neighbors.add(new ArrayList<Integer>());
        for (int a = 0; a < nodeCount; a++) {
            for (int b = a + 1; b < nodeCount; b++) {
                if (nodePoly[a] == nodePoly[b]) {
                    // On one convex shape, only neighboring corners see each other (along its side).
                    int n = polyX[nodePoly[a]].length;
                    int gap = Math.abs(cornerIndex[a] - cornerIndex[b]);
                    if (gap != 1 && gap != n - 1) continue;
                } else if (!tangentAt(a, nodeX[b], nodeY[b]) || !tangentAt(b, nodeX[a], nodeY[a])) {
                    continue;
                }
                if (!lineClear(nodeX[a], nodeY[a], nodeX[b], nodeY[b], null)) continue;
                neighbors.get(a).add(b);
                neighbors.get(b).add(a);
            }
        }
        adjStart = new int[nodeCount + 1];
        for (int i = 0; i < nodeCount; i++) adjStart[i + 1] = adjStart[i] + neighbors.get(i).size();
        adjNode = new int[adjStart[nodeCount]];
        adjLength = new double[adjStart[nodeCount]];
        for (int i = 0; i < nodeCount; i++) {
            int k = adjStart[i];
            for (int j : neighbors.get(i)) {
                adjNode[k] = j;
                adjLength[k] = Geometry.length(nodeX[j] - nodeX[i], nodeY[j] - nodeY[i]);
                k++;
            }
        }
    }

    static VisibilityGraph build(FieldMap.Snapshot snapshot, double radius, boolean contact) {
        return new VisibilityGraph(snapshot, radius, contact);
    }

    /** How much a wall is grown with this radius right now. */
    static double wallGrowth(double radius, boolean contact) {
        return ClearanceRule.circleNeed(Zone.wall("probe", 0, 0, 1, 1), radius, contact);
    }

    /** True if (x, y) is inside zone p grown by the radius (on the edge doesn't count). */
    boolean strictlyInside(int p, double x, double y) {
        if (x <= boxMinX[p] || x >= boxMaxX[p] || y <= boxMinY[p] || y >= boxMaxY[p]) return false;
        double[] px = polyX[p], py = polyY[p];
        int n = px.length;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double ex = px[j] - px[i], ey = py[j] - py[i];
            // Outward normal (ey, -ex). Inside means behind every side.
            if (ey * (x - px[i]) - ex * (y - py[i]) >= -1e-6 * Geometry.length(ex, ey)) return false;
        }
        return true;
    }

    /** A line from node a toward (x, y) grazes a's shape instead of cutting into it. */
    private boolean tangentAt(int a, double x, double y) {
        double dx = x - nodeX[a], dy = y - nodeY[a];
        double sidePrev = dx * (prevY[a] - nodeY[a]) - dy * (prevX[a] - nodeX[a]);
        double sideNext = dx * (nextY[a] - nodeY[a]) - dy * (nextX[a] - nodeX[a]);
        return sidePrev * sideNext >= -1e-9;
    }

    /**
     * True if the line from a to b doesn't go into any grown zone. Zones marked in
     * {@code skip} are not checked here (the caller checks them another way).
     */
    boolean lineClear(double ax, double ay, double bx, double by, boolean[] skip) {
        double loX = Math.min(ax, bx), hiX = Math.max(ax, bx), loY = Math.min(ay, by), hiY = Math.max(ay, by);
        for (int p = 0; p < polyX.length; p++) {
            if (skip != null && skip[p]) continue;
            if (hiX <= boxMinX[p] || loX >= boxMaxX[p] || hiY <= boxMinY[p] || loY >= boxMaxY[p]) continue;
            if (crossesInside(polyX[p], polyY[p], ax, ay, bx, by)) return false;
        }
        return true;
    }

    /** Clips the line against each side of a convex shape. Something left over means it goes inside. */
    private static boolean crossesInside(double[] px, double[] py, double ax, double ay, double bx, double by) {
        double low = 0, high = 1;
        double dx = bx - ax, dy = by - ay;
        int n = px.length;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double ex = px[j] - px[i], ey = py[j] - py[i];
            double eps = 1e-6 * Geometry.length(ex, ey);
            double start = ey * (ax - px[i]) - ex * (ay - py[i]);
            double rate = ey * dx - ex * dy;
            // Inside this side means start + t * rate < -eps.
            if (Math.abs(rate) < 1e-12) {
                if (start >= -eps) return false;
                continue;
            }
            double t = (-eps - start) / rate;
            if (rate > 0) high = Math.min(high, t);
            else low = Math.max(low, t);
            if (low >= high - 1e-9) return false;
        }
        return true;
    }

    /** One end of a search: the start or the end point, or a point just off it. */
    static final class Terminal {
        final double x, y;
        /** Direction the robot should leave a start in, or arrive at an end in. (0, 0) for any direction. */
        final double dirX, dirY;
        /** Extra cost of using this terminal (inches). */
        final double cost;
        /** Points between the real start or end and this terminal, as x, y pairs, in driving order. May be empty. */
        final double[] via;

        Terminal(double x, double y, double dirX, double dirY, double cost, double[] via) {
            this.x = x;
            this.y = y;
            double length = Geometry.length(dirX, dirY);
            this.dirX = length > 1e-9 ? dirX / length : 0;
            this.dirY = length > 1e-9 ? dirY / length : 0;
            this.cost = cost;
            this.via = via;
        }

        boolean hasDirection() {
            return dirX != 0 || dirY != 0;
        }
    }

    /**
     * Shortest (turn-weighted) path from any start terminal to any end terminal.
     * Returns the points in driving order as x, y pairs, from the real start
     * to the real end, or null if there is no way through.
     *
     * @param rule clearance rule at this graph's radius; used for lines leaving a
     *             terminal that sits inside a grown zone (near a wall, say)
     */
    double[] search(Terminal[] starts, Terminal[] ends, ClearanceRule rule, double turnWeight) {
        int s0 = nodeCount, e0 = nodeCount + starts.length, total = e0 + ends.length;
        double[] vx = new double[total], vy = new double[total];
        for (int i = 0; i < nodeCount; i++) {
            vx[i] = nodeX[i];
            vy[i] = nodeY[i];
        }
        for (int i = 0; i < starts.length; i++) {
            vx[s0 + i] = starts[i].x;
            vy[s0 + i] = starts[i].y;
        }
        for (int i = 0; i < ends.length; i++) {
            vx[e0 + i] = ends[i].x;
            vy[e0 + i] = ends[i].y;
        }

        // Which nodes each terminal can see, and which ends each start can drive straight to.
        boolean[][] startSkip = new boolean[starts.length][], endSkip = new boolean[ends.length][];
        for (int i = 0; i < starts.length; i++) startSkip[i] = zonesAround(starts[i]);
        for (int i = 0; i < ends.length; i++) endSkip[i] = zonesAround(ends[i]);
        boolean[][] startSees = new boolean[starts.length][total];
        for (int i = 0; i < starts.length; i++) {
            visibleNodes(starts[i], startSkip[i], rule, startSees[i]);
            for (int e = 0; e < ends.length; e++) {
                startSees[i][e0 + e] = directClear(starts[i], startSkip[i], ends[e], endSkip[e], rule);
            }
        }
        boolean[][] endSees = new boolean[ends.length][nodeCount];
        for (int i = 0; i < ends.length; i++) visibleNodes(ends[i], endSkip[i], rule, endSees[i]);

        // Edges for this search, in the same layout as the graph's: graph edges, plus node -> end, plus start -> anything.
        int[] start = new int[total + 1];
        int[] count = new int[total];
        for (int i = 0; i < nodeCount; i++) {
            count[i] = adjStart[i + 1] - adjStart[i];
            for (boolean[] sees : endSees) if (sees[i]) count[i]++;
        }
        for (int i = 0; i < starts.length; i++) {
            for (int j = 0; j < total; j++) if (startSees[i][j]) count[s0 + i]++;
        }
        for (int i = 0; i < total; i++) start[i + 1] = start[i] + count[i];
        int[] to = new int[start[total]];
        double[] length = new double[start[total]];
        int[] fill = Arrays.copyOf(start, total);
        for (int i = 0; i < nodeCount; i++) {
            for (int k = adjStart[i]; k < adjStart[i + 1]; k++) {
                to[fill[i]] = adjNode[k];
                length[fill[i]++] = adjLength[k];
            }
            for (int e = 0; e < ends.length; e++) {
                if (!endSees[e][i]) continue;
                to[fill[i]] = e0 + e;
                length[fill[i]++] = Geometry.length(vx[e0 + e] - vx[i], vy[e0 + e] - vy[i]);
            }
        }
        for (int s = 0; s < starts.length; s++) {
            int v = s0 + s;
            for (int j = 0; j < total; j++) {
                if (!startSees[s][j]) continue;
                to[fill[v]] = j;
                length[fill[v]++] = Geometry.length(vx[j] - vx[v], vy[j] - vy[v]);
            }
        }

        // A* where each state is an edge (where we came from matters for the turn cost).
        int edges = to.length;
        double[] cost = new double[edges];
        Arrays.fill(cost, Double.POSITIVE_INFINITY);
        int[] cameFrom = new int[edges];
        int[] edgeFrom = new int[edges];
        for (int v = 0; v < total; v++) {
            for (int k = start[v]; k < start[v + 1]; k++) edgeFrom[k] = v;
        }
        PriorityQueue<double[]> open = new PriorityQueue<>(64, (a, b) -> Double.compare(a[0], b[0]));
        for (int s = 0; s < starts.length; s++) {
            Terminal t = starts[s];
            for (int k = start[s0 + s]; k < start[s0 + s + 1]; k++) {
                if (length[k] < 1e-6) continue;
                double c = t.cost + length[k];
                if (t.hasDirection()) {
                    c += turnWeight * turnCost(t.dirX, t.dirY, vx[to[k]] - t.x, vy[to[k]] - t.y);
                }
                c += arrivalCost(to[k], k, e0, ends, vx, vy, edgeFrom, turnWeight);
                if (c < cost[k]) {
                    cost[k] = c;
                    cameFrom[k] = -1;
                    open.add(new double[]{c + guess(to[k], ends, vx, vy), k});
                }
            }
        }
        int found = -1;
        while (!open.isEmpty()) {
            double[] item = open.poll();
            int k = (int) item[1];
            if (item[0] - guess(to[k], ends, vx, vy) > cost[k] + 1e-9) continue; // stale
            int v = to[k];
            if (v >= e0) {
                found = k;
                break;
            }
            int u = edgeFrom[k];
            for (int k2 = start[v]; k2 < start[v + 1]; k2++) {
                int w = to[k2];
                if (w == u || length[k2] < 1e-6) continue;
                double c = cost[k] + length[k2]
                        + turnWeight * turnCost(vx[v] - vx[u], vy[v] - vy[u], vx[w] - vx[v], vy[w] - vy[v])
                        + arrivalCost(w, k2, e0, ends, vx, vy, edgeFrom, turnWeight);
                if (c < cost[k2] - 1e-9) {
                    cost[k2] = c;
                    cameFrom[k2] = k;
                    open.add(new double[]{c + guess(w, ends, vx, vy), k2});
                }
            }
        }
        if (found < 0) return null;

        // Walk back, then lay out: start's via points, the vertices, end's via points.
        List<Integer> vertices = new ArrayList<>();
        for (int k = found; k >= 0; k = cameFrom[k]) {
            vertices.add(0, to[k]);
            if (cameFrom[k] < 0) vertices.add(0, edgeFrom[k]);
        }
        Terminal first = starts[vertices.get(0) - s0];
        Terminal last = ends[vertices.get(vertices.size() - 1) - e0];
        double[] out = new double[first.via.length + 2 * vertices.size() + last.via.length];
        int n = 0;
        for (double value : first.via) out[n++] = value;
        for (int v : vertices) {
            out[n++] = vx[v];
            out[n++] = vy[v];
        }
        for (double value : last.via) out[n++] = value;
        return out;
    }

    /** Extra cost when an edge arrives at an end terminal: the end's own cost and its arrival direction. */
    private static double arrivalCost(int w, int k, int e0, Terminal[] ends, double[] vx, double[] vy,
                                      int[] edgeFrom, double turnWeight) {
        if (w < e0) return 0;
        Terminal end = ends[w - e0];
        double c = end.cost;
        if (end.hasDirection()) {
            int u = edgeFrom[k];
            c += turnWeight * turnCost(vx[w] - vx[u], vy[w] - vy[u], end.dirX, end.dirY);
        }
        return c;
    }

    /** Straight-line distance to the nearest end: never more than the real remaining cost. */
    private static double guess(int v, Terminal[] ends, double[] vx, double[] vy) {
        double best = Double.POSITIVE_INFINITY;
        for (Terminal end : ends) best = Math.min(best, Geometry.length(end.x - vx[v], end.y - vy[v]));
        return best;
    }

    /** 1 - cos(angle between the two directions): 0 straight on, 1 for a right angle, 2 for turning back. */
    static double turnCost(double ax, double ay, double bx, double by) {
        double la = Geometry.length(ax, ay), lb = Geometry.length(bx, by);
        if (la < 1e-9 || lb < 1e-9) return 0;
        return 1 - (ax * bx + ay * by) / (la * lb);
    }

    /** Grown zones the terminal is inside (near a wall, say). Lines from it get the relaxed rule for those. */
    private boolean[] zonesAround(Terminal t) {
        boolean[] inside = new boolean[polyX.length];
        for (int p = 0; p < polyX.length; p++) inside[p] = strictlyInside(p, t.x, t.y);
        return inside;
    }

    /** Marks in {@code sees} the graph nodes the terminal can drive straight to. */
    private void visibleNodes(Terminal t, boolean[] skip, ClearanceRule rule, boolean[] sees) {
        for (int j = 0; j < nodeCount; j++) {
            double dx = nodeX[j] - t.x, dy = nodeY[j] - t.y;
            if (dx * dx + dy * dy < 1e-12) continue;
            if (!tangentAt(j, t.x, t.y)) continue;
            if (!lineClear(t.x, t.y, nodeX[j], nodeY[j], skip)) continue;
            if (!relaxedClear(skip, rule, t.x, t.y, nodeX[j], nodeY[j])) continue;
            sees[j] = true;
        }
    }

    /** Straight from a start terminal to an end terminal, with the relaxed rule for zones around either one. */
    private boolean directClear(Terminal a, boolean[] skipA, Terminal b, boolean[] skipB, ClearanceRule rule) {
        if (Geometry.length(b.x - a.x, b.y - a.y) < 1e-6) return false;
        boolean[] skip = new boolean[skipA.length];
        for (int p = 0; p < skip.length; p++) skip[p] = skipA[p] || skipB[p];
        return lineClear(a.x, a.y, b.x, b.y, skip) && relaxedClear(skip, rule, a.x, a.y, b.x, b.y);
    }

    /** Checks the skipped zones with the relaxed rule, every inch along the line. */
    private static boolean relaxedClear(boolean[] skip, ClearanceRule rule, double ax, double ay, double bx, double by) {
        double length = Geometry.length(bx - ax, by - ay);
        int samples = Math.max(1, (int) Math.ceil(length));
        for (int p = 0; p < skip.length; p++) {
            if (!skip[p]) continue;
            for (int i = 1; i <= samples; i++) {
                double t = (double) i / samples;
                if (rule.slack(p, ax + t * (bx - ax), ay + t * (by - ay)) < -1e-6) return false;
            }
        }
        return true;
    }

    private static double min(double[] values) {
        double m = Double.POSITIVE_INFINITY;
        for (double v : values) m = Math.min(m, v);
        return m;
    }

    private static double max(double[] values) {
        double m = Double.NEGATIVE_INFINITY;
        for (double v : values) m = Math.max(m, v);
        return m;
    }
}
