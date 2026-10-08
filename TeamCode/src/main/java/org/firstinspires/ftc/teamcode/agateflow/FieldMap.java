package org.firstinspires.ftc.teamcode.agateflow;

import org.firstinspires.ftc.teamcode.field.Field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Everything AgateFlow keeps the robot away from: the four field walls, plus
 * zones grouped by where they came from ("field" for fixed things, "vision"
 * for what the camera sees, and so on). The robot may touch the walls (see
 * AgateFlow.WALL_GAP); every other zone gets the whole SAFETY_MARGIN.
 *
 * setZones() replaces all zones from one source at once, so vision can call it
 * every frame with what it sees now. It is safe to call from a camera thread
 * while the OpMode loop plans: each plan works on a snapshot() that never changes.
 *
 * The version goes up only when the zones really change, so calling setZones()
 * with the same zones every loop doesn't make AgateFlow rebuild anything.
 */
public final class FieldMap {
    /** The walls, as thick blocks just outside the field. */
    private static final double WALL_THICKNESS = 24;

    private final Map<String, List<Zone>> bySource = new LinkedHashMap<>();
    private volatile Snapshot snapshot;
    private long version = 0;

    public FieldMap() {
        snapshot = new Snapshot(Collections.<Zone>emptyList(), version);
        rebuild();
    }

    /**
     * Replaces every zone from this source. An empty list (or null) clears them.
     * Null zones, and zones with a NaN or infinite corner (vision lost the target,
     * say), are left out, so one bad detection can't break every plan.
     */
    public synchronized void setZones(String source, List<Zone> zones) {
        List<Zone> old = bySource.get(source);
        List<Zone> copy = new ArrayList<>();
        if (zones != null) {
            for (Zone zone : zones) if (zone != null && zone.finite()) copy.add(zone);
        }
        if (old == null ? copy.isEmpty() : old.equals(copy)) return;
        if (copy.isEmpty()) bySource.remove(source);
        else bySource.put(source, copy);
        rebuild();
    }

    public synchronized void clearZones(String source) {
        setZones(source, Collections.<Zone>emptyList());
    }

    /** All zones right now, walls included. It never changes, so one plan can use it throughout. */
    public Snapshot snapshot() {
        return snapshot;
    }

    private void rebuild() {
        List<Zone> all = new ArrayList<>(walls());
        for (List<Zone> zones : bySource.values()) all.addAll(zones);
        version++;
        snapshot = new Snapshot(Collections.unmodifiableList(all), version);
    }

    private static List<Zone> walls() {
        double s = Field.SIZE, t = WALL_THICKNESS;
        List<Zone> walls = new ArrayList<>();
        walls.add(Zone.wall("wall red", -t, -t, 0, s + t));
        walls.add(Zone.wall("wall blue", s, -t, s + t, s + t));
        walls.add(Zone.wall("wall audience", -t, -t, s + t, 0));
        walls.add(Zone.wall("wall far", -t, s, s + t, s + t));
        return walls;
    }

    /** The zones at one moment. Never changes after it is made. */
    public static final class Snapshot {
        public final List<Zone> zones;
        /** Goes up every time the zones change. */
        public final long version;

        Snapshot(List<Zone> zones, long version) {
            this.zones = zones;
            this.version = version;
        }

        /** Distance from the point to the nearest zone or wall, in inches. Negative inside one. */
        public double clearance(double x, double y) {
            double nearest = Double.POSITIVE_INFINITY;
            for (Zone zone : zones) nearest = Math.min(nearest, zone.distance(x, y));
            return nearest;
        }

        /** The nearest zone to the point, or null if there are none. */
        public Zone nearest(double x, double y) {
            Zone best = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            for (Zone zone : zones) {
                double d = zone.distance(x, y);
                if (d < bestDistance) {
                    bestDistance = d;
                    best = zone;
                }
            }
            return best;
        }
    }
}
