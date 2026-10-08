package org.firstinspires.ftc.teamcode.agateflow;

import org.firstinspires.ftc.teamcode.field.Field;

import java.util.List;

/**
 * How close the robot may get to each zone along one stretch of path.
 *
 * Normally its center stays {@code radius} (ROBOT_RADIUS plus SAFETY_MARGIN)
 * from a zone. Near the start and the end it is relaxed, because the robot may
 * start against a wall, or be sent to a spot right next to something (a FLOWER,
 * a corner). There the allowed distance starts at whatever the robot has at that
 * end and grows by {@code slope} inches per inch of travel, up to the normal one.
 * So the robot may leave a wall at an angle, but can't run along it.
 *
 * The robot may drift END_SLACK past an end while it settles, but never to
 * under ROBOT_RADIUS from a zone it was at least ROBOT_RADIUS from there: an
 * end in the safety margin only lets the robot into the margin, not the robot.
 *
 * slack(x, y) treats the robot as a circle. slack(x, y, heading) uses its real
 * outline, a rectangle (ROBOT_FRONT, ROBOT_BACK, ROBOT_HALF_WIDTH around the
 * point the localizer tracks; front from AgateFlow's flower intake setting): the
 * room between the outline and each zone must be at least the safety margin
 * (radius minus ROBOT_RADIUS). Then a robot passing a zone side-on may be closer
 * than one facing it. Near the start and end the same relaxing applies, measured
 * from the outline: at the start with the heading the robot really starts with
 * (so turning in place into a wall counts), elsewhere with its heading now. The
 * outline may start inside a zone (the robot is where it is, and must be able to
 * leave), but near the end it may only come into the margin, never into the zone
 * itself, so it can't end in one or swing into one on the way in. Unless the end
 * is meant to reach in (endMayOverlap: the flower intake into a FLOWER's opening).
 *
 * The four field walls are different: any part of the outline (the intake, the
 * sides, the back) may touch them. The outline keeps WALL_GAP from a wall
 * (default 0) instead of the safety margin; the "robot itself" rule (radius
 * ROBOT_RADIUS) keeps 0. For the circle checks (the searches, rounding corners) a
 * wall counts as ROBOT_RADIUS + SAFETY_MARGIN - (ROBOT_HALF_WIDTH + WALL_GAP) closer
 * than other zones, so a path may run along a wall with a side touching it; the
 * outline check on the predicted path then decides if the heading there fits
 * (front-on to a wall needs ROBOT_FRONT, back-on ROBOT_BACK). MotionModel also
 * makes sure the robot never hits a wall faster than WALL_CONTACT_SPEED.
 *
 * A contact rule (Route.allowContact, radius CONTACT_RADIUS) is the circle, the
 * same for walls and zones. With USE_OUTLINE off, everything is the circle.
 */
final class ClearanceRule {
    /** At the start and end, the robot may get this much closer than it is there: it drifts a little past a target while settling. */
    static final double END_SLACK = 0.5;

    final List<Zone> zones;
    final double radius;
    final double slope;
    private final double startX, startY, endX, endY;
    /** Per zone: distance from the start (or end) the center must keep (circle rule), or +infinity if that end isn't close enough to matter. */
    private final double[] startDistance, endDistance;
    /** Per zone: how far the center stays from it with the circle (walls less, see the class comment). */
    private final double[] circleNeed;
    /** Per zone: how much room the outline keeps from it (the safety margin, or WALL_GAP for a wall). */
    private final double[] outlineMargin;
    /** The robot's heading at the start, or NaN if it isn't known (then the heading at the point checked is used). */
    private final double startHeading;
    /** The end is meant to reach into a zone (the flower intake into a FLOWER): near it the outline may overlap as much as it does there. */
    private final boolean endMayOverlap;
    /** For the outline: per zone, the start's and the end's (center) distance to it. */
    private final double[] startAt, endAt;
    /** The outline (see AgateFlow.ROBOT_FRONT), read when the rule is made; front is longer with the flower intake down. */
    final double front, back, halfWidth;
    /**
     * How far the outline reaches ahead within startFrontRun inches of the start: longer
     * than front when the flower intake is still coming up there (see withStartFront).
     */
    private double startFront, startFrontRun = 0;
    /** A Route.allowContact stretch (see the class comment). */
    final boolean contact;
    private final boolean outline;
    private final double[] toward = new double[2];

    ClearanceRule(List<Zone> zones, double radius, double slope,
                  double startX, double startY, double endX, double endY) {
        this(zones, radius, slope, startX, startY, endX, endY, Double.NaN, AgateFlow.ROBOT_FRONT, false);
    }

    ClearanceRule(List<Zone> zones, double radius, double slope,
                  double startX, double startY, double endX, double endY, double startHeading, double front, boolean contact) {
        this(zones, radius, slope, startX, startY, endX, endY, startHeading, false, front, contact);
    }

    /**
     * @param startHeading  the robot's heading at the start, or NaN if not known
     * @param endMayOverlap the end is meant to reach into a zone (see the class comment)
     * @param front         how far the outline reaches ahead: ROBOT_FRONT, or the flower intake's reach when it is down
     * @param contact       a Route.allowContact stretch: the circle of {@code radius}, walls and zones alike
     */
    ClearanceRule(List<Zone> zones, double radius, double slope, double startX, double startY, double endX, double endY,
                  double startHeading, boolean endMayOverlap, double front, boolean contact) {
        this.zones = zones;
        this.radius = radius;
        this.contact = contact;
        this.slope = slope;
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.startHeading = startHeading;
        this.endMayOverlap = endMayOverlap;
        this.front = front;
        this.startFront = front;
        back = AgateFlow.ROBOT_BACK;
        halfWidth = AgateFlow.ROBOT_HALF_WIDTH;
        // The outline is used for the normal rule, not a contact one (Route.allowContact), which is about the circle.
        outline = AgateFlow.USE_OUTLINE && !contact;
        double margin = radius - AgateFlow.ROBOT_RADIUS;
        double wallMargin = margin > 1e-9 ? AgateFlow.WALL_GAP : 0;
        int count = zones.size();
        startAt = new double[count];
        endAt = new double[count];
        circleNeed = new double[count];
        outlineMargin = new double[count];
        startDistance = new double[count];
        endDistance = new double[count];
        for (int i = 0; i < count; i++) {
            Zone zone = zones.get(i);
            boolean wall = zone.wall && outline;
            circleNeed[i] = circleNeed(zone, radius, contact);
            outlineMargin[i] = wall ? wallMargin : margin;
            startAt[i] = zone.distance(startX, startY);
            endAt[i] = zone.distance(endX, endY);
            // The circle's floor: the robot itself (its half width beside a wall, else ROBOT_RADIUS).
            double floor = Math.min(circleNeed[i], wall ? AgateFlow.ROBOT_HALF_WIDTH : AgateFlow.ROBOT_RADIUS);
            startDistance[i] = startAt[i] < circleNeed[i] ? relaxed(startAt[i], floor) : Double.POSITIVE_INFINITY;
            endDistance[i] = endAt[i] < circleNeed[i] ? relaxed(endAt[i], floor) : Double.POSITIVE_INFINITY;
        }
    }

    /**
     * How far the robot's center stays from this zone with the circle rule of this
     * radius, away from the ends: the radius, except a wall lets it as close as its
     * side may come (see the class comment). The searches grow zones by this too.
     */
    static double circleNeed(Zone zone, double radius, boolean contact) {
        if (!zone.wall || contact || !AgateFlow.USE_OUTLINE) return radius;
        double wallRelief = AgateFlow.ROBOT_RADIUS + AgateFlow.SAFETY_MARGIN - AgateFlow.ROBOT_HALF_WIDTH - AgateFlow.WALL_GAP;
        return Math.max(radius - wallRelief, AgateFlow.ROBOT_HALF_WIDTH);
    }

    /** How close an end {@code at} inches from a zone lets the robot get there: END_SLACK closer, but not past {@code floor} if it wasn't already. */
    private static double relaxed(double at, double floor) {
        return at < floor ? at - END_SLACK : Math.max(at - END_SLACK, floor);
    }

    /** The same ends, a different radius (AgateFlow searches with a bigger one than it checks with). */
    ClearanceRule withRadius(double newRadius) {
        ClearanceRule rule = new ClearanceRule(zones, newRadius, slope, startX, startY, endX, endY, startHeading, endMayOverlap, front, contact);
        rule.startFront = startFront;
        rule.startFrontRun = startFrontRun;
        return rule;
    }

    /**
     * The same rule, with the outline reaching {@code reach} ahead at the start and up to
     * {@code run} inches from it: the flower intake is still coming up after a
     * Route.flowerIntakeDown stretch.
     */
    ClearanceRule withStartFront(double reach, double run) {
        ClearanceRule rule = withRadius(radius);
        rule.startFront = Math.max(front, reach);
        rule.startFrontRun = run;
        return rule;
    }

    /**
     * How far the robot's outline reaches from its center toward a zone that is in
     * direction (towardX, towardY) (a unit vector), when it faces {@code heading}.
     */
    double reach(double towardX, double towardY, double heading) {
        return reach(towardX, towardY, heading, front);
    }

    private double reach(double towardX, double towardY, double heading, double front) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double ahead = towardX * c + towardY * s, side = -towardX * s + towardY * c;
        return (ahead > 0 ? front : back) * Math.abs(ahead) + halfWidth * Math.abs(side);
    }

    /**
     * Exact room between the outline at (x, y) facing {@code heading} and the zones,
     * inches: the least over every zone (walls too). Negative: it overlaps one.
     */
    double outlineRoom(double x, double y, double heading) {
        double least = Double.POSITIVE_INFINITY;
        for (Zone zone : zones) least = Math.min(least, zone.outlineDistance(x, y, heading, front, back, halfWidth));
        return least;
    }

    /**
     * Room between the outline at (x, y) facing {@code heading} and the field's walls:
     * how close its nearest corner is to the nearest wall, inches. Exact (the walls
     * are the field's edges). Negative: it is past one.
     */
    double wallRoom(double x, double y, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double least = Double.POSITIVE_INFINITY;
        for (int k = 0; k < 4; k++) {
            double along = k < 2 ? front : -back, across = k % 2 == 0 ? halfWidth : -halfWidth;
            double cx = x + along * c - across * s, cy = y + along * s + across * c;
            least = Math.min(least, Math.min(Math.min(cx, Field.SIZE - cx), Math.min(cy, Field.SIZE - cy)));
        }
        return least;
    }

    /**
     * Spare room for the robot's real outline at (x, y) facing {@code heading}: how
     * much further it could move toward the closest zone before breaking the rule.
     * 0 or more is OK. The outline's room to a zone is first estimated past the line
     * through the zone's nearest point square to the way to it (never more than the
     * real room, exact when a flat side faces the zone), and measured exactly when
     * that estimate is within half an inch of the limit.
     * For a contact rule, or with USE_OUTLINE off, this is slack(x, y).
     */
    double slack(double x, double y, double heading) {
        if (!outline) return slack(x, y);
        double fromStart = Geometry.length(x - startX, y - startY), fromEnd = Geometry.length(x - endX, y - endY);
        double startFacing = Double.isNaN(startHeading) ? heading : startHeading;
        // Near the start the flower intake may still be down (see withStartFront).
        double front = fromStart < startFrontRun ? startFront : this.front;
        double most = Math.max(Math.hypot(front, halfWidth), Math.hypot(back, halfWidth));
        double least = Double.POSITIVE_INFINITY;
        for (int i = 0; i < zones.size(); i++) {
            Zone zone = zones.get(i);
            double margin = outlineMargin[i], floor = Math.min(margin, 0);
            // Far from this zone in any direction: plenty of room, skip the exact work.
            double far = zone.distanceAtLeast(x, y, margin + most + 1);
            if (far >= margin + most + 1) {
                least = Math.min(least, far - most - margin);
                continue;
            }
            double at = zone.distanceToward(x, y, toward);
            double room = at - (at > 0 ? reach(toward[0], toward[1], heading, front) : most);
            // Near an end that is itself close to the zone, the rule is relaxed from the room
            // there, measured exactly (so the estimate above only ever makes the rule stricter).
            double need = margin;
            if (startAt[i] < margin + most) {
                double startRoom = zone.outlineDistance(startX, startY, startFacing, startFront, back, halfWidth);
                if (startRoom < margin) need = Math.min(need, relaxed(startRoom, floor) + slope * fromStart);
            }
            if (endAt[i] < margin + most) {
                double endRoom = zone.outlineDistance(endX, endY, heading, this.front, back, halfWidth);
                // Near an end the outline may come as close as it is there (at this heading), but
                // never into the zone, unless the end is meant to reach in (endMayOverlap).
                double endNeed = relaxed(endRoom, floor) + slope * fromEnd;
                if (!endMayOverlap) endNeed = Math.max(endNeed, 0);
                if (endRoom < margin) need = Math.min(need, endNeed);
            }
            // The estimate is quick but can be pessimistic past a corner: when it is close to the
            // limit, measure exactly.
            if (room - need < 0.5) room = zone.outlineDistance(x, y, heading, front, back, halfWidth);
            least = Math.min(least, room - need);
        }
        return least;
    }

    /** How far the robot's center must stay from this zone when it is at (x, y). */
    double required(int zone, double x, double y) {
        double need = circleNeed[zone];
        if (startDistance[zone] != Double.POSITIVE_INFINITY) {
            need = Math.min(need, startDistance[zone] + slope * Geometry.length(x - startX, y - startY));
        }
        if (endDistance[zone] != Double.POSITIVE_INFINITY) {
            need = Math.min(need, endDistance[zone] + slope * Geometry.length(x - endX, y - endY));
        }
        return need;
    }

    /** How far the center stays from zone i with the circle, away from the ends (the searches grow the zones this much). */
    double circleNeed(int zone) {
        return circleNeed[zone];
    }

    /**
     * Spare room at (x, y): how much further the point could move toward the
     * closest zone before breaking the rule. 0 or more is OK. Values of 1 or more
     * are not exact (it stops looking once it knows there is at least 1 inch).
     */
    double slack(double x, double y) {
        double fromStart = Geometry.length(x - startX, y - startY), fromEnd = Geometry.length(x - endX, y - endY);
        double least = Double.POSITIVE_INFINITY;
        for (int i = 0; i < zones.size(); i++) {
            double need = Math.min(circleNeed[i], Math.min(startDistance[i] + slope * fromStart, endDistance[i] + slope * fromEnd));
            double room = zones.get(i).distanceAtLeast(x, y, need + 1) - need;
            if (room < least) least = room;
        }
        return least;
    }

    /** slack() for one zone only. */
    double slack(int zone, double x, double y) {
        double need = required(zone, x, y);
        return zones.get(zone).distanceAtLeast(x, y, need + 1) - need;
    }

    /** True if every point on the straight line from a to b meets the rule (checked every {@code step} inches). */
    boolean lineClear(double ax, double ay, double bx, double by, double step) {
        double length = Geometry.length(bx - ax, by - ay);
        int samples = Math.max(1, (int) Math.ceil(length / step));
        for (int i = 0; i <= samples; i++) {
            double t = (double) i / samples;
            if (slack(ax + t * (bx - ax), ay + t * (by - ay)) < -1e-6) return false;
        }
        return true;
    }
}
