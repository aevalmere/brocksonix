package org.firstinspires.ftc.teamcode.agateflow;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Field;

/**
 * How the robot turns on the way to a waypoint. Set it with Route.heading(...)
 * right after the waypoint it applies to.
 *
 * <ul>
 * <li>{@link #linear()} (the default): turns evenly from the old heading to the waypoint's.</li>
 * <li>{@link #finishBy(double)} / {@link #between(double, double)}: turns only during part of the way,
 *     e.g. finishBy(0.3) is facing the new way 30% in. Turning early leaves the end straight,
 *     which is good for intaking or lining up a shot.</li>
 * <li>{@link #tangent()} / {@link #reverseTangent()}: faces (or backs) along the direction of travel.
 *     The robot drives forward, its fastest direction. The waypoint's heading is then the direction
 *     it arrives in.</li>
 * <li>{@link #facing(double, double)}: keeps facing one point (written for red), like the HIVE.</li>
 * </ul>
 * Turning always takes the shorter way around.
 */
public final class Heading {
    enum Kind { TURN, TANGENT, REVERSE_TANGENT, FACING }

    final Kind kind;
    /** For TURN: the part of the way, 0 to 1, over which the turn happens. */
    final double from, to;
    /** For FACING: the point, in the coordinates the Heading was made in. */
    final double x, y;

    private Heading(Kind kind, double from, double to, double x, double y) {
        this.kind = kind;
        this.from = from;
        this.to = to;
        this.x = x;
        this.y = y;
    }

    public static Heading linear() {
        return new Heading(Kind.TURN, 0, 1, 0, 0);
    }

    /** Finishes the turn this fraction of the way in (0 to 1), then holds it. */
    public static Heading finishBy(double fraction) {
        return between(0, fraction);
    }

    /** Turns only between these fractions of the way (0 to 1). Before, holds the old heading. After, the new one. */
    public static Heading between(double startFraction, double endFraction) {
        // The turn needs some length (at() divides by it), so it starts by 0.999 at the latest.
        double a = Math.max(0, Math.min(1 - 1e-3, startFraction));
        double b = Math.max(0, Math.min(1, endFraction));
        if (b <= a) b = a + 1e-3;
        return new Heading(Kind.TURN, a, b, 0, 0);
    }

    public static Heading tangent() {
        return new Heading(Kind.TANGENT, 0, 1, 0, 0);
    }

    public static Heading reverseTangent() {
        return new Heading(Kind.REVERSE_TANGENT, 0, 1, 0, 0);
    }

    /** Always faces this point. Written for red, like waypoint poses. */
    public static Heading facing(double x, double y) {
        return new Heading(Kind.FACING, 0, 1, x, y);
    }

    Heading forAlliance(Alliance alliance) {
        if (kind != Kind.FACING || alliance != Alliance.BLUE) return this;
        return new Heading(kind, from, to, Field.SIZE - x, Field.SIZE - y);
    }

    /**
     * The heading part way along.
     *
     * @param fraction     how far along this stretch, 0 to 1, by distance
     * @param start        heading at the start of the stretch
     * @param turn         signed turn from start to the waypoint's heading (shortest way)
     * @param travelAngle  direction of travel here
     * @param px           where the robot is
     * @param py           where the robot is
     */
    double at(double fraction, double start, double turn, double travelAngle, double px, double py) {
        switch (kind) {
            case TANGENT:
                return travelAngle;
            case REVERSE_TANGENT:
                return travelAngle + Math.PI;
            case FACING:
                return Math.atan2(y - py, x - px);
            case TURN:
            default:
                double f = (fraction - from) / (to - from);
                return start + turn * Math.max(0, Math.min(1, f));
        }
    }
}
