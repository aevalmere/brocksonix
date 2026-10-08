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
}
