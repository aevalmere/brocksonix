package org.firstinspires.ftc.teamcode.util;

public final class Angles {
    private Angles() {}

    /** Wraps to [-180, 180]. Use it on a difference to get the shortest way around. */
    public static double wrapDegrees(double degrees) {
        return Math.IEEEremainder(degrees, 360);
    }

    /** Wraps to [-π, π]. */
    public static double wrapRadians(double radians) {
        return Math.IEEEremainder(radians, 2 * Math.PI);
    }
}
