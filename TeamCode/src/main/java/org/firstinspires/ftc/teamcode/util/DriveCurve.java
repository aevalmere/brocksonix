package org.firstinspires.ftc.teamcode.util;

/**
 * Stick shaping: small stick movements give fine control, full stick still
 * gives full speed.
 *
 * 1. Deadband: anything under it counts as 0, and the rest of the stick is
 *    stretched so the output starts from 0 right past the deadband (no jump).
 * 2. Cubic blend: out = w * x³ + (1 - w) * x. w = 0 is a straight line,
 *    w = 1 is a full cube. Most teams land around 0.5 to 0.8.
 */
public final class DriveCurve {
    private DriveCurve() {}

    public static double shape(double stick, double deadband, double cubicWeight) {
        double magnitude = Math.min(Math.abs(stick), 1);
        if (magnitude <= deadband) return 0;
        double x = (magnitude - deadband) / (1 - deadband);
        double shaped = cubicWeight * x * x * x + (1 - cubicWeight) * x;
        return Math.copySign(shaped, stick);
    }
}
