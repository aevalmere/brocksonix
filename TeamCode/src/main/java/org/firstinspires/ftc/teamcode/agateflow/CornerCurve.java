package org.firstinspires.ftc.teamcode.agateflow;

import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.TValue;
import com.pedropathing.paths.curves.Curve;

/**
 * The rounded corner between two straight lines: a degree-5 Bezier curve that
 * starts and ends exactly along the lines with zero bend (curvature), so
 * line -> corner -> line never jerks the robot sideways (G2 continuous).
 *
 * Its six control points sit on the two lines: three on the way in, three on
 * the way out. Where they sit was found by search (see SHAPE_*) to give the
 * smallest peak bend for each corner angle: within about 3% of a perfect
 * circular arc of the same size, which can't be G2 at all.
 *
 * It implements Pedro's Curve, so Foresight follows it like any Pedro curve.
 * It is faster than Pedro's BezierCurve: points come from a plain polynomial,
 * and length lookups from a table built once.
 */
public final class CornerCurve implements Curve {
    /** Corner angles (degrees of turn) the shape table was found at. */
    private static final double[] SHAPE_ANGLE = {0, 30, 60, 90, 120, 150, 180};
    /** Second and third control points, as fractions of the run-in length from the corner. */
    private static final double[] SHAPE_OUTER = {0.86, 0.86, 0.84, 0.86, 0.88, 0.93, 0.97};
    private static final double[] SHAPE_INNER = {0.26, 0.27, 0.32, 0.39, 0.51, 0.69, 0.90};

    private static final int TABLE = 64;
    private static final double[] GAUSS_T = {0.5 - Math.sqrt(0.15), 0.5, 0.5 + Math.sqrt(0.15)};
    private static final double[] GAUSS_W = {5.0 / 18, 8.0 / 18, 5.0 / 18};

    /** Polynomial coefficients: x(t) = cx[0] + cx[1] t + ... + cx[5] t^5. */
    private final double[] cx = new double[6], cy = new double[6];
    /** Arc length from t = 0 to t = i / TABLE. */
    private final double[] lengthAt = new double[TABLE + 1];
    private final double length;

    /** Control points as x, y pairs (12 numbers). */
    public CornerCurve(double[] controls) {
        // Bernstein to power basis for degree 5.
        double[][] m = {
                {1, 0, 0, 0, 0, 0},
                {-5, 5, 0, 0, 0, 0},
                {10, -20, 10, 0, 0, 0},
                {-10, 30, -30, 10, 0, 0},
                {5, -20, 30, -20, 5, 0},
                {-1, 5, -10, 10, -5, 1}};
        for (int k = 0; k < 6; k++) {
            for (int i = 0; i < 6; i++) {
                cx[k] += m[k][i] * controls[2 * i];
                cy[k] += m[k][i] * controls[2 * i + 1];
            }
        }
        for (int i = 0; i < TABLE; i++) {
            double a = (double) i / TABLE, piece = 0;
            for (int g = 0; g < 3; g++) piece += GAUSS_W[g] * speed(a + GAUSS_T[g] / TABLE);
            lengthAt[i + 1] = lengthAt[i] + piece / TABLE;
        }
        length = lengthAt[TABLE];
    }

    /**
     * The corner at (cornerX, cornerY), coming in along direction (inX, inY) and
     * leaving along (outX, outY) (both unit length). The curve starts {@code runIn}
     * inches before the corner and ends {@code runIn} inches after it.
     */
    public static CornerCurve at(double cornerX, double cornerY, double inX, double inY,
                                 double outX, double outY, double runIn) {
        return new CornerCurve(controls(cornerX, cornerY, inX, inY, outX, outY, runIn));
    }

    /** The six control points of at(...), as x, y pairs. */
    static double[] controls(double cornerX, double cornerY, double inX, double inY,
                             double outX, double outY, double runIn) {
        double turn = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, inX * outX + inY * outY))));
        double outer = shape(SHAPE_OUTER, turn), inner = shape(SHAPE_INNER, turn);
        double[] fractions = {1, outer, inner};
        double[] c = new double[12];
        for (int i = 0; i < 3; i++) {
            c[2 * i] = cornerX - inX * runIn * fractions[i];
            c[2 * i + 1] = cornerY - inY * runIn * fractions[i];
            c[2 * (5 - i)] = cornerX + outX * runIn * fractions[i];
            c[2 * (5 - i) + 1] = cornerY + outY * runIn * fractions[i];
        }
        return c;
    }

    /** Point at t on the curve with these control points, into out[0], out[1]. Cheaper than building the curve. */
    static void point(double[] c, double t, double[] out) {
        double u = 1 - t;
        double b0 = u * u * u * u * u, b1 = 5 * t * u * u * u * u, b2 = 10 * t * t * u * u * u;
        double b3 = 10 * t * t * t * u * u, b4 = 5 * t * t * t * t * u, b5 = t * t * t * t * t;
        out[0] = b0 * c[0] + b1 * c[2] + b2 * c[4] + b3 * c[6] + b4 * c[8] + b5 * c[10];
        out[1] = b0 * c[1] + b1 * c[3] + b2 * c[5] + b3 * c[7] + b4 * c[9] + b5 * c[11];
    }

    private static double shape(double[] table, double turnDegrees) {
        double t = Math.max(0, Math.min(180, turnDegrees));
        int i = Math.min((int) (t / 30), SHAPE_ANGLE.length - 2);
        double f = (t - SHAPE_ANGLE[i]) / 30;
        return table[i] + f * (table[i + 1] - table[i]);
    }

    // ---- Raw polynomial ----

    double x(double t) {
        return ((((cx[5] * t + cx[4]) * t + cx[3]) * t + cx[2]) * t + cx[1]) * t + cx[0];
    }

    double y(double t) {
        return ((((cy[5] * t + cy[4]) * t + cy[3]) * t + cy[2]) * t + cy[1]) * t + cy[0];
    }

    double dx(double t) {
        return (((5 * cx[5] * t + 4 * cx[4]) * t + 3 * cx[3]) * t + 2 * cx[2]) * t + cx[1];
    }

    double dy(double t) {
        return (((5 * cy[5] * t + 4 * cy[4]) * t + 3 * cy[3]) * t + 2 * cy[2]) * t + cy[1];
    }

    double ddx(double t) {
        return ((20 * cx[5] * t + 12 * cx[4]) * t + 6 * cx[3]) * t + 2 * cx[2];
    }

    double ddy(double t) {
        return ((20 * cy[5] * t + 12 * cy[4]) * t + 6 * cy[3]) * t + 2 * cy[2];
    }

    private double speed(double t) {
        return Geometry.length(dx(t), dy(t));
    }

    /** Signed curvature (1/inches), positive turning left. */
    double bend(double t) {
        double vx = dx(t), vy = dy(t);
        double v = Geometry.length(vx, vy);
        if (v < 1e-9) return 0;
        return (vx * ddy(t) - vy * ddx(t)) / (v * v * v);
    }

    /** Arc length from the start to t. */
    double lengthTo(double t) {
        t = Math.max(0, Math.min(1, t));
        int i = Math.min((int) (t * TABLE), TABLE - 1);
        double a = (double) i / TABLE, span = t - a;
        if (span <= 0) return lengthAt[i];
        double piece = 0;
        for (int g = 0; g < 3; g++) piece += GAUSS_W[g] * speed(a + GAUSS_T[g] * span);
        return lengthAt[i] + piece * span;
    }

    /** The t that is this far along the curve. */
    double tAtLength(double s) {
        if (s <= 0) return 0;
        if (s >= length) return 1;
        int lo = 0, hi = TABLE;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (lengthAt[mid] <= s) lo = mid;
            else hi = mid;
        }
        double t = (lo + (s - lengthAt[lo]) / (lengthAt[hi] - lengthAt[lo])) / TABLE;
        for (int i = 0; i < 2; i++) {
            double v = speed(t);
            if (v < 1e-9) break;
            t = Math.max(0, Math.min(1, t - (lengthTo(t) - s) / v));
        }
        return t;
    }

    // ---- Pedro's Curve ----

    @Override
    public Vector2D get(double t) {
        TValue.check(t);
        return Vector2D.cartesian(x(t), y(t));
    }

    @Override
    public Vector2D derivative(double t) {
        TValue.check(t);
        return Vector2D.cartesian(dx(t), dy(t));
    }

    /** Direction of travel. Never fails: where the curve momentarily stops (a sharp U-turn), it looks just ahead. */
    @Override
    public Vector2D tangent(double t) {
        TValue.check(t);
        double vx = dx(t), vy = dy(t);
        for (int i = 1; i <= 4 && Geometry.length(vx, vy) < 1e-6; i++) {
            double near = t + 1e-3 * i <= 1 ? t + 1e-3 * i : t - 1e-3 * i;
            vx = dx(near);
            vy = dy(near);
        }
        double length = Geometry.length(vx, vy);
        if (length < 1e-12) return Vector2D.cartesian(1, 0);
        return Vector2D.cartesian(vx / length, vy / length);
    }

    @Override
    public double curvature(double t) {
        TValue.check(t);
        return bend(t);
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public double remainingDistance(double t) {
        TValue.check(t);
        return length - lengthTo(t);
    }

    @Override
    public double pathCompletion(double t) {
        TValue.check(t);
        return length == 0 ? 0 : lengthTo(t) / length;
    }

    @Override
    public double parameter(double pathCompletion) {
        TValue.check(pathCompletion);
        return tAtLength(pathCompletion * length);
    }

    @Override
    public double closestParameter(Vector2D position, double initialGuess) {
        double px = position.x(), py = position.y();
        // Best of the guess and 11 evenly spaced points, then Newton's method on the distance.
        double best = Math.max(0, Math.min(1, initialGuess));
        double bestDistance = distanceSquared(best, px, py);
        for (int i = 0; i <= 10; i++) {
            double t = i / 10.0, d = distanceSquared(t, px, py);
            if (d < bestDistance) {
                bestDistance = d;
                best = t;
            }
        }
        double t = best;
        for (int i = 0; i < 8; i++) {
            double ex = x(t) - px, ey = y(t) - py;
            double vx = dx(t), vy = dy(t);
            double slope = ex * vx + ey * vy;
            double curve = vx * vx + vy * vy + ex * ddx(t) + ey * ddy(t);
            if (curve <= 1e-12) break;
            double next = Math.max(0, Math.min(1, t - slope / curve));
            if (Math.abs(next - t) < 1e-9) {
                t = next;
                break;
            }
            t = next;
        }
        return distanceSquared(t, px, py) <= bestDistance ? t : best;
    }

    private double distanceSquared(double t, double px, double py) {
        double ex = x(t) - px, ey = y(t) - py;
        return ex * ex + ey * ey;
    }

    @Override
    public Vector2D startPoint() {
        return Vector2D.cartesian(cx[0], cy[0]);
    }

    @Override
    public Vector2D endPoint() {
        return get(1);
    }
}
