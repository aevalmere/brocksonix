package org.firstinspires.ftc.teamcode.util;

public final class LookupTable {
    private LookupTable() {}

    /**
     * Reads a value from a table of {x, y} rows sorted by x, drawing a straight
     * line between the two nearest rows. Past either end it returns the end value.
     */
    public static double at(double[][] rows, double x) {
        if (x <= rows[0][0]) return rows[0][1];
        int last = rows.length - 1;
        if (x >= rows[last][0]) return rows[last][1];

        for (int i = 1; i <= last; i++) {
            if (x <= rows[i][0]) {
                double x0 = rows[i - 1][0], y0 = rows[i - 1][1];
                double x1 = rows[i][0], y1 = rows[i][1];
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
            }
        }
        return rows[last][1];
    }

    public static double minX(double[][] rows) {
        return rows[0][0];
    }

    public static double maxX(double[][] rows) {
        return rows[rows.length - 1][0];
    }
}
